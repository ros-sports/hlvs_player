#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <json/value.h>
#include <jsoncpp/json/json.h>
#include <fstream>
#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/executors/events_executor/events_executor.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include "hlvs_player/network_client.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class WebotsController : public rclcpp::Node
{
public:
  WebotsController() : Node("hlvs_player")
  {
    // Parameters
    this->declare_parameter<std::string>("host", "127.0.0.1");
    this->declare_parameter<int>("port", 10001);
    this->declare_parameter<std::string>("devices_file", "resources/devices.json");

    // Enable devices
    ActuatorRequests request;
    Json::Value devices;
    std::ifstream json_file(this->get_parameter("devices_file").as_string());

    // Check if file exists
    if (!json_file.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Could not open devices file. Please check the 'devices_file' parameter.");
      return;
    }
    json_file >> devices;

    // Publishers
    clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", 10);
    real_clock_publisher_ = this->create_publisher<sensor_msgs::msg::TimeReference>("server_time_clock", 10);
    camera_image_publishers_ = {};
    camera_info_publishers_ = {};
    imu_publishers_ = {};

    // Subscriptions
    motor_command_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_command", 10, std::bind(&WebotsController::command_callback, this, _1));

    // Timer and its callback
    // simulation does a step, it does not really make sense to run this in any other frequency
    timer_ = this->create_wall_timer(8ms, std::bind(&WebotsController::timer_callback, this));

    // Client construction and connecting
    std::string host;
    int port;
    this->get_parameter("host", host);
    this->get_parameter("port", port);
    client_ = new NetworkClient(host, port, 3, this->get_logger());
    client_->connectClient();

    // Joints
    map_proto_to_ros_ = {};
    map_ros_to_proto_ = {};
    for (unsigned int i = 0; i < devices["joints"].size(); i++)
    {
      SensorTimeStep *sensor = request.add_sensor_time_steps();
      std::string sensor_name = devices["joints"][i]["proto_sensor_name"].asString();
      std::string motor_name = devices["joints"][i]["proto_motor_name"].asString();
      sensor->set_name(sensor_name);
      sensor->set_timestep(devices["joints"][i]["time_step"].asDouble());
      std::string ros_name = devices["joints"][i]["ros_name"].asString();
      // we might need to change the name between the proto and ros (e.g. because there is a [shoulder] in the joint
      // name of the proto)
      if (ros_name != "")
      {
        map_proto_to_ros_.insert({sensor_name, ros_name});
        map_ros_to_proto_.insert({ros_name, motor_name});
      }
      else
      {
        // no change necessary
        map_proto_to_ros_.insert({sensor_name, sensor_name});
        map_ros_to_proto_.insert({motor_name, motor_name});
      }
    }
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Bumpers
    for (unsigned int i = 0; i < devices["bumpers"].size(); i++)
    {
      std::string frame_name = devices["bumpers"][i]["frame_name"].asString();
      bumper_frame_names_.push_back(frame_name);
      std::string frame_axis = devices["bumpers"][i]["frame_axis"].asString();
      if (frame_axis == "x" || frame_axis == "y" || frame_axis == "z")
      {
        bumper_frame_axis_.push_back(frame_axis);
      }
      else
      {
        bumper_frame_axis_.push_back("x");
        RCLCPP_WARN_STREAM(this->get_logger(), "Bumper frame axis for frame " << frame_name << " not correctly specified. Must be 'x', 'y', or 'z'.");
      }

      SensorTimeStep *bumper_sensor = request.add_sensor_time_steps();
      bumper_sensor->set_name(devices["bumpers"][i]["proto_bumper_name"].asString());
      bumper_sensor->set_timestep(devices["bumpers"][i]["time_step"].asDouble());
      bumper_publishers_.push_back(
          this->create_publisher<geometry_msgs::msg::WrenchStamped>(
            devices["bumpers"][i]["proto_bumper_name"].asString() + "/data", 10));
    }

    // Force sensors 1d
    for (unsigned int i = 0; i < devices["force_sensors_1d"].size(); i++)
    {
      std::string frame_name = devices["force_sensors_1d"][i]["frame_name"].asString();
      force1d_frame_names_.push_back(frame_name);
      std::string frame_axis = devices["force_sensors_1d"][i]["frame_axis"].asString();
      if (frame_axis == "x" || frame_axis == "y" || frame_axis == "z")
      {
        force1d_frame_axis_.push_back(frame_axis);
      }
      else
      {
        force1d_frame_axis_.push_back("x");
        RCLCPP_WARN_STREAM(this->get_logger(), "Force1d sensor frame axis for frame " << frame_name << " not correctly specified. Must be 'x', 'y', or 'z'.");
      }

      SensorTimeStep *force1d_sensor = request.add_sensor_time_steps();
      force1d_sensor->set_name(devices["force_sensors_1d"][i]["proto_sensor_name"].asString());
      force1d_sensor->set_timestep(devices["force_sensors_1d"][i]["time_step"].asDouble());
      force1d_publishers_.push_back(
          this->create_publisher<geometry_msgs::msg::WrenchStamped>(
            devices["force_sensors_1d"][i]["proto_sensor_name"].asString() + "/data", 10));
    }

    // Force sensors 3d
    for (unsigned int i = 0; i < devices["force_sensors_3d"].size(); i++)
    {
      std::string frame_name = devices["force_sensors_3d"][i]["frame_name"].asString();
      force3d_frame_names_.push_back(frame_name);
      SensorTimeStep *force3d_sensor = request.add_sensor_time_steps();
      force3d_sensor->set_name(devices["force_sensors_3d"][i]["proto_sensor_name"].asString());
      force3d_sensor->set_timestep(devices["force_sensors_3d"][i]["time_step"].asDouble());
      force3d_publishers_.push_back(
          this->create_publisher<geometry_msgs::msg::WrenchStamped>(
            devices["force_sensors_3d"][i]["proto_sensor_name"].asString() + "/data" , 10));
    }

    // Force sensors 6d
    for (unsigned int i = 0; i < devices["force_sensors_6d"].size(); i++)
    {
      std::string frame_name = devices["force_sensors_6d"][i]["frame_name"].asString();
      force6d_frame_names_.push_back(frame_name);
      SensorTimeStep *force6d_sensor = request.add_sensor_time_steps();
      force6d_sensor->set_name(devices["force_sensors_6d"][i]["proto_sensor_name"].asString());
      force6d_sensor->set_timestep(devices["force_sensors_6d"][i]["time_step"].asDouble());
      force6d_publishers_.push_back(
          this->create_publisher<geometry_msgs::msg::WrenchStamped>(
            devices["force_sensors_6d"][i]["proto_sensor_name"].asString() + "/data", 10));
    }

    // Cameras
    for (unsigned int i = 0; i < devices["cameras"].size(); i++)
    {
      std::string frame_name = devices["cameras"][i]["optical_frame"].asString();
      camera_frame_names_.push_back(frame_name);
      SensorTimeStep *camera_sensor = request.add_sensor_time_steps();
      camera_sensor->set_name(devices["cameras"][i]["proto_camera_name"].asString());
      camera_sensor->set_timestep(devices["cameras"][i]["time_step"].asDouble());
      camera_image_publishers_.push_back(
          this->create_publisher<sensor_msgs::msg::Image>(
            devices["cameras"][i]["proto_camera_name"].asString() + "/image_raw", 10));
      camera_info_publishers_.push_back(this->create_publisher<sensor_msgs::msg::CameraInfo>(
          devices["cameras"][i]["proto_camera_name"].asString() + "/camera_info", 10));
      camera_fovs_.push_back(devices["cameras"][i]["horizontal_field_of_view"].asDouble());
    }

    // IMUs
    imu_frame_names_ = {};
    for (unsigned int i = 0; i < devices["IMUs"].size(); i++)
    {
      imu_frame_names_.push_back(devices["IMUs"][i]["frame_name"].asString());
      // gyro
      SensorTimeStep *sensor = request.add_sensor_time_steps();
      sensor->set_name(devices["IMUs"][i]["proto_gyro_name"].asString());
      sensor->set_timestep(devices["IMUs"][i]["time_step"].asDouble());
      // accel
      sensor = request.add_sensor_time_steps();
      sensor->set_name(devices["IMUs"][i]["proto_accel_name"].asString());
      sensor->set_timestep(devices["IMUs"][i]["time_step"].asDouble());
      imu_publishers_.push_back(
          this->create_publisher<sensor_msgs::msg::Imu>(
            devices["IMUs"][i]["ros_name"].asString() + "/data", 10));
    }

    client_->sendRequest(request);
    SensorMeasurements measurements = client_->receive();
  }

private:
  void timer_callback()
  {
    if (client_->isOk())
    {
      try
      {
        ActuatorRequests request;
        client_->sendRequest(request);
        SensorMeasurements measurements = client_->receive();

        // publish simulation time
        auto clk = rosgraph_msgs::msg::Clock();
        clk.clock = ms_to_ros_time(measurements.time());
        clock_publisher_->publish(clk);
        // publish wall clock time of server
        auto ref_clk = sensor_msgs::msg::TimeReference();
        ref_clk.header.stamp = ms_to_ros_time(measurements.real_time());
        ref_clk.time_ref = ms_to_ros_time(measurements.time());
        ref_clk.source = "server";
        real_clock_publisher_->publish(ref_clk);

        handle_messages(measurements);
        publishBumpers(measurements);
        publishForce1d(measurements);
        publishForce3d(measurements);
        publishForce6d(measurements);
        publishImage(measurements);
        publishIMUs(measurements);
        publishJointStates(measurements);
      }
      catch (const std::runtime_error &exc)
      {
        std::cerr << "Runtime error: " << exc.what() << std::endl;
      }
    }
  }

  void handle_messages(const SensorMeasurements &measurements)
  {
    for (int i = 0; i < measurements.messages_size(); i++)
    {
      std::string text = measurements.messages(i).text();
      if (measurements.messages(i).message_type() == Message::ERROR_MESSAGE)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "RECEIVED ERROR: " << text);
      }
      else if (measurements.messages(i).message_type() == Message::WARNING_MESSAGE)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "RECEIVED WARNING: " << text);
      }
      else
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "RECEIVED UNKNOWN MESSAGE: " << text);
      }
    }
  }

  void publishBumpers(const SensorMeasurements &measurements)
  { // todo we should always check the names rather than relying on the numbering as this could maybe differ if not all have the same timestep??
    for (int i = 0; i < measurements.bumpers_size(); i++)
    {
      auto wrench = geometry_msgs::msg::WrenchStamped();
      wrench.header.stamp = ms_to_ros_time(measurements.time());
      wrench.header.frame_id = bumper_frame_names_[i];
      bool bumper_active = measurements.bumpers(i).value();
      // we encode the binary bumper force as 1N as this can then be displayed in RViz
      float force;
      if (bumper_active)
      {
        force = 1;
      }
      else
      {
        force = 0;
      }

      // set the force for the specified axis
      if (bumper_frame_axis_[i] == "x")
      {
        wrench.wrench.force.x = force;
      }
      else if (bumper_frame_axis_[i] == "y")
      {
        wrench.wrench.force.y = force;
      }
      else
      {
        wrench.wrench.force.z = force;
      }
      bumper_publishers_[i]->publish(wrench);
    }
  }

  void publishForce1d(const SensorMeasurements &measurements)
  {
    for (int i = 0; i < measurements.forces_size(); i++)
    {
      auto wrench = geometry_msgs::msg::WrenchStamped();
      wrench.header.stamp = ms_to_ros_time(measurements.time());
      wrench.header.frame_id = force1d_frame_names_[i];
      float force = measurements.forces(i).value();

      // set the force for the specified axis
      if (force1d_frame_axis_[i] == "x")
      {
        wrench.wrench.force.x = force;
      }
      else if (force1d_frame_axis_[i] == "-x")
      {
        wrench.wrench.force.x = -force;
      }
      else if (force1d_frame_axis_[i] == "y")
      {
        wrench.wrench.force.y = force;
      }
      else if (force1d_frame_axis_[i] == "-y")
      {
        wrench.wrench.force.y = -force;
      }
      else if (force1d_frame_axis_[i] == "z")
      {
        wrench.wrench.force.z = force;
      }
      else if (force1d_frame_axis_[i] == "-z")
      {
        wrench.wrench.force.z = -force;
      }
      else
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "DEFINITION OF FORCE SENSOR " << measurements.forces(i).name() << " WRONG");
      }
      force1d_publishers_[i]->publish(wrench);
    }
  }

  void publishForce3d(const SensorMeasurements &measurements)
  {
    for (int i = 0; i < measurements.force3ds_size(); i++)
    {
      auto wrench = geometry_msgs::msg::WrenchStamped();
      wrench.header.stamp = ms_to_ros_time(measurements.time());
      wrench.header.frame_id = force3d_frame_names_[i];
      wrench.wrench.force.x = measurements.force3ds(i).value().x();
      wrench.wrench.force.y = measurements.force3ds(i).value().y();
      wrench.wrench.force.z = measurements.force3ds(i).value().z();
      force3d_publishers_[i]->publish(wrench);
    }
  }

  void publishForce6d(const SensorMeasurements &measurements)
  {
    for (int i = 0; i < measurements.force6ds_size(); i++)
    {
      auto wrench = geometry_msgs::msg::WrenchStamped();
      wrench.header.stamp = ms_to_ros_time(measurements.time());
      wrench.header.frame_id = force6d_frame_names_[i];
      wrench.wrench.force.x = measurements.force6ds(i).force().x();
      wrench.wrench.force.y = measurements.force6ds(i).force().y();
      wrench.wrench.force.z = measurements.force6ds(i).force().z();
      wrench.wrench.torque.x = measurements.force6ds(i).torque().x();
      wrench.wrench.torque.y = measurements.force6ds(i).torque().y();
      wrench.wrench.torque.z = measurements.force6ds(i).torque().z();
      force6d_publishers_[i]->publish(wrench);
    }
  }

  void publishImage(const SensorMeasurements &measurements)
  {
    // iterate over the used cameras
    for (int i = 0; i < measurements.cameras_size(); i++)
    {
      const CameraMeasurement &sensor_data = measurements.cameras(i);
      if (sensor_data.quality() != -1)
      {
        throw std::runtime_error("Encoded images are not supported in this client");
      }

      cv::Mat img(sensor_data.height(), sensor_data.width(), CV_8UC3, (void *)sensor_data.image().c_str());

      auto imgmsg = sensor_msgs::msg::Image();
      cv_bridge::CvImage img_bridge;
      img_bridge = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, img);
      img_bridge.toImageMsg(imgmsg);
      imgmsg.header.stamp = ms_to_ros_time(measurements.time());
      imgmsg.header.frame_id = camera_frame_names_[i];
      camera_image_publishers_[i]->publish(imgmsg);

      // calculate and publish the camera info
      sensor_msgs::msg::CameraInfo camera_info_msg = sensor_msgs::msg::CameraInfo();
      camera_info_msg.header = imgmsg.header;
      int height = sensor_data.height();
      int width = sensor_data.width();
      camera_info_msg.height = height;
      camera_info_msg.width = width;
      double f_y = mat_from_fov_and_resolution(h_fov_to_v_fov(camera_fovs_[i], height, width), height);
      double f_x = mat_from_fov_and_resolution(camera_fovs_[i], width);
      camera_info_msg.k = {f_x, 0.0, width / 2.0, 0.0, f_y, height / 2.0, 0.0, 0.0, 1.0};
      camera_info_msg.p = {f_x, 0.0, width / 2.0, 0.0, 0.0, f_y, height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0};
      camera_info_publishers_[i]->publish(camera_info_msg);
    }
  }

  void publishIMUs(const SensorMeasurements &measurements)
  {
    // iterate over the used IMUs
    for (int i = 0; i < measurements.accelerometers_size(); i++)
    {
      const AccelerometerMeasurement &accel_data = measurements.accelerometers(i);
      const GyroMeasurement &gyro_data = measurements.gyros(i);

      auto imu_msg = sensor_msgs::msg::Imu();
      imu_msg.header.stamp = ms_to_ros_time(measurements.time());
      imu_msg.header.frame_id = imu_frame_names_[i];
      imu_msg.linear_acceleration.x = accel_data.value().x();
      imu_msg.linear_acceleration.y = accel_data.value().y();
      imu_msg.linear_acceleration.z = accel_data.value().z();
      imu_msg.angular_velocity.x = gyro_data.value().x();
      imu_msg.angular_velocity.y = gyro_data.value().y();
      imu_msg.angular_velocity.z = gyro_data.value().z();
      // we can not get the orientation from the simulation, still make sure
      // that there is at least a valid quaternion in the message, otherwise it may lead to errors in RViz2
      imu_msg.orientation.w = 1;
      imu_publishers_[i]->publish(imu_msg);
    }
  }

  void publishJointStates(const SensorMeasurements &measurements)
  {
    auto jointmsg = sensor_msgs::msg::JointState();
    for (int i = 0; i < measurements.position_sensors_size(); i++)
    {
      std::string ros_name = map_proto_to_ros_[measurements.position_sensors(i).name()];
      jointmsg.name.push_back(ros_name);
      jointmsg.position.push_back(measurements.position_sensors(i).value());
    }
    jointmsg.header.stamp = ms_to_ros_time(measurements.time());
    joint_state_publisher_->publish(jointmsg);
  }

  void command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    ActuatorRequests request;

    for (unsigned int i = 0; i < msg->name.size(); i++)
    {
      std::string name = msg->name[i].c_str();
      if (!map_ros_to_proto_.count(name))
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Got joint command with unknown joint " << name.c_str());
        continue;
      }
      std::string proto_name = map_ros_to_proto_[name];
      if (msg->position.size() == 0)
      {
        // no positions give, will use torque control
        MotorTorque *motor_torque = request.add_motor_torques();
        motor_torque->set_name(proto_name);
        motor_torque->set_torque(msg->effort[i]);
      }
      else
      {
        // use position and velocity
        MotorPosition *motor_position = request.add_motor_positions();
        motor_position->set_name(proto_name);
        motor_position->set_position(msg->position[i]);
        if (msg->velocity.size() != 0)
        {
          // if additional velocity is give, use it as maximal speed
          MotorVelocity *motor_velocity = request.add_motor_velocities();
          motor_velocity->set_name(proto_name);
          motor_velocity->set_velocity(msg->velocity[i]);
        }
      }
    }
    client_->sendRequest(request);
  }

  double mat_from_fov_and_resolution(double fov, double res)
  {
    return 0.5 * res * (cos((fov / 2)) / sin((fov / 2)));
  }
  double h_fov_to_v_fov(double h_fov, int height, int width)
  {
    return 2 * atan(tan(h_fov * 0.5) * (height / width));
  }

  rclcpp::Time ms_to_ros_time(u_int32_t ms)
  {
    return rclcpp::Time(ms / 1000, (ms % 1000) * 1000000);
  }

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr real_clock_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr> bumper_publishers_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr> force1d_publishers_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr> force3d_publishers_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr> force6d_publishers_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> camera_image_publishers_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_publishers_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> imu_publishers_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_command_subscription_;

  std::map<std::string, std::string> map_proto_to_ros_;
  std::map<std::string, std::string> map_ros_to_proto_;
  std::vector<std::string> bumper_frame_names_;
  std::vector<std::string> bumper_frame_axis_;
  std::vector<std::string> force1d_frame_names_;
  std::vector<std::string> force1d_frame_axis_;
  std::vector<std::string> force3d_frame_names_;
  std::vector<std::string> force6d_frame_names_;
  std::vector<std::string> camera_frame_names_;
  std::vector<std::string> imu_frame_names_;
  std::vector<double> camera_fovs_;
  NetworkClient *client_;
};

int main(int argc, char *argv[])
{
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WebotsController>();

  // we use the event executor since it greatly reduces the CPU usage
  rclcpp::executors::EventsExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}

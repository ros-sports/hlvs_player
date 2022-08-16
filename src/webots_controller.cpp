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
#include <std_msgs/msg/string.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>

#include "ros_bridge/robot_client/robot_client.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class WebotsController : public rclcpp::Node {
public:
  WebotsController() : Node("webots_controller") {
    // Parameters
    this->declare_parameter<std::string>("host", "127.0.0.1");
    this->declare_parameter<int>("port", 10001);

    // Enable devices
    ActuatorRequests request;
    Json::Value devices;
    std::ifstream json_file("resources/devices.json");
    json_file >> devices;

    // Publishers
    clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", 10);
    // todo use this instead? https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/TimeReference.msg
    real_clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("server_time_clock", 10);
    camera_image_publishers_ = {};
    camera_info_publishers_ = {};
    imu_publishers_ = {};

    // Subscriptions
    motor_command_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        devices["joint_command_topic_name"].asString(), 10, std::bind(&WebotsController::command_callback, this, _1));

    // Timer and its callback
    // simulation does a step, it does not really make sense to run this in any other frequency
    timer_ = this->create_wall_timer(8ms, std::bind(&WebotsController::timer_callback, this));

    // Client construction and connecting
    std::string host;
    int port;
    this->get_parameter("host", host);
    this->get_parameter("port", port);
    client = new RobotClient(host, port, 3);
    client->connectClient();

    // Joints
    map_proto_to_ros = {};
    map_ros_to_proto = {};
    for (unsigned int i = 0; i < devices["joints"].size(); i++) {
      SensorTimeStep* sensor = request.add_sensor_time_steps();
      std::string sensor_name = devices["joints"][i]["proto_sensor_name"].asString();
      std::string motor_name = devices["joints"][i]["proto_motor_name"].asString();
      sensor->set_name(sensor_name);
      sensor->set_timestep(devices["joints"][i]["time_step"].asDouble());
      std::string ros_name = devices["joints"][i]["ros_name"].asString();
      // we might need to change the name between the proto and ros (e.g. because there is a [shoulder] in the joint
      // name of the proto)
      if (ros_name != "") {
        map_proto_to_ros.insert({sensor_name, ros_name});
        map_ros_to_proto.insert({ros_name, motor_name});
      } else {
        // no change necessary
        map_proto_to_ros.insert({sensor_name, sensor_name});
        map_ros_to_proto.insert({motor_name, motor_name});
      }
    }
    joint_state_publisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>(devices["joint_state_topic_name"].asString(), 10);

    // Cameras
    for (unsigned int i = 0; i < devices["cameras"].size(); i++) {
      std::string frame_name = devices["cameras"][i]["optical_frame"].asString();
      camera_frame_names_.push_back(frame_name);
      SensorTimeStep* camera_sensor = request.add_sensor_time_steps();
      camera_sensor->set_name(devices["cameras"][i]["proto_camera_name"].asString());
      camera_sensor->set_timestep(devices["cameras"][i]["time_step"].asDouble());
      camera_image_publishers_.push_back(
          this->create_publisher<sensor_msgs::msg::Image>(devices["cameras"][i]["image_topic_name"].asString(), 10));

      // camera info should be latched and only published once
      rclcpp::QoS qos(rclcpp::KeepLast(1));
      qos.transient_local().reliable();
      camera_info_publishers_.push_back(this->create_publisher<sensor_msgs::msg::CameraInfo>(
          devices["cameras"][i]["info_topic_name"].asString(), qos));
      // calculate and publish the camera info
      sensor_msgs::msg::CameraInfo camera_info_msg = sensor_msgs::msg::CameraInfo();
      camera_info_msg.header.frame_id = frame_name;
      int height = devices["cameras"][i]["height"].asInt();
      int width = devices["cameras"][i]["width"].asInt();
      double FOV = devices["cameras"][i]["FOV"].asDouble();
      camera_info_msg.height = height;
      camera_info_msg.width = width;
      double f_y = mat_from_fov_and_resolution(h_fov_to_v_fov(FOV, height, width), height);
      double f_x = mat_from_fov_and_resolution(FOV, width);
      camera_info_msg.k = {f_x, 0.0, width / 2.0, 0.0, f_y, height / 2.0, 0.0, 0.0, 1.0};
      camera_info_msg.p = {f_x, 0.0, width / 2.0, 0.0, 0.0, f_y, height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0};
      camera_info_publishers_[i]->publish(camera_info_msg);
    }

    // IMUs
    imu_frame_names_ = {};
    for (unsigned int i = 0; i < devices["IMUs"].size(); i++) {
      imu_frame_names_.push_back(devices["IMUs"][i]["frame_name"].asString());
      // gyro
      SensorTimeStep* sensor = request.add_sensor_time_steps();
      sensor->set_name(devices["IMUs"][i]["proto_gyro_name"].asString());
      sensor->set_timestep(devices["IMUs"][i]["time_step"].asDouble());
      // accel
      sensor = request.add_sensor_time_steps();
      sensor->set_name(devices["IMUs"][i]["proto_accel_name"].asString());
      sensor->set_timestep(devices["IMUs"][i]["time_step"].asDouble());
      imu_publishers_.push_back(
          this->create_publisher<sensor_msgs::msg::Imu>(devices["IMUs"][i]["topic_name"].asString(), 10));
    }

    client->sendRequest(request);
    SensorMeasurements measurements = client->receive();
  }

private:
  void timer_callback() {
    if (client->isOk()) {
      try {
        ActuatorRequests request;
        client->sendRequest(request);
        SensorMeasurements measurements = client->receive();

        // publish simulation time
        auto clk = rosgraph_msgs::msg::Clock();
        clk.clock = rclcpp::Time(measurements.time());
        clock_publisher_->publish(clk);
        // publish wall clock time of server
        clk.clock = rclcpp::Time(measurements.real_time());
        real_clock_publisher_->publish(clk);

        handle_messages(measurements);
        publishBumpers(measurements);
        publishForce1d(measurements);
        publishForce3d(measurements);
        publishForce6d(measurements);
        publishImage(measurements);
        publishIMUs(measurements);
        publishJointStates(measurements);
      } catch (const std::runtime_error& exc) {
        std::cerr << "Runtime error: " << exc.what() << std::endl;
      }
    }
  }

  void handle_messages(const SensorMeasurements& measurements) {
    for (int i = 0; i < measurements.messages_size(); i++) {
      std::string text = measurements.messages(i).text();
      if (measurements.messages(i).message_type() == Message::ERROR_MESSAGE) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "RECEIVED ERROR: " << text);
      } else if (measurements.messages(i).message_type() == Message::WARNING_MESSAGE) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "RECEIVED WARNING: " << text);
      } else {
        RCLCPP_ERROR_STREAM(this->get_logger(), "RECEIVED UNKNOWN MESSAGE: " << text);
      }
    }
  }

  void publishBumpers(const SensorMeasurements& measurements) {
    // todo
    for (int i = 0; i < measurements.bumpers_size(); i++) {
      RCLCPP_WARN_STREAM(this->get_logger(), "bumpers not implemented yet");
    }
  }

  void publishForce1d(const SensorMeasurements& measurements) {
    // todo
    for (int i = 0; i < measurements.forces_size(); i++) {
      RCLCPP_WARN_STREAM(this->get_logger(), "force 1d sensors not implemented yet");
    }
  }

  void publishForce3d(const SensorMeasurements& measurements) {
    // todo
    for (int i = 0; i < measurements.force3ds_size(); i++) {
      RCLCPP_WARN_STREAM(this->get_logger(), "force 3d sensors not implemented yet");
    }
  }

  void publishForce6d(const SensorMeasurements& measurements) {
    // todo
    for (int i = 0; i < measurements.force6ds_size(); i++) {
      RCLCPP_WARN_STREAM(this->get_logger(), "force 6d sensors not implemented yet");
    }
  }

  void publishImage(const SensorMeasurements& measurements) {
    // iterate over the used cameras
    for (int i = 0; i < measurements.cameras_size(); i++) {
      const CameraMeasurement& sensor_data = measurements.cameras(i);
      if (sensor_data.quality() != -1) {
        throw std::runtime_error("Encoded images are not supported in this client");
      }

      cv::Mat img(sensor_data.height(), sensor_data.width(), CV_8UC3, (void*)sensor_data.image().c_str());

      auto imgmsg = sensor_msgs::msg::Image();
      cv_bridge::CvImage img_bridge;
      img_bridge = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, img);
      img_bridge.toImageMsg(imgmsg);
      imgmsg.header.stamp = rclcpp::Time(measurements.time());
      imgmsg.header.frame_id = camera_frame_names_[i];
      camera_image_publishers_[i]->publish(imgmsg);
    }
  }

  void publishIMUs(const SensorMeasurements& measurements) {
    // iterate over the used IMUs
    for (int i = 0; i < measurements.accelerometers_size(); i++) {
      const AccelerometerMeasurement& accel_data = measurements.accelerometers(i);
      const GyroMeasurement& gyro_data = measurements.gyros(i);

      auto imu_msg = sensor_msgs::msg::Imu();
      imu_msg.header.stamp = rclcpp::Time(measurements.time());
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

  void publishJointStates(const SensorMeasurements& measurements) {
    auto jointmsg = sensor_msgs::msg::JointState();
    for (int i = 0; i < measurements.position_sensors_size(); i++) {
      std::string ros_name = map_proto_to_ros[measurements.position_sensors(i).name()];
      jointmsg.name.push_back(ros_name);
      jointmsg.position.push_back(measurements.position_sensors(i).value());
      // todo velocity?
      // todo effort?
    }
    jointmsg.header.stamp = rclcpp::Time(measurements.time());
    joint_state_publisher_->publish(jointmsg);
  }

  void command_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    ActuatorRequests request;

    for (unsigned int i = 0; i < msg->name.size(); i++) {
      std::string name = msg->name[i].c_str();
      if (!map_ros_to_proto.count(name)) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Got joint command with unknown joint " << name.c_str());
        continue;
      }
      std::string proto_name = map_ros_to_proto[name];
      if (msg->position.size() == 0) {
        // no positions give, will use torque control
        MotorTorque* motor_torque = request.add_motor_torques();
        motor_torque->set_name(proto_name);
        motor_torque->set_torque(msg->effort[i]);
      } else {
        // use position and velocity
        MotorPosition* motor_position = request.add_motor_positions();
        motor_position->set_name(proto_name);
        motor_position->set_position(msg->position[i]);
        if (msg->velocity.size() != 0) {
          // if additional velocity is give, use it as maximal speed
          MotorVelocity* motor_velocity = request.add_motor_velocities();
          motor_velocity->set_name(proto_name);
          motor_velocity->set_velocity(msg->velocity[i]);
        }
      }
    }
    // todo check if it could lead to errors due to the paralelism, maybe we need some kind of lock
    client->sendRequest(request);
  }

  double mat_from_fov_and_resolution(double fov, double res) {
    return 0.5 * res * (cos((fov / 2)) / sin((fov / 2)));
  }
  double h_fov_to_v_fov(double h_fov, int height, int width) {
    return 2 * atan(tan(h_fov * 0.5) * (height / width));
  }
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr real_clock_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> camera_image_publishers_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_publishers_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> imu_publishers_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_command_subscription_;

  std::map<std::string, std::string> map_proto_to_ros;
  std::map<std::string, std::string> map_ros_to_proto;
  std::vector<std::string> camera_frame_names_;
  std::vector<std::string> imu_frame_names_;
  RobotClient* client;
};

int main(int argc, char* argv[]) {
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
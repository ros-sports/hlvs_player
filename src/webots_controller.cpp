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

    // Publishers
    clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", 10);
    camera_image_publishers_ = {};
    camera_info_publishers_ = {};
    imu_publishers_ = {};

    // Subscriptions
    motor_command_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "command", 10, std::bind(&WebotsController::command_callback, this, _1));

    // Timer and its callback
    // todo would it make sense to rather do this as a callback of /clock? We basically want to do this exactly once the
    // simulation does a step, it does not really make sense to run this in any other frequency
    timer_ = this->create_wall_timer(8ms, std::bind(&WebotsController::timer_callback, this));

    // Client construction and connecting
    std::string host;
    int port;
    this->get_parameter("host", host);
    this->get_parameter("port", port);
    client = new RobotClient(host, port, 3);
    client->connectClient();

    // Enable devices
    ActuatorRequests request;
    Json::Value devices;
    std::ifstream json_file("resources/devices.json");
    json_file >> devices;

    // Joints
    for (unsigned int i = 0; i < devices["joints"].size(); i++) {
      SensorTimeStep* sensor = request.add_sensor_time_steps();
      sensor->set_name(devices["joints"][i]["proto_sensor_name"].asString());
      sensor->set_timestep(devices["joints"][i]["time_step"].asDouble());
    }
    joint_state_publisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>(devices["joint_state_topic_name"].asString(), 10);

    // Cameras
    for (unsigned int i = 0; i < devices["cameras"].size(); i++) {
      SensorTimeStep* camera_sensor = request.add_sensor_time_steps();
      camera_sensor->set_name(devices["cameras"][i]["proto_camera_name"].asString());
      camera_sensor->set_timestep(devices["cameras"][i]["time_step"].asDouble());
      camera_image_publishers_.push_back(
          this->create_publisher<sensor_msgs::msg::Image>(devices["cameras"][i]["image_topic_name"].asString(), 10));

      // camera info should be latched and only published once
      rclcpp::QoS qos(rclcpp::KeepLast(1));
      qos.transient_local().reliable();
      camera_info_publishers_.push_back(
          this->create_publisher<sensor_msgs::msg::CameraInfo>(devices["cameras"][i]["info_topic_name"].asString(),
                                                               qos));
      // calculate and publish the camera info
      sensor_msgs::msg::CameraInfo camera_info_msg = sensor_msgs::msg::CameraInfo();
      camera_info_msg.header.frame_id = devices["cameras"][i]["optical_frame"].asString();
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
    for (unsigned int i = 0; i < devices["IMUs"].size(); i++) {
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
        auto clk = rosgraph_msgs::msg::Clock();
        clk.clock = rclcpp::Time(measurements.time());
        clock_publisher_->publish(clk);
        publishJointStates(measurements);
        publishImage(measurements);
        publishIMUs(measurements);
      } catch (const std::runtime_error& exc) {
        std::cerr << "Runtime error: " << exc.what() << std::endl;
      }
    }
  }

  void publishJointStates(const SensorMeasurements& measurements) {
    auto jointmsg = sensor_msgs::msg::JointState();
    for (int i = 0; i < measurements.position_sensors_size(); i++) {
      jointmsg.name.push_back(measurements.position_sensors(i).name());
      jointmsg.position.push_back(measurements.position_sensors(i).value());
      // todo velocity? effort?
    }
    jointmsg.header.stamp = rclcpp::Time(measurements.time());
    joint_state_publisher_->publish(jointmsg);
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
      // todo frame id?
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
      // todo
      // imu_msg.header.frame_id = self.imu_frame;
      // todo the following data needs to be scaled correctly
      imu_msg.linear_acceleration.x = accel_data.value().x();
      imu_msg.linear_acceleration.y = accel_data.value().y();
      imu_msg.linear_acceleration.z = accel_data.value().z();
      imu_msg.angular_velocity.x = gyro_data.value().x();
      imu_msg.angular_velocity.y = gyro_data.value().y();
      imu_msg.angular_velocity.z = gyro_data.value().z();
      // we can not get the orientation from the simulation, still make sure
      // that there is at least a valid quaternion in the message
      imu_msg.orientation.w = 1;
      imu_publishers_[i]->publish(imu_msg);
    }
  }

  void command_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const {
    ActuatorRequests request;

    for (unsigned int i = 0; i < msg->name.size(); i++) {
      MotorPosition* sensor = request.add_motor_positions();
      sensor->set_name(msg->name[i]);
      sensor->set_position(msg->position[i]);
    }
    client->sendRequest(request);
    // todo why do we get meassurements here and ignore them?
    SensorMeasurements measurements = client->receive();
  }

  double mat_from_fov_and_resolution(double fov, double res) {
    return 0.5 * res * (cos((fov / 2)) / sin((fov / 2)));
  }
  double h_fov_to_v_fov(double h_fov, int height, int width) {
    return 2 * atan(tan(h_fov * 0.5) * (height / width));
  }
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> camera_image_publishers_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_publishers_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> imu_publishers_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_command_subscription_;

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
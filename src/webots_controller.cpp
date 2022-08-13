#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <json/value.h>
#include <jsoncpp/json/json.h>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/string.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.h>

#include "ros_bridge/robot_client/robot_client.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class WebotsController : public rclcpp::Node {
public:
  WebotsController()
      : Node("webots_controller") {

    // Parameters
    this->declare_parameter<std::string>("host", "127.0.0.1");
    this->declare_parameter<int>("port", 10001);

    // Publishers
    clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", 10);
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
    sensor_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("sensor", 10);
    gyro_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("gyro", 10);

    // Subscriptions
    motor_command_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "command", 10, std::bind(&WebotsController::command_callback, this, _1));

    // Timer and its callback
    timer_ = this->create_wall_timer(
        8ms, std::bind(&WebotsController::timer_callback, this));

    // Client construction and connecting
    this->get_parameter("host", host_);
    this->get_parameter("port", port_);
    client = new RobotClient(host_, port_, 3);
    client->connectClient();

    // Enable devices
    ActuatorRequests request;
    Json::Value devices;
    std::ifstream json_file("src/ros_bridge/resources/devices.json");
    json_file >> devices;

    // Cameras
    for (unsigned int i = 0; i < devices["cameras"].size(); i++) {
      SensorTimeStep *camera_sensor = request.add_sensor_time_steps();
      camera_sensor->set_name(devices["cameras"][i]["name"].asString());
      camera_sensor->set_timestep(devices["cameras"][i]["time_step"].asDouble());
    }

    // Position sensors
    for (unsigned int i = 0; i < devices["joints"].size(); i++) {
      SensorTimeStep *sensor = request.add_sensor_time_steps();
      sensor->set_name(devices["joints"][i]["sensor_name"].asString());
      sensor->set_timestep(devices["joints"][i]["time_step"].asDouble());
    }

    // Gyros
    for (unsigned int i = 0; i < devices["gyros"].size(); i++) {
      SensorTimeStep *sensor = request.add_sensor_time_steps();
      sensor->set_name(devices["gyros"][i]["name"].asString());
      sensor->set_timestep(devices["gyros"][i]["time_step"].asDouble());
    }

    // Accelerometers
    for (unsigned int i = 0; i < devices["accelerometers"].size(); i++) {
      SensorTimeStep *sensor = request.add_sensor_time_steps();
      sensor->set_name(devices["accelerometers"][i]["name"].asString());
      sensor->set_timestep(devices["accelerometers"][i]["time_step"].asDouble());
    }

    client->sendRequest(request);
    SensorMeasurements sensors = client->receive();
  }

private:
  void timer_callback() {
    if (client->isOk()) {
      try {
        ActuatorRequests request;
        client->sendRequest(request);
        SensorMeasurements sensors = client->receive();
        auto clk = rosgraph_msgs::msg::Clock();
        clk.clock = rclcpp::Time(sensors.time());
        clock_publisher_->publish(clk);
        publishImage(sensors);
        publishSensors(sensors);
      }
      catch (const std::runtime_error &exc) {
        std::cerr << "Runtime error: " << exc.what() << std::endl;
      }
    }
  }

  void publishImage(const SensorMeasurements &sensors) {
    for (int i = 0; i < sensors.cameras_size(); i++) {
      const CameraMeasurement &sensor_data = sensors.cameras(i);
      if (sensor_data.quality() != -1) {
        throw std::runtime_error("Encoded images are not supported in this client");
      }

      cv::Mat img(sensor_data.height(), sensor_data.width(), CV_8UC3, (void *)sensor_data.image().c_str());

      auto imgmsg = sensor_msgs::msg::Image();
      cv_bridge::CvImage img_bridge;
      img_bridge = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, img);
      img_bridge.toImageMsg(imgmsg);
      image_publisher_->publish(imgmsg);
    }
  }

  void publishSensors(const SensorMeasurements &sensors) {
    auto jointmsg = sensor_msgs::msg::JointState();
    for (int i = 0; i < sensors.position_sensors_size(); i++) {
      jointmsg.name.push_back(sensors.position_sensors(i).name());
      jointmsg.position.push_back(sensors.position_sensors(i).value());
    }
    sensor_publisher_->publish(jointmsg);
  }

  void command_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const {

    ActuatorRequests request;

    for (unsigned int i = 0; i < msg->name.size(); i++) {
      MotorPosition *sensor = request.add_motor_positions();
      sensor->set_name(msg->name[i]);
      sensor->set_position(msg->position[i]);
    }
    client->sendRequest(request);
    SensorMeasurements sensors = client->receive();
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sensor_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr gyro_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_command_subscription_;
  RobotClient *client;
  std::string host_;
  int port_;
};

int main(int argc, char *argv[]) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebotsController>());
  rclcpp::shutdown();
  return 0;
}
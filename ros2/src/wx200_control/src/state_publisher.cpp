#include <cmath>

#include "dynamixel_helper/dynamixel_helper.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// Change to usb device that appears when dynamixel controller is pluggied in
#define DEVICE_PORT "/dev/ttyUSB0"
#define BAUDRATE 1000000

using namespace std::chrono_literals;

DynamixelHelper dh(DEVICE_PORT);

class StatePublisher : public rclcpp::Node {
public:
  StatePublisher() : Node("state_publisher") {
    this->publisher = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10);
    this->timer = this->create_wall_timer(
        20ms, std::bind(&StatePublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    std::vector<double> angles = dh.groupGetAngle({1, 3, 4, 5, 6});
    for (double &angle : angles) {
      angle -= M_PI;
    }

    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = this->get_clock()->now();
    message.name = {"waist_joint", "shoulder_joint", "elbow_joint",
                    "wrist_angle_joint", "wrist_roll_joint"};
    message.position = angles;
    publisher->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
};

int main(int argc, char **argv) {
  // Initialize connection
  dh.openPort();
  dh.setBaudrate(BAUDRATE);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StatePublisher>());
  rclcpp::shutdown();
  return 0;
}

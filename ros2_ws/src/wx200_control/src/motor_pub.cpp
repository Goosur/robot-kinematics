#include <cmath>
#include <cstdint>
#include <sstream>

#include "dynamixel_helper/dynamixel_helper.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// Change to usb device that appears when dynamixel controller is pluggied in
#define DEVICE_PORT "/dev/ttyUSB0"
#define BAUDRATE 1000000

using std::placeholders::_1;

DynamixelHelper dh(DEVICE_PORT);

class MultiMotorWriter : public rclcpp::Node {
public:
  MultiMotorWriter() : Node("motors_writer") {
    subscription_ =
        this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_goals", 10,
            std::bind(&MultiMotorWriter::write_motors, this, _1));
  }

private:
  void write_motors(const sensor_msgs::msg::JointState &msg) {
    // Arrays for dynamixel communication
    std::stringstream log;
    for (size_t i = 0; i < msg.name.size(); ++i) {
      log << "\nID: " << msg.name[i] << " Position: " << msg.position[i];
    }
    log << "\n--------------";
    RCLCPP_INFO(this->get_logger(), "%s", log.str().c_str());

    // Send goals to wx200
    // dh.groupSetAngle({1, 3, 4, 5, 6}, msg.position);
  }
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      subscription_;
};

int main(int argc, char **argv) {
  // Initialize connection
  dh.openPort();
  dh.setBaudrate(BAUDRATE);

  // TODO: MOVE THIS TO A SERVICE
  std::vector<uint8_t> ids{1, 3, 4, 5, 6};
  // Enable torque
  dh.groupTorqueEnable(ids);

  // Initialize and start ros node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiMotorWriter>());

  // Clean up when user kills the node
  rclcpp::shutdown();

  // TODO: MOVE THIS TO A SERVICE
  dh.groupTorqueDisable(ids);

  return 0;
}

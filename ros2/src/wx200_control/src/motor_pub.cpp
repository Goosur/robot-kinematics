#include <cmath>
#include <cstdint>
#include <sstream>

#include "dynamixel_helper/dynamixel_helper.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "wx200_control_interfaces/msg/set_multi_motors.hpp"

// Change to usb device that appears when dynamixel controller is pluggied in
#define DEVICE_PORT "/dev/ttyUSB0"
#define BAUDRATE 1000000

using std::placeholders::_1;

DynamixelHelper dh(DEVICE_PORT);

class MultiMotorWriter : public rclcpp::Node {
public:
  MultiMotorWriter() : Node("motors_writer") {
    subscription_ = this->create_subscription<
        wx200_control_interfaces::msg::SetMultiMotors>(
        "set_motor_positions", 10,
        std::bind(&MultiMotorWriter::write_motors, this, _1));
  }

private:
  void write_motors(const wx200_control_interfaces::msg::SetMultiMotors &msg) {
    // Arrays for dynamixel communication
    std::vector<uint8_t> ids(msg.ids.size());
    std::vector<double> angles(msg.angles.size());

    std::stringstream log;
    for (size_t i = 0; i < msg.names.size(); ++i) {
      ids[i] = msg.ids[i];
      angles[i] = msg.angles[i] * 180.0 / M_PI / 0.088;
      log << "\nName: " << msg.names[i] << "\nID: " << msg.ids[i]
          << "\nAngle: " << msg.angles[i] << "\n--------------";
    }
    RCLCPP_INFO(this->get_logger(), "%s", log.str().c_str());

    // Send goals to wx200
    dh.groupSetAngle(ids, angles);
  }
  rclcpp::Subscription<wx200_control_interfaces::msg::SetMultiMotors>::SharedPtr
      subscription_;
};

int main(int argc, char **argv) {
  // Initialize connection
  dh.openPort();
  dh.setBaudrate(BAUDRATE);

  // TODO: MOVE THIS TO A SERVICE
  std::vector<uint8_t> ids{1, 2, 4, 5, 6, 7};
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

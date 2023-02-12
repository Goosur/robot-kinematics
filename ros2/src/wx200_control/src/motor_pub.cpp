#include <cstdint>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk/packet_handler.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"

/*******************
 * Useful settings *
 *******************/
// Use latest protocol
#define PROTOCOL_VERSION 2.0
// Change to usb device that appears when dynamixel controller is pluggied in
#define DEVICE_PORT "/dev/ttyUSB0"
// Default baudrate is 57600
#define BAUDRATE 1000000

/**********************************
 * Useful control table registers *
 **********************************/
#define TORQUE_ENABLE_ADDRESS 64
#define LED_ADDRESS 65
#define GOAL_POSITION_ADDRESS 116
#define PRESENT_POSITION_ADDRESS 132

using std::placeholders::_1;

// Dynamixel variables that need to be global
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
int dxl_comm_result = COMM_TX_FAIL;
uint8_t dxl_error = 0;

class MultiMotorWriter : public rclcpp::Node {
public:
  MultiMotorWriter() : Node("motors_writer") {
    subscription_ = this->create_subscription<std_msgs::msg::Int8MultiArray>(
        "set_motor_positions", 10,
        std::bind(&MultiMotorWriter::write_motors, this, _1));
  }

private:
  void write_motors(const std_msgs::msg::Int8MultiArray &msg) {
    RCLCPP_INFO(this->get_logger(), "Message received");
  }
  rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_PORT);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Initialize and start ros node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiMotorWriter>());

  // Clean up when user kills the node
  rclcpp::shutdown();
  delete portHandler;
  delete packetHandler;
  return 0;
}

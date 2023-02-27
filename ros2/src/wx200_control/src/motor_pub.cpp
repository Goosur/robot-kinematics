#include <cmath>
#include <cstdint>
#include <sstream>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "wx200_control_interfaces/msg/set_multi_motors.hpp"

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

// Data length in bytes
#define TORQUE_ENABLE_LENGTH 1
#define LED_LENGTH 1
#define GOAL_POSITION_LENGTH 4
#define PRESENT_POSITION_LENGTH 4

using std::placeholders::_1;

// Dynamixel variables that need to be global
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
int dxl_comm_result = COMM_TX_FAIL;
uint8_t dxl_error = 0;
bool dxl_add_param_result = false;

void groupWriteMotor(uint8_t *ids, size_t ids_size, uint16_t *data,
                     uint16_t address, uint8_t byte_size) {
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, address,
                                           byte_size);

  uint8_t *param_data = new uint8_t[byte_size];
  for (int i = 0; i < (int)ids_size; i++) {
    // Convert data to raw data for transfer
    for (int j = 0; j < byte_size; j++) {
      if (j % 2 == 0) {
        if (j % 4 < 2)
          param_data[j] = DXL_LOBYTE(DXL_LOWORD(data[i]));
        else
          param_data[j] = DXL_LOBYTE(DXL_HIWORD(data[i]));
      } else {
        if (j % 4 < 2)
          param_data[j] = DXL_HIBYTE(DXL_LOWORD(data[i]));
        else
          param_data[j] = DXL_HIBYTE(DXL_HIWORD(data[i]));
      }
    }

    // Add each goal to each motor write parameter
    dxl_add_param_result = groupSyncWrite.addParam(ids[i], param_data);
    if (!dxl_add_param_result) {
      RCLCPP_ERROR(rclcpp::get_logger("motors_writer"),
                   "[dynamixel]: (torque disable)[ID: %03d] GroupSyncWrite "
                   "AddParam failed",
                   ids[i]);
    }
  }

  // Send write packet
  dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("motors_writer"),
                 "[dynamixel]: (torque disable)%s",
                 packetHandler->getTxRxResult(dxl_comm_result));
  }

  // Clear stored write values
  groupSyncWrite.clearParam();
  delete[] param_data;
}

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
    uint8_t *ids = new uint8_t[msg.ids.size()];
    uint16_t *angles = new uint16_t[msg.angles.size()];
    
    std::stringstream log;
    for (size_t i = 0; i < msg.names.size(); ++i) {
      ids[i] = msg.ids[i];
      angles[i] = msg.angles[i] * 180.0 / M_PI / 0.088;
      log << "\nName: " << msg.names[i] << "\nID: " << msg.ids[i]
          << "\nAngle: " << msg.angles[i] << "\n--------------";
    }
    RCLCPP_INFO(this->get_logger(), "%s", log.str().c_str());

    // Send goals to wx200
    groupWriteMotor(ids, msg.ids.size(), angles, GOAL_POSITION_ADDRESS, GOAL_POSITION_LENGTH);
  }
  rclcpp::Subscription<wx200_control_interfaces::msg::SetMultiMotors>::SharedPtr
      subscription_;
};

int main(int argc, char **argv) {
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_PORT);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Initialize connection
  if (portHandler->openPort()) {
    RCLCPP_INFO(rclcpp::get_logger("motors_writer"),
                "[dynamixel]: Successfully opened port on %s", DEVICE_PORT);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("motors_writer"),
                 "[dynamixel]: Failed to open port, exiting...");
    std::exit(1);
  }
  if (portHandler->setBaudRate(BAUDRATE)) {
    RCLCPP_INFO(rclcpp::get_logger("motors_writer"),
                "[dynamixel]: Successfully set baudrate");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("motors_writer"),
                 "[dynamixel]: Failed to set baudrate, exiting...");
    std::exit(1);
  }

  // TODO: MOVE THIS TO A SERVICE
  uint8_t ids[] = {1, 2, 4, 5, 6, 7};
  size_t ids_len = sizeof(ids) / sizeof(ids[0]);
  // Enable torque
  uint16_t enable_torque_data[] = {1, 1, 1, 1, 1, 1};
  groupWriteMotor(ids, ids_len, enable_torque_data, TORQUE_ENABLE_ADDRESS, TORQUE_ENABLE_LENGTH);

  // Initialize and start ros node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiMotorWriter>());

  // Clean up when user kills the node
  rclcpp::shutdown();

  // TODO: MOVE THIS TO A SERVICE
  // Disable torque
  uint16_t disable_torque_data[] = {0, 0, 0, 0, 0, 0};
  groupWriteMotor(ids, ids_len, disable_torque_data, TORQUE_ENABLE_ADDRESS, TORQUE_ENABLE_LENGTH);

  portHandler->closePort();
  delete portHandler;
  delete packetHandler;
  return 0;
}

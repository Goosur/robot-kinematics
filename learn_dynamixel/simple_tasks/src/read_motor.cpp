#include <dynamixel_helper/dynamixel_helper.h>
#include <iostream>

int main() {
  // Initialize dyanmixel helper on USB0 port
  DynamixelHelper dh("/dev/ttyUSB0");

  // We will be moving motor 4 which is the elbow of the widowx200
  uint8_t motor_id = 4;
  double motor_position;

  // Configure open and configure serial communication
  dh.openPort();
  dh.setBaudrate(1000000);

  // Get 1000 data points from the motor
  int count = 0;
  while (count < 1000) {
    // Request current position of motor and output to console
    motor_position = dh.getAngle(motor_id);
    std::cout << motor_position << std::endl;
    count++;
  }

  return 0;
}

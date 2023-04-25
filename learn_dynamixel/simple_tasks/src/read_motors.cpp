#include <dynamixel_helper/dynamixel_helper.h>
#include <iostream>

int main() {

  const char port[] = "/dev/ttyUSB0";
  DynamixelHelper dh(port);

  std::vector<uint8_t> motor_ids{1, 2, 4, 5, 6};
  std::vector<double> motor_positions;

  dh.openPort();
  dh.setBaudrate(1000000);

  while (true) {
    motor_positions = dh.groupGetAngle(motor_ids);
    for (int i = 0; i < motor_ids.size(); i++)
      std::cout << motor_positions[i] << "\t";
    std::cout << std::endl;
  }

  return 0;
}

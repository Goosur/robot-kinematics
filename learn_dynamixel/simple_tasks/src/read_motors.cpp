#include <dynamixel_helper/dynamixel_helper.h>
#include <iostream>

int main() {
  DynamixelHelper dh("/dev/ttyUSB0");

  std::vector<uint8_t> motor_ids{1, 2, 4, 5, 6};

  dh.openPort();
  dh.setBaudrate(1000000);

  while (true) {
    for (auto angle : dh.groupGetAngle(motor_ids))
      std::cout << angle << "\t";
    std::cout << std::endl;
  }

  return 0;
}

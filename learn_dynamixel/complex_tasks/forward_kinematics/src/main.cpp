#include "fk.h"
#include <dynamixel_helper/dynamixel_helper.h>
#include <iomanip>
#include <iostream>

int main(int argc, char **argv) {
  DynamixelHelper dh("/dev/ttyUSB0");

  std::vector<uint8_t> motor_ids{1, 3, 4, 5, 6};
  std::vector<double> motor_angles;
  std::array<double, 3> xyz;

  dh.openPort();
  dh.setBaudrate(1000000);

  // Record 1000 end effector positions
  for (int i = 0; i < 1000; i++) {

    // Get current motor angles
    motor_angles = dh.groupGetAngle(motor_ids);

    // Find end effector coordinates given current coordinates
    std::array<double, 5> thetas;
    std::copy(motor_angles.begin(), motor_angles.end(), thetas.begin());
    xyz = FK::where(thetas);

    // Display end effector coordinates
    std::cout << std::fixed << std::setprecision(6) << xyz[0] << "\t" << xyz[1]
              << "\t" << xyz[2] << std::endl;
  }

  return 0;
}

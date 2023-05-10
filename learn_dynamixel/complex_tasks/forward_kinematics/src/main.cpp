#include <cmath>
#include <dynamixel_helper/dynamixel_helper.h>
#include <iomanip>
#include <iostream>

/**
 * Pre-generated DH transform for the widowx 200 robot arm
 */
std::array<double, 3> where(const std::array<double, 5> &thetas) {
  std::array<std::array<double, 4>, 6> wx200_T05{
      {{std::sin(thetas[0]) * std::sin(thetas[4]) -
            1.0 * std::sin(thetas[1] + thetas[2] + thetas[3]) *
                std::cos(thetas[0]) * std::cos(thetas[4]),
        std::sin(thetas[0]) * std::cos(thetas[4]) +
            1.0 * std::sin(thetas[4]) *
                std::sin(thetas[1] + thetas[2] + thetas[3]) *
                std::cos(thetas[0]),
        -1.0 * std::cos(thetas[0]) *
            std::cos(thetas[1] + thetas[2] + thetas[3]),
        -(1.0 * -174.15 * std::cos(thetas[1] + thetas[2] + thetas[3]) +
          1.0 * 200 * std::cos(thetas[1] + thetas[2]) +
          206.16 * std::sin(thetas[1] - 0.240159419698206)) *
            std::cos(thetas[0])},
       {-1.0 * std::sin(thetas[0]) *
                std::sin(thetas[1] + thetas[2] + thetas[3]) *
                std::cos(thetas[4]) -
            1.0 * std::sin(thetas[4]) * std::cos(thetas[0]),
        1.0 * std::sin(thetas[0]) * std::sin(thetas[4]) *
                std::sin(thetas[1] + thetas[2] + thetas[3]) -
            std::cos(thetas[0]) * std::cos(thetas[4]),
        -1.0 * std::sin(thetas[0]) *
            std::cos(thetas[1] + thetas[2] + thetas[3]),
        -(1.0 * -174.15 * std::cos(thetas[1] + thetas[2] + thetas[3]) +
          1.0 * 200 * std::cos(thetas[1] + thetas[2]) +
          206.16 * std::sin(thetas[1] - 0.240159419698206)) *
            std::sin(thetas[0])},
       {-1.0 * std::cos(thetas[4]) *
            std::cos(thetas[1] + thetas[2] + thetas[3]),
        1.0 * std::sin(thetas[4]) * std::cos(thetas[1] + thetas[2] + thetas[3]),
        1.0 * std::sin(thetas[1] + thetas[2] + thetas[3]),
        1.0 * -174.15 * std::sin(thetas[1] + thetas[2] + thetas[3]) +
            1.0 * 200 * std::sin(thetas[1] + thetas[2]) -
            206.16 * std::cos(thetas[1] - 0.240159419698206)},
       {0, 0, 0, 1}}};

  return {{wx200_T05[0][3], wx200_T05[1][3], wx200_T05[2][3]}};
}

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
    xyz = where(thetas);

    // Display end effector coordinates
    std::cout << std::fixed << std::setprecision(6) << xyz[0] << "\t" << xyz[1]
              << "\t" << xyz[2] << std::endl;
  }

  return 0;
}

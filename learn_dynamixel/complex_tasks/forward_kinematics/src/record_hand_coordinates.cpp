#include "../include/fk.h"
#include <dynamixel_helper/dynamixel_helper.h>
#include <fstream>
#include <iostream>

/* wx200 parameters adjusted for dynamixel motors (added PI to all variable thetas)
 * alpha: [0.0, -M_PI / 2, M_PI, 0.0, 0.0, M_PI / 2, 0.0]
 *     a: [0.0, 0.0, 200.0, 50.0, 200.0, 0.0, 0.0]
 *     d: [113.25, 0.0, 0.0, 0.0, 0.0, 0.0, 174.15]
 * theta: [thetas[0] - M_PI, thetas[1] - 3 * M_PI / 2, -M_PI / 2, thetas[2] - M_PI,
 *         thetas[3] - M_PI / 2, thetas[4] - M_PI / 2, 0.0] */
int main(int argc, char **argv) {
  // File to store angles
  std::ofstream f;
  f.open(argv[1]);

  // Write headers for motors
  f << "waist,shoulder,elbow,wrist_angle,wrist_rotate,end_effector_x,end_"
       "effector_y,end_effector_z"
    << std::endl;

  const char port[] = "/dev/ttyUSB0";
  DynamixelHelper dh(port);

  std::vector<uint8_t> motor_ids{1, 2, 4, 5, 6};
  std::vector<double> motor_angles;
  std::array<double, 3> hand_coordinates;

  // Prime forward kinematics specifically for wx200 arm
  std::vector<double> alpha{0.0, -M_PI / 2, M_PI, 0.0, 0.0, M_PI / 2, 0.0};
  std::vector<double> a{0.0, 0.0, 200.0, 50.0, 200.0, 0.0, 0.0};
  std::vector<double> d{113.25, 0.0, 0.0, 0.0, 0.0, 0.0, 174.15};
  FK wx200(alpha, a, d);

  dh.openPort();
  dh.setBaudrate(1000000);

  // Record 1000 end effector positions
  for (int i = 0; i < 1000; i++) {

    // Get current motor angles
    motor_angles = dh.groupGetAngle(motor_ids);

    // Convert current motor angles to end effector coordinates
    hand_coordinates = wx200.get_end_effector_coordinates(motor_angles);

    // Write motor angles and end effector coordinates to file
    f << motor_angles[0] << "," << motor_angles[1] << "," << motor_angles[2]
      << "," << motor_angles[3] << "," << motor_angles[4] << ","
      << hand_coordinates[0] << "," << hand_coordinates[1] << ","
      << hand_coordinates[2] << std::endl;

    // Display end effector coordinates
    std::cout << hand_coordinates[0] << "\t" << hand_coordinates[1] << "\t"
         << hand_coordinates[2] << std::endl;
  }

  f.close();

  return 0;
}

#include "fk.h"
#include <dynamixel_helper.h>
#include <fstream>
#include <iostream>

/* int main(int argc, char **argv) {
  // File to store angles
  std::ofstream f;
  f.open(argv[1]);

  // Write headers for motors
  f << "waist,shoulder,elbow,wrist_angle,wrist_rotate,end_effector_x,end_"
       "effector_y,end_effector_z"
    << std::endl;

  const char port[] = "/dev/ttyUSB0";
  DynamixelHelper dh(port);

  vector<uint8_t> motor_ids{1, 2, 4, 5, 6};
  vector<double> motor_angles;
  vector<double> hand_coordinates;

  dh.openPort();
  dh.setBaudrate(1000000);

  while (true) {
    // Get current motor angles
    motor_angles = dh.groupGetAngle(motor_ids);

    // Convert current motor angles to end effector coordinates
    hand_coordinates = forward_kinematics::get_hand_coordinates(motor_angles);

    // Write motor angles and end effector coordinates to file
    f << motor_angles[0] << "," << motor_angles[1] << "," << motor_angles[2]
      << "," << motor_angles[3] << "," << motor_angles[4] << ","
      << hand_coordinates[0] << "," << hand_coordinates[1] << ","
      << hand_coordinates[2] << std::endl;

    // Display motor angles and end effector coordinates
    // std::clog << motor_angles[0] << "\t" << motor_angles[1] << "\t" <<
    // motor_angles[2] << "\t" << motor_angles[3] << "\t" << motor_angles[4]
    //           << "\t" << hand_coordinates[0] << "\t" << hand_coordinates[1]
    //           << "\t" << hand_coordinates[2] << std::endl;
    std::clog << hand_coordinates[0] << "\t" << hand_coordinates[1] << "\t"
              << hand_coordinates[2] << std::endl;
  }

  f.close();

  return 0;
} */

int main() {
  vector<double> alpha{1.1, 1.2, 1.3, 1.4};
  vector<double> a{2.1, 2.2, 2.3, 2.4};
  vector<double> d{3.1, 3.2, 3.3, 3.4};
  FK fk(alpha, a, d);
  return 0;
}

#include <dynamixel_helper.h>
#include <fstream>
#include <iostream>

int main(int argc, char **argv) {
  // Open file for writing with user defined file name
  std::ofstream f;
  f.open(argv[1]);

  // Write headers for motors
  f << "waist,shoulder,elbow,wrist_angle,wrist_rotate" << std::endl;

  // Initialize dynamixel helper
  const char port[] = "/dev/ttyUSB0";
  DynamixelHelper dh(port);

  vector<uint8_t> motor_ids{1, 2, 4, 5, 6};
  vector<double> current_joint_angles;

  dh.openPort();
  dh.setBaudrate(1000000);

  int time_step = 0;
  while (time_step < 1000) {
    current_joint_angles = dh.groupGetAngle(motor_ids);

    for (int i = 0; i < motor_ids.size(); i++) {
      std::cout << current_joint_angles[i] << "\t";
      if (i != motor_ids.size() - 1)
        f << current_joint_angles[i] << ",";
      else
        f << current_joint_angles[i];
    }
    f << std::endl;
    std::cout << std::endl;

    time_step++;
  }

  f.close();

  return 0;
}

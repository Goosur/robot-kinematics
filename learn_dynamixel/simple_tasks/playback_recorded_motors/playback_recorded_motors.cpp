#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>

#include "dynamixel_helper.h"

int main(int argc, char **argv) {
  // Initialize dynamixel helper
  const char port[] = "/dev/ttyUSB0";
  DynamixelHelper dh(port);

  // Open recorded joint angle csv
  std::ifstream f(argv[1]);

  // Position data
  vector<uint8_t> motor_ids{1, 2, 4, 5, 6};

  // Distance between current position and goal position that is considered
  // close enough to the goal
  double moving_status_threshold = 10 * 2 * 20 * 0.088 * M_PI / 180.0;

  // Initialize connection
  dh.openPort();
  dh.setBaudrate(1000000);

  // Torque up motor of interest
  dh.groupTorqueEnable(motor_ids);

  // Read from csv and play back movement
  std::string line;
  getline(f, line); // Skip first line which is headers
  while (getline(f, line)) {
    // Split csv line at commas
    char *element = strtok(const_cast<char *>(line.c_str()), ",");

    // Take each substring from the current line and convert it to an array of
    // doubles
    vector<double> new_joint_angles;
    while (element != nullptr) {
      new_joint_angles.push_back(std::stod(element));
      element = strtok(nullptr, ",");
    }

    // Write this line's joint angles to the motors
    dh.groupSetAngle(motor_ids, new_joint_angles);

    // Wait until goal is reached
    bool still_moving;
    vector<double> current_joint_angles;
    do {
      still_moving = false;
      current_joint_angles = dh.groupGetAngle(motor_ids);
      // See how far the arm is from the goal position by checking every joint
      for (int i = 0; i < motor_ids.size(); i++)
        still_moving |=
            std::abs(new_joint_angles[i] - current_joint_angles[i]) >
            moving_status_threshold;
    } while (still_moving);
  }

  // Stop torque before ending program
  dh.groupTorqueDisable(motor_ids);

  return 0;
}

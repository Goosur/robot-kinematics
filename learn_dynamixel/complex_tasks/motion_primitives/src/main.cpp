#include <cmath>
#include <dynamixel_helper/dynamixel_helper.h>
#include "motion_primitives.h"

int main() {
  // Initialize Dynamixel Helper
  const char port[] = "/dev/ttyUSB0";
  DynamixelHelper dh(port);

  // Motor information
  vector<uint8_t> motor_ids{1, 2, 4, 5, 6};
  vector<double> present_positions;
  vector<double> goal_positions;
  double moving_status_threshold = 10 * 20 * 0.088 * M_PI / 180.0;

  dh.openPort();
  dh.setBaudrate(1000000);
  dh.groupTorqueEnable(motor_ids);

  int steps = 0;

  double dphase = 0.001;
  double phase = 0.0;
  while (phase < 1.0) {
    // goal_positions = motion_primitives::draw_x(phase);
    goal_positions = motion_primitives::sleep_to_home(phase);
    // goal_positions = motion_primitives::home_to_sleep(phase);
    // goal_positions = motion_primitives::draw_line(phase);

    dh.groupSetAngle(motor_ids, goal_positions);

    bool still_moving = true;
    do {
      present_positions = dh.groupGetAngle(motor_ids);

      still_moving = false;
      for (int i = 0; i < motor_ids.size(); i++)
        still_moving |= std::abs(goal_positions[i] - present_positions[i]) >
                        moving_status_threshold;
    } while (still_moving);

    phase += dphase;
  }

  phase = 0.0;
  while (phase < 1.0) {
    // goal_positions = motion_primitives::draw_x(phase);
    // goal_positions = motion_primitives::sleep_to_home(phase);
    goal_positions = motion_primitives::home_to_sleep(phase);
    // goal_positions = motion_primitives::draw_line(phase);

    dh.groupSetAngle(motor_ids, goal_positions);

    bool still_moving = true;
    do {
      present_positions = dh.groupGetAngle(motor_ids);

      still_moving = false;
      for (int i = 0; i < motor_ids.size(); i++)
        still_moving |= std::abs(goal_positions[i] - present_positions[i]) >
                        moving_status_threshold;
    } while (still_moving);

    phase += dphase;
  }

  dh.groupTorqueDisable(motor_ids);

  return 0;
}

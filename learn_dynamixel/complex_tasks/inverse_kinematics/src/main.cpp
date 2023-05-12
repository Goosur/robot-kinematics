#include "fk.h"
#include "ik.h"
#include <cmath>
#include <dynamixel_helper/dynamixel_helper.h>
#include <iostream>
#include <thread>

using namespace std::chrono_literals;

int main() {
  DynamixelHelper dh("/dev/ttyUSB0");
  dh.openPort();
  dh.setBaudrate(1000000);

  std::vector<uint8_t> motor_ids{1, 3, 4, 5, 6};

  // End effector goal pose [x, y, z, roll, pitch]
  std::vector<std::array<double, 5>> goal_poses{
      {200.0, 0.0, 250.0, 0.0, 0.0},
      // {350.0, 0.0, 0.0, 0.0, 0.0},
  };
  // std::array<double, 5> goal_pose;

  std::vector<double> initial_thetas = dh.groupGetAngle(motor_ids);
  std::array<double, 5> current_thetas;
  std::copy_n(initial_thetas.begin(), 5, current_thetas.begin());
  std::array<double, 5> next_thetas;

  // Enable torque before we try sending angles to the motors
  dh.groupTorqueEnable(motor_ids);

  for (auto goal_pose : goal_poses) {
    std::this_thread::sleep_for(500ms);
    bool goal_reached = false;
    while (!goal_reached) {
      // Generate new thetas and move to them
      next_thetas = IK::get_next_thetas(current_thetas, goal_pose);
      std::vector<double> next_thetas_v(next_thetas.begin(), next_thetas.end());
      dh.groupSetAngle(motor_ids, next_thetas_v);

      // Naive error correction
      std::vector<double> real_thetas = dh.groupGetAngle(motor_ids);
      // for (int i = 0; i < next_thetas.size(); i++) {
      //   next_thetas_v[i] += next_thetas[i] - real_thetas[i];
      // }
      // std::cout << std::endl;
      // dh.groupSetAngle(motor_ids, next_thetas_v);

      bool moving = true;
      while (moving) {
        std::vector<uint32_t> moving_motors =
            dh.groupReadMotor(motor_ids, 122, 1);
        for (bool moving_motor : moving_motors) {
          moving &= moving_motor;
        }
      }

      // Update current pose from real robot angles
      real_thetas = dh.groupGetAngle(motor_ids);
      std::array<double, 5> real_thetas_array;
      std::copy_n(real_thetas.begin(), 5, real_thetas_array.begin());
      std::array<double, 3> current_xyz = FK::where(real_thetas_array);
      // Update current pose
      std::array<double, 5> current_pose = {
          {current_xyz[0], current_xyz[1], current_xyz[2],
           real_thetas_array[4] - M_PI,
           real_thetas_array[1] + real_thetas_array[2] + real_thetas_array[3] -
               3 * M_PI}};

      // Announce current pose
      std::cout << "x: " << current_pose[0] << '\t' << "y: " << current_pose[1]
                << '\t' << "z: " << current_pose[2] << '\t'
                << "roll: " << current_pose[3] << '\t'
                << "pitch: " << current_pose[4] << '\t' << std::endl;

      // We are close to the goal if the old thetas are very close to the new
      // thetas
      bool reached = true;
      for (int i = 0; i < goal_pose.size(); i++) {
        if (i < 3)
          reached &= std::abs(goal_pose[i] - current_pose[i]) < 5;
        else
          reached &= std::abs(goal_pose[i] - current_pose[i]) < 0.2;
      }

      std::cout << std::endl;
      goal_reached = reached;

      // Update thetas
      current_thetas = next_thetas;
    }
    std::cout << "Goal reached" << std::endl;
  }

  // Disable torque before exiting program so motor doesn't stay stiff until
  // power removed
  std::cout << "Press Enter to detorque" << std::endl;
  std::cin.get();
  dh.groupTorqueDisable(motor_ids);

  return 0;
}

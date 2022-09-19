#include <cmath>
#include <dynamixel_helper.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

// Get character input
int getch() {
  struct termios oldt {
  }, newt{}; // old terminal mode, new terminal mode
  int ch;
  tcgetattr(STDIN_FILENO,
            &oldt); // Get old terminal mode from terminal STDIN_FILENO
  newt = oldt;      // Store old terminal mode in new terminal mode variable
  newt.c_lflag &= ~(ICANON | ECHO); // Enter non-canonical mode (don't wait for
                                    // enter key) and disable echoing.
  tcsetattr(STDIN_FILENO, TCSANOW, &newt); // Set terminal to new mode
  ch = getchar(); // Get character from terminal in new mode
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // Set terminal to old mode again
  return ch;
}

int main() {
  const char port[] = "/dev/ttyUSB0";
  DynamixelHelper dh(port);

  // Position data
  vector<uint8_t> motor_ids{1, 2, 4, 5, 6};
  vector<double> present_positions;
  vector<vector<double>> goal_positions{
      {M_PI, M_PI, M_PI, M_PI, M_PI},
      {0 + M_PI, -1.88 + M_PI, 1.5 + M_PI, 0.8 + M_PI, 0 + M_PI},
  };

  double moving_status_threshold = 2 * 20 * 0.088 * M_PI / 180.0;

  // Initialize connection
  dh.openPort();
  dh.setBaudrate(1000000);

  // Torque up motor of interest
  dh.groupTorqueEnable(motor_ids);

  bool index = 0;
  while (true) {
    std::cout << "Press any key to continue. (Press q to exit)" << std::endl;
    if (getch() == 0x71) // ASCII hex for q
      break;

    // Write goal position
    dh.groupSetAngle(motor_ids, goal_positions[index]);

    // Wait until goal is reached
    bool still_moving;
    do {
      still_moving = false;
      present_positions = dh.groupGetAngle(motor_ids);
      for (int i = 0; i < motor_ids.size(); i++)
        still_moving |=
            std::abs(goal_positions[index][i] - present_positions[i]) >
            moving_status_threshold;
      std::clog << "Waiting for arm to reach goal..." << std::endl;
    } while (still_moving);
    std::clog << "Finished moving" << std::endl;

    // Switch the Goal Position
    index = !index;
  }

  dh.groupTorqueDisable(motor_ids);

  return 0;
}

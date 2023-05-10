#include <cmath>
#include <dynamixel_helper/dynamixel_helper.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

// Get character input
int getch() {
  struct termios oldt, newt; // old terminal mode, new terminal mode
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
  DynamixelHelper dh("/dev/ttyUSB0");

  // Position data
  double present_position = 0.0;
  double goal_positions[] = {M_PI / 2, 3 * M_PI / 2};

  uint8_t motor_id = 6; // Wrist rotate

  // Initialize connection
  dh.openPort();
  dh.setBaudrate(1000000);

  // Torque up motor of interest
  dh.torqueEnable(motor_id);

  bool index = 0;
  while (true) {
    std::cout << "Press any key to continue. (Press q to exit)" << std::endl;
    if (getch() == 0x71)
      break;

    // Write goal position
    dh.setAngle(motor_id, goal_positions[index]);

    // Wait until goal is reached
    bool moving = true;
    while (moving) {
      std::cout << "Distance from goal (rad): "
                << std::abs(goal_positions[index] - dh.getAngle(motor_id))
                << std::endl;
      moving = dh.readMotor(motor_id, 128);
    }
    std::clog << "Finished moving" << std::endl;

    // Switch the Goal Position
    index = !index;
  }

  dh.torqueDisable(motor_id);

  return 0;
}

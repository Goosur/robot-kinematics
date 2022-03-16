#include <cmath>
#include <unistd.h>
#include <termios.h>

#include <iostream>
#include <math.h>

#include "dynamixel_sdk.h"
#include "dynamixel_helper.h"

// Get character input
int getch() {
    struct termios oldt, newt; // old terminal mode, new terminal mode
    int ch;
    tcgetattr(STDIN_FILENO, &oldt); // Get old terminal mode from terminal STDIN_FILENO
    newt = oldt; // Store old terminal mode in new terminal mode variable
    newt.c_lflag &= ~(ICANON | ECHO); // Enter non-canonical mode (don't wait for enter key) and disable echoing.
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // Set terminal to new mode
    ch = getchar(); // Get character from terminal in new mode
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // Set terminal to old mode again
    return ch;
}

int main() {

    const char port[] = "/dev/ttyUSB0";
    DynamixelHelper dynamixelHelper(port);

    // Position data
    double present_position = 0.0;
    double goal_positions[] = {M_PI / 2, 3 * M_PI / 2};

    uint8_t motor_id = 6; // Wrist rotate
    double moving_status_threshold = 20 * 0.088 * M_PI / 180.0;

    // Initialize connection
    dynamixelHelper.openPort();
    dynamixelHelper.setBaudrate(1000000);

    // Torque up motor of interest
    dynamixelHelper.torqueEnable(motor_id);

    bool index = 0;
    while (true)
    {
        std::cout << "Press any key to continue. (Press [ESC] to exit)" << std::endl;
        if (getch() == 0x1b)
            break;

        // Write goal position
        dynamixelHelper.setAngle(motor_id, goal_positions[index]);

        // Wait until goal is reached
        do
        {
            present_position = dynamixelHelper.getAngle(motor_id);
            std::clog << "[ID: " << (int)motor_id << "] Goal Position: " << goal_positions[index] << ", Present Position: " << present_position << ", Distance to Goal: " << std::abs(goal_positions[index] - present_position) << std::endl;
        }
        while (std::abs(goal_positions[index] - present_position) > moving_status_threshold);
        std::clog << "Finished moving" << std::endl;

        // Switch the Goal Position
        index = !index;
    }

    dynamixelHelper.torqueDisable(motor_id);

    return 0;
}

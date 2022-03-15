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
    uint8_t motor_ids[]{1, 2, 4, 5, 6};
    size_t ids_size = sizeof(motor_ids) / sizeof(motor_ids[0]); double *present_positions;
    double goal_positions[][5] = {
        {M_PI, M_PI, M_PI, M_PI, M_PI},
        {0 + M_PI, -1.88 + M_PI, 1.5 + M_PI, 0.8 + M_PI, 0 + M_PI},
    };

    double moving_status_threshold = 2 * 20 * 0.088 * M_PI / 180.0;

    // Initialize connection
    dynamixelHelper.openPort();
    dynamixelHelper.setBaudrate(1000000);

    // Torque up motor of interest
    dynamixelHelper.groupTorqueEnable(motor_ids, ids_size);

    bool index = 0;
    while (true)
    {
        std::cout << "Press any key to continue. (Press [ESC] to exit)" << std::endl;
        if (getch() == 0x1b)
            break;

        // Write goal position
        dynamixelHelper.groupSetAngle(motor_ids, ids_size, goal_positions[index]);

        // Wait until goal is reached
        bool still_moving = true;
        do
        {
            still_moving = false;
            present_positions = dynamixelHelper.groupGetAngle(motor_ids, ids_size);
            for (int i = 0; i < (int)ids_size; i++)
                still_moving |= std::abs(goal_positions[index][i] - present_positions[i]) > moving_status_threshold;
            std::clog << "Waiting for arm to reach goal..." << std::endl;
        }
        while (still_moving);
        std::clog << "Finished moving" << std::endl;

        // Switch the Goal Position
        index = !index;
    }

    dynamixelHelper.groupTorqueDisable(motor_ids, ids_size);

    return 0;
}

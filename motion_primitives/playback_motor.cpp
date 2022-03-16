#include <stdlib.h>
#include <iostream>
#include <math.h>

#include "dynamixel_helper.h"
#include "motion_primitives.h"

int main() {
    
    // Initialize Dynamixel Helper
    const char port[] = "/dev/ttyUSB0";
    DynamixelHelper dynamixelHelper(port);

    // Motor information
    uint8_t motor_ids[]{1, 2, 4, 5, 6};
    size_t ids_size = sizeof(motor_ids) / sizeof(motor_ids[0]);
    double *present_positions;
    double *goal_positions;
    double moving_status_threshold = 10 * 20 * 0.088 * M_PI / 180.0;

    dynamixelHelper.openPort();
    dynamixelHelper.setBaudrate(1000000);
    dynamixelHelper.groupTorqueEnable(motor_ids, ids_size);

    int steps = 0;
    
    double phase = 0.0;
    while(phase < 1.0)
    {
        // goal_positions = motion_primitives::draw_x(phase);
        // goal_positions = motion_primitives::sleep_to_home(phase);
        // goal_positions = motion_primitives::home_to_sleep(phase);
        goal_positions = motion_primitives::draw_line(phase);

        dynamixelHelper.groupSetAngle(motor_ids, ids_size, goal_positions);

        bool still_moving = true;
        do
        {
            present_positions = dynamixelHelper.groupGetAngle(motor_ids, ids_size);

            still_moving = false;
            for (int i = 0; i < (int)ids_size; i++)
                still_moving |= std::abs(goal_positions[i] - present_positions[i]) > moving_status_threshold;
        }
        while (still_moving);

        phase += 0.001;
    }

    dynamixelHelper.groupTorqueDisable(motor_ids, ids_size);

    delete[] present_positions;
    delete[] goal_positions;

    return 0;
}

#include <iostream>
#include <fstream>

#include "dynamixel_sdk.h"
#include "dynamixel_helper.h"

int main(int argc, char** argv) {

    // File to store angles
    std::ofstream f;
    f.open(argv[1]);
    // Write headers for motors
    f << "waist,shoulder,elbow,wrist_angle,wrist_rotate" << std::endl;

    const char port[] = "/dev/ttyUSB0";
    DynamixelHelper dynamixelHelper(port);

    uint8_t motor_ids[] = {1, 2, 4, 5, 6};
    size_t ids_size = sizeof(motor_ids) / sizeof(motor_ids[0]);
    double *motor_positions;

    dynamixelHelper.openPort();
    dynamixelHelper.setBaudrate(1000000);

    while(true)
    {
        motor_positions = dynamixelHelper.groupGetAngle(motor_ids, ids_size);

        for (int i = 0; i < (int)ids_size; i++)
        {
            std::cout << motor_positions[i] << "\t";
            if (i != (int)ids_size - 1)
                f << motor_positions[i] << ",";
            else
                f << motor_positions[i];
        }
        f << std::endl;
        std::cout << std::endl;
    }

    f.close();

    return 0;
}

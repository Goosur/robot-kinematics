#include <iostream>

#include "dynamixel_sdk.h"
#include "dynamixel_helper.h"

int main() {

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
        }
        std::cout << std::endl;
    }

    return 0;
}

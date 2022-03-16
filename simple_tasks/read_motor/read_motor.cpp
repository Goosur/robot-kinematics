#include <iostream>

#include "dynamixel_sdk.h"
#include "dynamixel_helper.h"

int main() {

    const char port[] = "/dev/ttyUSB0";
    DynamixelHelper dynamixelHelper(port);

    uint8_t motor_id = 4;
    double motor_position;

    dynamixelHelper.openPort();
    dynamixelHelper.setBaudrate(1000000);

    while(true)
    {
        motor_position = dynamixelHelper.getAngle(motor_id);
        std::cout << motor_position << std::endl;
    }

    return 0;
}

#include <iostream>
#include "dynamixel_helper.h"

int main()
{
    vector<uint8_t> motor_ids{1, 2, 4, 5, 6, 7};
//    DynamixelHelper dh("/dev/ttyUSB0");

//    dh.openPort();
//    dh.setBaudrate(56700);
//    dh.getAngle(1);

    auto portHandler = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
    auto packetHandler = dynamixel::PacketHandler::getPacketHandler();

    portHandler->openPort();
    portHandler->setBaudRate(1000000);

    std::cout << packetHandler->getTxRxResult(packetHandler->factoryReset(portHandler, 0xFF)) << std::endl;

    return 0;
}
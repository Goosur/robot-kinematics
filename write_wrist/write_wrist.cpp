#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#include <windows.h>
#endif

#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk.h"  // Uses DYNAMIXEL SDK library

// Control table address for wx200 arm motors
#define ADDR_TORQUE_ENABLE          64      // RW - Init Val: 0,    Range: 0 ~ 1
#define ADDR_GOAL_POSITION          116     // RW - Init Val: -,    Range: Min Position Limit(52) ~ Max Position Limit(48)
#define ADDR_PRESENT_POSITION       132     // R  - Init Val: -,    Range: -
#define MIN_POSITION_LIMIT          0       // RW - Init Val: 0,    Range: 0 ~ 4095
#define MAX_POSITION_LIMIT          4095    // RW - Init Val: 4095, Range 0 ~ 4095
#define BAUDRATE                    1000000 // Dynamixel XM430-W350-T / XL430-W250-T BAUDRATE

// DYNAMIXEL Protocol Version (1.0 / 2.0)
// https://emanual.robotis.com/docs/en/dxl/protocol2/
#define PROTOCOL_VERSION  2.0

// Use the actual port assigned to the U2D2.
// ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
#define DEVICENAME  "/dev/ttyUSB0"

#define ESC_ASCII_VALUE                 0x1b

// Get character input
int getch() {
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt; // old terminal mode, new terminal mode
    int ch;
    tcgetattr(STDIN_FILENO, &oldt); // Get old terminal mode from terminal STDIN_FILENO
    newt = oldt; // Store old terminal mode in new terminal mode variable
    newt.c_lflag &= ~(ICANON | ECHO); // Enter non-canonical mode (don't wait for enter key) and disable echoing.
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // Set terminal to new mode
    ch = getchar(); // Get character from terminal in new mode
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // Set terminal to old mode again
    return ch;
#elif defined(_WIN32) || defined(_WIN64)
    return _getch();
#endif
}

int main() {
    // Initialize PortHandler instance
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Error statuses
    uint8_t dxl_error = 0; // Motor error code
    int dxl_comm_result = COMM_TX_FAIL; // Communication result

    // Position data
    uint32_t dxl_present_position = 0;
    int dxl_goal_position[2] = {MIN_POSITION_LIMIT, MAX_POSITION_LIMIT}; // Goal position
    bool index = 0;

    int motor_id = 6; // Wrist rotate
    uint8_t moving_status_threshold = 20;

    // Open port
    if (portHandler->openPort())
    {
        printf("Successfully opened the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
        printf("Successfully changed the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    // Enable DYNAMIXEL Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("[ID: %03d] Torque Enabled\n", motor_id);
    }

    while (true)
    {
        printf("Press any key to continue. (Press [ESC] to exit)\n");
        if (getch() == ESC_ASCII_VALUE)
            break;

        // Write goal position
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, motor_id, ADDR_GOAL_POSITION, dxl_goal_position[index], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }

        // Wait until goal is reached
        do
        {
            // Read the Present Position
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, motor_id, ADDR_PRESENT_POSITION, &dxl_present_position, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0)
            {
                printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            }

            printf("[ID: %03d] Goal Position: %03d  Present Position: %03d\n", motor_id, dxl_goal_position[index], dxl_present_position);

        }
        while ((dxl_goal_position[index] - dxl_present_position) > moving_status_threshold);

        // Switch the Goal Position
        index = !index;
    }

    // Disable DYNAMIXEL Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else
    {
        printf("[ID: %03d] Torque Disabled\n", motor_id);
    }

    portHandler->closePort();
    return 0;
}

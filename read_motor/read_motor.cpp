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
#define ADDR_PRESENT_POSITION       132     // R  - Init Val: -, Range: -
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

    std::array<int, 6> motor_ids = {1, 2, 4, 5, 6, 7};
    std::array<uint32_t, 6> motor_positions = {0, 0, 0, 0, 0, 0};

    // Open port
    if (portHandler->openPort()) {
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

    while(true)
    {
        printf("\n");
        for (int i = 0; i < (int)motor_ids.size(); i++)
        {
            // Read motor i present position
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, motor_ids[i], ADDR_PRESENT_POSITION, &motor_positions[i], &dxl_error);
            
            // Display any errors in communication or motors.
            if (dxl_comm_result != COMM_SUCCESS)
            {
                printf("[ID: %03d]: %s\n", i, packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0)
            {
                printf("[ID: %03d]: %s\n", i, packetHandler->getRxPacketError(dxl_error));
            }

            printf("[ID: %03d] Present Position: %04d\n", motor_ids[i], motor_positions[i]);
        }
        sleep(1);
    }

    portHandler->closePort();
    return 0;
}

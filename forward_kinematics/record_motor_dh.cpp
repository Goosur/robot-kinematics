#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#include <windows.h>
#endif

//#include <stdlib.h>
//#include <stdio.h>
#include <cmath>
#include <iostream>
#include <math.h>
#include <array>

#include "Eigen/Dense"      // Eigen library for matrix operations
#include "dynamixel_sdk.h"  // Uses DYNAMIXEL SDK library
#include "fk.h"             // Contains functions for DH operations
#include <fstream>          // Record data to a file


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

    // Motor information
    std::array<int, 5> motor_ids = {1, 2, 4, 5, 6};
    std::array<uint32_t, 5> motor_positions = {0, 0, 0, 0, 0};
    std::array<double, 5> motor_angles = {0.0, 0.0, 0.0, 0.0, 0.0};

    // File for storing motor data
    std::ofstream file;
    file.open("motor_angles_xyz.csv");
    // Headers
    file << "waist,shoulder,elbow,wrist_angle,wrist_rotate,hand_x,hand_y,hand_z" << std::endl;

    // Open port
    if (portHandler->openPort()) {
        std::cout << "Successfully opened the port!" << std::endl;
    }
    else
    {
        std::cout << "Failed to open the port!" << std::endl;
        std::cout << "Press any key to terminate ..." << std::endl;
        getch();
        return 0;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
        std::cout << "Successfully changed the baudrate!" << std::endl;
    }
    else
    {
        std::cout << "Failed to change the baudrate!" << std::endl;
        std::cout << "Press any key to terminate ..." << std::endl;
        getch();
        return 0;
    }

    while(true)
    {
        for (int i = 0; i < (int)motor_ids.size(); i++)
        {
            // Read motor i present position
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, motor_ids[i], ADDR_PRESENT_POSITION, &motor_positions[i], &dxl_error);

            // Convert motor position into radians
            // 0.088 degrees = 1 position value
            // pi = 180 degrees
            motor_angles[i] = motor_positions[i] * 0.088 * M_PI / 180.0;
            
            // Display any errors in communication or motors.
            if (dxl_comm_result != COMM_SUCCESS)
            {
                std::cout << "[ID: " << i << "] " << packetHandler->getTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
                std::cout << "[ID: " << i << "] " << packetHandler->getRxPacketError(dxl_error);
            }

            // std::cout << "[ID: " << motor_ids[i] << "] Current Position: " << motor_positions[i] << std::endl;
        }

        std::array<double, 3> hand_position = ForwardKinematics::get_hand_position(motor_angles);

        // Write motor angles and end effector coordinates to file
        file << motor_angles[0] << "," << motor_angles[1] << "," << motor_angles[2] << "," << motor_angles[3] << "," << motor_angles[4]
                  << "," << hand_position[0] << "," << hand_position[1] << "," << hand_position[2] << std::endl;

        // Display motor angles and end effector coordinates
        std::cout << "[ " << motor_angles[0] << ", " << motor_angles[1] << ", " << motor_angles[2] << ", " << motor_angles[3] << ", " << motor_angles[4]
                  << " ], [" << hand_position[0] << ", " << hand_position[1] << ", " << hand_position[2] << " ]" << std::endl;

        // // Wait 1 second
        // sleep(1);
    }

    portHandler->closePort();
    file.close();
    return 0;
}

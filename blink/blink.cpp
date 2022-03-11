// For getting key press
#include <unistd.h>
#include <termios.h>

#include <iostream>

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

    uint8_t motorIDs[] = {1, 2, 4, 5, 6};

    dynamixelHelper.openPort();
    dynamixelHelper.setBaudrate(1000000);

    // Blink all motor leds
    bool led_status = false;
    while(true) {
        std::cout << "Press any key to toggle LEDs. (Press [ESC] to exit)" << std::endl;
        if (getch() == 0x1b)
          break;

        // Toggle led on/off
        led_status = !led_status;

        for (uint8_t id : motorIDs)
        {
            if (led_status)
                dynamixelHelper.ledEnable(id);            
            else
                dynamixelHelper.ledDisable(id);
        }
    }

    // Disable all leds before stopping program
    for (uint8_t id : motorIDs)
        dynamixelHelper.ledDisable(id);

    return 0;
}

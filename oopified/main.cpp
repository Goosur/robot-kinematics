#include <iostream>
#include <cmath>
#include <vector>

#include "dynamixel_helper.h"
#include "Robot.h"

using std::vector;

int main()
{
	char port[] = "/dev/ttyUSB0";
	auto *dxlHelper = new DynamixelHelper(port);
    dxlHelper->openPort();
    dxlHelper->setBaudrate(1000000);

    // Define wx200 motor ids
	vector<uint8_t> motor_ids{1, 2, 4, 5, 6};

    // Create wx200 robot
    Robot wx200(motor_ids, dxlHelper);

    // Initialize some wx200 stuff
    wx200.setNFrames((int)motor_ids.size() + 2);
    wx200.setAlphas(vector<double>{0.0, M_PI / 2, 0.0, 0.0, 0.0, M_PI / 2, 0.0});
    wx200.setAs(vector<double>{0.0, 0.0, 200.0, 50.0, 200.0, 0.0, 0.0});
    wx200.setDs(vector<double>{113.25, 0.0, 0.0, 0.0, 0.0, 0.0, 174.15});

	int count = 0;
	while (count < 1000)
	{
        for (auto angle : wx200.getJointAngles())
            std::cout << angle << "\t";
        for (auto coord : wx200.getGripperPosition())
            std::cout << coord << "\t";
        std::cout << std::endl;
        count++;
	}

    // Clean up
    delete dxlHelper;

	return 0;
}

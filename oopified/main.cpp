#include <iostream>
#include <cmath>
#include <vector>
#include <unistd.h>

#include "dynamixel_helper.h"
#include "transform.h"
#include "fk.h"

using std::vector;

int main()
{
	char port[] = "/dev/ttyUSB0";
	DynamixelHelper d_helper = DynamixelHelper(port);

	uint8_t motor_ids[] = {1, 2, 4, 5, 6};
	size_t num_motors = sizeof(motor_ids) / sizeof(motor_ids[0]);
	int num_frames = num_motors + 2;
	double *angles;

	vector<double> alphas {0.0, M_PI / 2, 0.0, 0.0, 0.0, M_PI / 2, 0.0};
	vector<double> as {0.0, 0.0, 200.0, 50.0, 200.0, 0.0, 0.0};
	vector<double> ds {113.25, 0.0, 0.0, 0.0, 0.0, 0.0, 174.15};
	vector<double> thetas;
	vector<double> xyz;

	d_helper.openPort();
	d_helper.setBaudrate(1000000);

	int count = 0;
	while (count < 1000)
	{
		angles = d_helper.groupGetAngle(motor_ids, num_motors);
		thetas = {angles[0], angles[1] - M_PI / 2, M_PI / 2, angles[2] - M_PI, angles[3] - M_PI / 2, angles[4], 0.0};
		xyz = FK::get_gripper_coords(num_frames, alphas, as, ds, thetas);

		
		// std::cout << angles[0] << "\t" << angles[1] << "\t" << angles[2] << "\t" << angles[3] << "\t" << angles[4] << std::endl;
		std::cout << xyz[0] << "\t\t" << xyz[1] << "\t\t" << xyz[2] << "\t\t" << std::endl;

		usleep(100);
		count++;
	}

	return 0;
}

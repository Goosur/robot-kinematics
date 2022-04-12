#ifndef FK_H
#define FK_H

#include <cmath>
#include <vector>

#include "transform.h"

using std::vector;

namespace FK
{
	vector<double> get_gripper_coords(int n_frames, vector<double> alphas, vector<double> as, vector<double> ds, vector<double> thetas)
	{
		Transform T_0N = Transform(alphas[0], as[0], ds[0], thetas[0]);

		for (int i = 1; i < n_frames; i++)
			T_0N = T_0N * Transform(alphas[i], as[i], ds[i], thetas[i]);

		return T_0N.get_position();
	}
}

#endif

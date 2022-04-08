#ifndef FK_H
#define FK_H

#include <math.h>
#include <vector>

#include "transform.h"

using std::vector;

/*
class FK
{
	public:
		FK(int n_frames, vector<double> alphas, vector<double> as, vector<double> ds, vector<double> thetas) : n_frames(n_frames), alphas(alphas), as(as), ds(ds), thetas(thetas) {}

		vector<double> get_gripper_coords()
		{
			Transform T_0N = Transform(this->alphas[0], this->as[0], this->ds[0], this->thetas[0]);

			for (int i = 1; i < this->n_frames; i++)
				T_0N = T_0N * Transform(this->alphas[i], this->as[i], this->ds[i], this->thetas[i]);
	
			return T_0N.get_position();
		}

	private:
		int n_frames;
		vector<double> alphas;
		vector<double> as;
		vector<double> ds;
		vector<double> thetas;
};
*/

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

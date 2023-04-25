#ifndef MOTION_PRIMITIVES_H
#define MOTION_PRIMITIVES_H

#include <vector>
#include <cmath>

using std::vector;

namespace motion_primitives
{
    vector<double> home_to_sleep(double phase);
    vector<double> sleep_to_home(double phase);
    vector<double> draw_x(double phase);
    vector<double> draw_line(double phase);
}

#endif

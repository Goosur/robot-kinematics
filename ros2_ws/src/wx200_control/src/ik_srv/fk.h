#ifndef FK_H
#define FK_H

#include <array>
#include <cmath>

namespace FK {

/**
 * Pre-generated DH transform for the widowx 200 robot arm
 */
std::array<double, 3> where(const std::array<double, 5> &thetas);

} // namespace FK

#endif

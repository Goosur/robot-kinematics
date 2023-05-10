#include "fk.h"

std::array<double, 3> FK::where(const std::array<double, 5> &thetas) {
  std::array<std::array<double, 4>, 6> wx200_T05{
      {{std::sin(thetas[0]) * std::sin(thetas[4]) -
            1.0 * std::sin(thetas[1] + thetas[2] + thetas[3]) *
                std::cos(thetas[0]) * std::cos(thetas[4]),
        std::sin(thetas[0]) * std::cos(thetas[4]) +
            1.0 * std::sin(thetas[4]) *
                std::sin(thetas[1] + thetas[2] + thetas[3]) *
                std::cos(thetas[0]),
        -1.0 * std::cos(thetas[0]) *
            std::cos(thetas[1] + thetas[2] + thetas[3]),
        -(1.0 * -174.15 * std::cos(thetas[1] + thetas[2] + thetas[3]) +
          1.0 * 200 * std::cos(thetas[1] + thetas[2]) +
          206.16 * std::sin(thetas[1] - 0.240159419698206)) *
            std::cos(thetas[0])},
       {-1.0 * std::sin(thetas[0]) *
                std::sin(thetas[1] + thetas[2] + thetas[3]) *
                std::cos(thetas[4]) -
            1.0 * std::sin(thetas[4]) * std::cos(thetas[0]),
        1.0 * std::sin(thetas[0]) * std::sin(thetas[4]) *
                std::sin(thetas[1] + thetas[2] + thetas[3]) -
            std::cos(thetas[0]) * std::cos(thetas[4]),
        -1.0 * std::sin(thetas[0]) *
            std::cos(thetas[1] + thetas[2] + thetas[3]),
        -(1.0 * -174.15 * std::cos(thetas[1] + thetas[2] + thetas[3]) +
          1.0 * 200 * std::cos(thetas[1] + thetas[2]) +
          206.16 * std::sin(thetas[1] - 0.240159419698206)) *
            std::sin(thetas[0])},
       {-1.0 * std::cos(thetas[4]) *
            std::cos(thetas[1] + thetas[2] + thetas[3]),
        1.0 * std::sin(thetas[4]) * std::cos(thetas[1] + thetas[2] + thetas[3]),
        1.0 * std::sin(thetas[1] + thetas[2] + thetas[3]),
        1.0 * -174.15 * std::sin(thetas[1] + thetas[2] + thetas[3]) +
            1.0 * 200 * std::sin(thetas[1] + thetas[2]) -
            206.16 * std::cos(thetas[1] - 0.240159419698206)},
       {0, 0, 0, 1}}};

  return {{wx200_T05[0][3], wx200_T05[1][3], wx200_T05[2][3]}};
}

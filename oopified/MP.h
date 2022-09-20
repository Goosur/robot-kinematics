/**
 * @file MP.h
 * @author Devon Gardner, Nick Floyd, Sydney Silverman
 * @brief Motion Primitive class
 * @version 0.1
 * @date 2022-04-04
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef MP_H
#define MP_H

#include <vector>

#include "BasisFunction.h"

using std::vector;

class MP {
public:
  explicit MP(vector<BasisFunction *> funcs);
  vector<double> at(double phase);
  vector<BasisFunction *> getFuncs();

private:
  vector<BasisFunction *> funcs;
};

#endif

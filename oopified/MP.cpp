#include "MP.h"

MP::MP(vector<BasisFunction *> funcs) : funcs(std::move(funcs)) {}

vector<double> MP::at(double phase) {
  vector<double> angles;
  for (BasisFunction *func : funcs)
    angles.push_back(func->at(phase));

  return angles;
}

vector<BasisFunction *> MP::getFuncs() { return this->funcs; }

/**
 * @file BasisFunction.h
 * @author Devon Gardner, Nick Floyd, Sydney Silverman
 * @brief
 * @version 0.1
 * @date 2022-04-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef BASISFUNCTION_H
#define BASISFUNCTION_H

class BasisFunction { // abstract class
public:
  virtual double at(double x) = 0;
  virtual void to_string() = 0;
};

#endif

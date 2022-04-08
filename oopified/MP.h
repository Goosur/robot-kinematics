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
#include <vector>

#include "BasisFunction.h"

using std::vector;

class MP{
    public:
        MP();

        double getFunc( int i );    // returns ??

    private:
        vector<BasisFunction> funcs;  //get length from??
};

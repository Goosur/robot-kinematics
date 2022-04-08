/**
 * @file Robot.h
 * @author Devon Gardner, Nick Floyd, Sydney Silverman
 * @brief 
 * @version 0.1
 * @date 2022-03-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ROBOT_H
#define ROBOT_H

#include "MP.h"
#include <vector>

using std::vector;

class Robot{
    public:
        //constructor
        Robot( int numjoints );

        //getters
        double getAlpha( int i );
        double getA( int i );
        double getD( int i );

        /**
         * @brief Get the DH parameters for joint i
         * 
         * @param i (int) joint number
         * @return (vector<double>) alphas[i-1], as[i-1], ds[i]
         */
        vector<double> getJointParams( int i );

        //setters
        void set_alphas( vector<double> alphas );
        void setAs( vector<double> as );
        void setDs( vector<double> ds );

        //other methods
        void play( MP mp );
        vector<double> fk();    //TODO paramaters??

    private:
        int numjoints;
        vector<double> alphas;
        vector<double> as;
        vector<double> ds;
        vector<double> joints_ids;

        vector<double> position[3];     //current (x,y,z) position
};

#endif

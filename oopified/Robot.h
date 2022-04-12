/**
 * @file Robot.h
 * @author Devon Gardner, Nick Floyd, Sydney Silverman
 * @brief 
 * @version 0.1
 * @date 2022-03-30
 * 
 * @copyright Copyright (c) 2022
 */
#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include "MP.h"
#include "dynamixel_helper.h"

using std::vector;

class Robot
{
public:
	// Constructors
	Robot(int nJoints, const DynamixelHelper& dxlHelper);

	// Actions
	void playMPs(vector<MP> mps);

	// Getters
	vector<int> getJointIDs();
	vector<double> getAlphas();
	vector<double> getAs();
	vector<double> getDs();
	vector<double> getThetas();
	vector<double> getEndPosition();

	// Setters
	void setJointIDs(vector<int> jointIDs);
	void setAlphas(vector<double> alphas);
	void setAs(vector<double> as);
	void setDs(vector<double> ds);
	void setThetas(vector<double> thetas);

private:
	DynamixelHelper dxlHelper;
	int nJoints;
	vector<int> jointIDs;
	vector<double> alphas;
	vector<double> as;
	vector<double> ds;
	vector<double> thetas;
	vector<double> endPosition; // x, y, z
};

#endif

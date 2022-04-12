#include "Robot.h"

// Constructors

Robot::Robot(int nJoints, const DynamixelHelper &dxlHelper) : nJoints(nJoints), dxlHelper(dxlHelper) {}


// Actions

void Robot::playMPs(vector<MP> mps)
{
	double phase;
	for (MP mp : mps)
	{
		phase = 0.0;
		while (phase <= 1.0)
		{
			phase += 0.001;
		}
	}
}


// Getters

vector<double> Robot::getAlphas() { return this->alphas; }

vector<double> Robot::getAs() { return this->as; }

vector<double> Robot::getDs() { return this->ds; }

vector<int> Robot::getJointIDs() { return this->jointIDs; }


// Setters

void Robot::setAlphas(vector<double> alphas) { this->alphas = alphas; }

void Robot::setAs(vector<double> as) { this->as = as; }

void Robot::setDs(vector<double> ds) { this->ds = ds; }

void Robot::setJointIDs(vector<int> jointIDs) { this->jointIDs = jointIDs; }
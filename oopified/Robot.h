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

#include "MP.h"
#include "fk.h"
#include <dynamixel_helper.h>
#include <vector>

using std::vector;

class Robot {
public:
  // Constructors
  Robot(vector<uint8_t> jointIDs, DynamixelHelper *dxlHelper);

  // Actions
  void playMPs();
  void printPosition();

  // Getters
  vector<double> getJointAngles() {
    this->jointAngles =
        std::move(this->dxlHelper->groupGetAngle(this->jointIDs));
    return this->jointAngles;
  }
  vector<double> getGripperPosition() {
    this->thetas = Robot::jointAnglesToThetas(this->jointAngles);
    this->gripperPosition = FK::get_gripper_coords(
        this->nFrames, this->alphas, this->as, this->ds, this->thetas);
    return this->gripperPosition;
  }
  vector<MP> getMPs() { return this->mps; }

  // Setters
  void displayPosition(bool val) { this->positionVisible = val; }

  void setJointIDs(vector<uint8_t> vals) {
    this->jointIDs = std::move(vals);
    this->nJoints = (int)this->jointIDs.size();
  }

  void setNFrames(int n) { this->nFrames = n; }
  void setAlphas(vector<double> vals) { this->alphas = std::move(vals); }
  void setAs(vector<double> vals) { this->as = std::move(vals); }
  void setDs(vector<double> vals) { this->ds = std::move(vals); }

  void addMP(MP &mp) { this->mps.push_back(mp); }
  void clearMPs() { this->mps.clear(); }

private:
  DynamixelHelper *dxlHelper;
  bool positionVisible;

  // TODO: Find a way to make this user easily user definable.
  double movingStatusThreshold = 2 * 20 * 0.088 * M_PI / 180.0;

  // Properties
  int nJoints;
  int nFrames;
  vector<uint8_t> jointIDs;
  vector<double> jointAngles;
  vector<double> alphas;
  vector<double> as;
  vector<double> ds;
  vector<double> thetas;
  vector<double> gripperPosition; // x, y, z
  vector<MP> mps;

  // Functions
  static vector<vector<double>> interpolate(vector<double> initialAngles,
                                            vector<double> finalAngles,
                                            int nSteps);
  static vector<double> jointAnglesToThetas(vector<double> angles);
  void waitForMove();
};

#endif

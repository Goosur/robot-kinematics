#include "Robot.h"

// Constructors
Robot::Robot(vector<uint8_t> jointIDs, DynamixelHelper *dxlHelper)
    : jointIDs(std::move(jointIDs)), dxlHelper(dxlHelper) {
  this->nJoints = (int)this->jointIDs.size();
  this->gripperPosition.resize(3);
  this->jointAngles = std::move(this->dxlHelper->groupGetAngle(this->jointIDs));
}

// Actions
void Robot::playMPs() {
  double phase;
  double dp = 0.001;

  for (MP mp : this->mps) {
    // Interpolate between current robot position and start of current
    // motion primitive to ensure a smooth start or transition between
    // motion primitives.
    for (vector<double> step :
         Robot::interpolate(this->jointAngles, mp.at(0.0), 100)) {
      this->jointAngles = std::move(step);
      this->dxlHelper->groupSetAngle(this->jointIDs, this->jointAngles);
      this->waitForMove();

      if (this->positionVisible)
        this->printPosition();
    }

    // Play the motion current primitive
    phase = 0.0;
    while (phase <= 1.0) {
      // Get next target position from motion primitive
      this->jointAngles = std::move(mp.at(phase));
      // Tell the robot to move to that position
      this->dxlHelper->groupSetAngle(this->jointIDs, this->jointAngles);
      // Make sure goal was reached before moving on
      this->waitForMove();

      if (this->positionVisible)
        this->printPosition();

      phase += dp;
    }
  }
}

void Robot::printPosition() {
  this->thetas = Robot::jointAnglesToThetas(this->jointAngles);
  this->gripperPosition = FK::get_gripper_coords(
      this->nFrames, this->alphas, this->as, this->ds, this->thetas);
  printf("Joint Angles:\t");
  for (double angle : this->jointAngles)
    printf("%.3f\t", angle);
  printf("\tEnd Effector (x, y, z):\t");
  for (double coord : this->gripperPosition)
    printf("%.3f\t", coord);
  printf("\n");
}

// PRIVATE STUFF

vector<vector<double>> Robot::interpolate(vector<double> initialAngles,
                                          vector<double> finalAngles,
                                          int nSteps) {
  vector<vector<double>> interpolatedAngles;

  for (int i = 0; i < finalAngles.size(); i++)
    interpolatedAngles[i].push_back(
        initialAngles[i] + (finalAngles[i] - initialAngles[i]) * i / nSteps);

  return interpolatedAngles;
}

// TODO: This is specific to wx200; needs reworking
vector<double> Robot::jointAnglesToThetas(vector<double> angles) {
  return vector<double>{angles[0],
                        angles[1] - M_PI / 2,
                        M_PI / 2,
                        angles[2] - M_PI,
                        angles[3] - M_PI / 2,
                        angles[4],
                        0.0};
}

void Robot::waitForMove() {
  vector<double> current_angles;
  bool still_moving = true;
  do {
    current_angles = std::move(this->dxlHelper->groupGetAngle(this->jointIDs));
    for (int i = 0; i < this->nJoints; i++)
      still_moving |= std::abs(this->jointAngles[i] - current_angles[i]) >
                      this->movingStatusThreshold;
  } while (still_moving);
}

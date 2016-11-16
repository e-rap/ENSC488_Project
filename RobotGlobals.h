#ifndef RobotGlobals_h__
#define RobotGlobals_h__

#include "matrix.h"

/////////////////////////////////////
// Global Constants and Parameters //
/////////////////////////////////////

////////////////////
// Link Constants //
////////////////////
const double L1 = 405.0f;
const double L2 = 70.0f;
const double L3 = 195.0f;
const double L4 = 142.0f;
const double L5 = 410.0f;
const double L6 = 80.0f;
const double L7 = 60.0f;
const double GRIPPER_OFFSET = -10.0f;
const double PICK_PLACE_TOLERANCE = 10.0f;
const int NUM_OF_LINK_VARS = 4;

//////////////////
// Angle Limits //
//////////////////
const double THETA1_MAX = 150.0f;
const double THETA1_MIN = -150.0f;

const double THETA2_MAX = 100.0f;
const double THETA2_MIN = -100.0f;

const double D3_MAX = -100.0f;
const double D3_MIN = -200.0f;

const double THETA4_MAX = 160.0f;
const double THETA4_MIN = -160.0f;

//////////////////
// Known Frames //
//////////////////
const matrix T_SB = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
const matrix T_WT = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, L7 }, { 0, 0, 0, 1 } };


// Limit Check Functions //
bool Theta1Check(double ang)
{
  return ((ang > THETA1_MAX || ang < THETA1_MIN) ? false : true);
}

bool Theta2Check(double ang)
{
  return ((ang > THETA2_MAX || ang < THETA2_MIN) ? false : true);
}

bool Theta4Check(double ang)
{
  return ((ang > THETA4_MAX || ang < THETA4_MIN) ? false : true);
}

bool D3Check(double dist)
{
  return ((dist > D3_MAX || dist < D3_MIN) ? false : true);
}

void JointToVect(JOINT joint, vect& vector)
{
  for (int i = 0; i < NUM_OF_LINK_VARS; i++)
  {
    vector[i] = joint[i];
  }
}

void VectToJoint(vect vector, JOINT& joint)
{
  for (int i = 0; i < NUM_OF_LINK_VARS; i++)
  {
    joint[i] = vector[i];
  }
}

void GetCurrentConfig(vect& cur_config)
{
  JOINT config;
  GetConfiguration(config);
  JointToVect(config, cur_config);
}
#endif // RobotGlobals_h__
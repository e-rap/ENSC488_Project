#ifndef RobotGlobals_h__
#define RobotGlobals_h__

// Global Constants and Parameters

// Link Constants
const float L1 = 405.0f;
const float L2 = 70.0f;
const float L3 = 195.0f;
const float L4 = 142.0f;
const float L5 = 410.0f;
const float L6 = 80.0f;
const float L7 = 60.0f;
const float GRIPPER_OFFSET = -10.0f;

// Angle Limits
const float THETA1_MAX = 150.0f;
const float THETA1_MIN = -150.0f;

const float THETA2_MAX = 100.0f;
const float THETA2_MIN = -100.0f;

const float D3_MAX = -100.0f;
const float D3_MIN = -200.0f;

const float THETA4_MAX = 160.0f;
const float THETA4_MIN = -160.0f;

// Limit Check Functions
bool Theta1Check(float ang)
{
  return ((ang > THETA1_MAX || ang < THETA1_MIN) ? false : true);
}

bool Theta2Check(float ang)
{
  return ((ang > THETA2_MAX || ang < THETA2_MIN) ? false : true);
}

bool Theta4Check(float ang)
{
  return ((ang > THETA4_MAX || ang < THETA4_MIN) ? false : true);
}

bool D3Check(float dist)
{
  return ((dist > D3_MAX || dist < D3_MIN) ? false : true);
}

#endif // RobotGlobals_h__
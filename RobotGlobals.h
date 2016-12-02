#ifndef RobotGlobals_h__
#define RobotGlobals_h__

#include "matrix.h"
#include <windows.h>
#define DEBUG

//////////////////////////
// Trajectory Constants //
//////////////////////////
#define S_TO_MILIS 1000.0
#define MAX_VIA_POINTS 5
#define SAMPLING_RATE 120
#define MAX_TIME 60
#define MAX_DATA_POINTS SAMPLING_RATE*MAX_TIME
#define filename "viapoints2.txt"

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
const double L8 = 30.0f;
const double GRIPPER_OFFSET = -10.0f;
const double PICK_PLACE_TOLERANCE = 10.0f;
const int NUM_OF_LINK_VARS = 4;
const double m1 = 1.0;
const double m2 = 1.7;
const double m3 = 1.0;
const double m4 = 1.7;
const double g = 9.81*1000;
const double FrictionCoef = 0.0;


////////////////
//Sample Times//
////////////////
double T1=0.1;
double T2=0.01;
double T3=0.001;

//////////////////
// Angle Limits //
//////////////////
const double THETA1_MAX = 150.0f;
const double THETA1_MIN = -150.0f;
const double VelTheta1_MAX=10.0f;


const double THETA2_MAX = 100.0f;
const double THETA2_MIN = -100.0f;
const double VelTheta2_MAX=10.0f;

const double D3_MAX = -100.0f;
const double D3_MIN = -200.0f;
const double VelD3_MAX=10.0f;

const double THETA4_MAX = 160.0f;
const double THETA4_MIN = -160.0f;
const double VelTheta4_MAX=10.0f;

//////////////////
// Known Frames //
//////////////////
const matrix T_SB = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
const matrix T_WT = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, L7 }, { 0, 0, 0, 1 } };


#ifdef DEBUG
// Testing Via Points //
double via_times[5] = { 0, 5, 10, 15, 20 };
double via_x[5] = { 0, 0, 0, 0 ,0};
double via_y[5] = { 320, 315, 310, 305, 300};
double via_z[5] = { 26, 30, 35, 40 , 45};
double via_phi[5] = { 0, 0, 0, 0, 0};
#endif


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

bool Theta1Check2(double ang)
{
  return ((ang >= THETA1_MAX || ang <= THETA1_MIN) ? false : true);
}

bool Theta2Check2(double ang)
{
  return ((ang >= THETA2_MAX || ang <= THETA2_MIN) ? false : true);
}

bool Theta4Check2(double ang)
{
  return ((ang >= THETA4_MAX || ang <= THETA4_MIN) ? false : true);
}

bool D3Check2(double dist)
{
  return ((dist <= D3_MAX || dist >= D3_MIN) ? false : true);
}

bool VelTheta1Check(double vel)
{
    return ((vel > VelTheta1_MAX || vel < -VelTheta1_MAX) ? false : true);
}

bool VelTheta2Check(double vel)
{
    return ((vel > VelTheta2_MAX || vel < -VelTheta2_MAX) ? false : true);
}

bool VelD3Check(double vel)
{
    return ((vel > VelD3_MAX || vel < -VelD3_MAX) ? false : true);
}

bool VelTheta4Check(double vel)
{
    return ((vel > VelTheta4_MAX || vel < -VelTheta4_MAX) ? false : true);
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

void GetCurrentConfig(vect& curConfig)
{
  JOINT Config;
  GetConfiguration(Config);
  VectorCopy(Config, curConfig);
}


/////////////////////
// Timer Functions //
/////////////////////

double PCFreq = 0.0;
__int64 CounterStart = 0;

void StartCounter()
{
  LARGE_INTEGER li;
  if (!QueryPerformanceFrequency(&li))
    std::cout << "QueryPerformanceFrequency failed!\n";

  PCFreq = double(li.QuadPart) / S_TO_MILIS;

  QueryPerformanceCounter(&li);
  CounterStart = li.QuadPart;
}

double GetCounter()
{
  LARGE_INTEGER li;
  QueryPerformanceCounter(&li);
  return double(li.QuadPart - CounterStart) / PCFreq;
}

void microsleep(double duration)
{
  StartCounter();
  while (true)
  {
    if (GetCounter() >= duration)
    {
      return;
    }
  }

}

#endif // RobotGlobals_h__

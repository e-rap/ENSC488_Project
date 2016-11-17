#ifndef TrajectoryPlanning_h__
#define TrajectoryPlanning_h__

#include "matrix.h"
#include "ensc-488.h"
#include "RobotGlobals.h"
#include "InverseKin.h"
#include <exception>


// Description : Uses the Jacobian to convert joint space velocities to Cartesian space velocities
// Inputs:
//   JointConfig - Joint Configuration State (theta1,theta2,d3,theta4)
//   JointVel    - Current Joint Velocities (theta1dot, theta2dot, V3, theta4dot)
// Outputs:
//   CartVel     - Cartesian velocities (Vx, Vy, Vz, phidot)
// Throws Exception At singularities 
void JointVel2CartVel(vect JointConfig, vect JointVel, vect& CartVel)
{

  // calculated the 2x2 det for x y theta1 theta2
  double det = (-L3*sin(DEG2RAD(JointConfig[0])) - L4*sin(DEG2RAD(JointConfig[0] + JointConfig[1])))
               *(L4*cos(DEG2RAD(JointConfig[0] + JointConfig[1]))) + L4*sin(DEG2RAD(JointConfig[0] + JointConfig[1]))
               *(L3*cos(DEG2RAD(JointConfig[0])) + L4*cos(DEG2RAD(JointConfig[0] + JointConfig[1])));

  // Check if at a singularity
  if (det == 0 || JointConfig[2] == -100 || JointConfig[2] == -200 || Theta1Check2(JointConfig[0]) || Theta2Check2(JointConfig[1]) || D3Check2(JointConfig[2]) || Theta2Check2(JointConfig[3]))
  {
    throw std::exception("Encountered Boundary singularity!");
  }

  CartVel[0] = (-L3*sin(DEG2RAD(JointConfig[0])) - L4*sin(DEG2RAD(JointConfig[0] + JointConfig[1]))) * JointVel[0] 
                - L4*sin(DEG2RAD(JointConfig[0] + JointConfig[1])) * JointVel[1];

  CartVel[1] = (L3*cos(DEG2RAD(JointConfig[0])) + L4*cos(DEG2RAD(JointConfig[0] + JointConfig[1]))) * JointVel[0]
    + L4*cos(DEG2RAD(JointConfig[0] + JointConfig[1])) * JointVel[1];
  CartVel[2] = -JointVel[2];
  CartVel[3] = JointVel[0] + JointVel[1] - JointVel[3];
}


// Description : Uses the Inverse of the Jacobian to convert Cartesian space velocities to joint space velocities
// Inputs:
//   CartConfig  - Cartesian configuration vector (x,y,z,phi)
//   CartVel     - Cartesian velocity vector (Vx, Vy, Vz, phidot)
// Outputs:
//   JointVel    - Current joint velocity vector (theta1dot, theta2dot, V3, theta4dot)
// Throws Exception At singularities
void CartVel2JointVel(vect CartConfig, vect CartVel, vect& JointVel)
{
  matrix CartConfigMatrix = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };
  U2I(CartConfig, CartConfigMatrix);
  vect JointConfig = { 0, 0, 0, 0 };
  vect far_sol = { 0, 0, 0, 0 };
  bool sol = false;
  SOLVE(CartConfigMatrix, CartConfig, JointConfig, far_sol, sol);

  JointVel[0] = (cos(DEG2RAD(JointConfig[0] + JointConfig[1])) / (L3*sin(DEG2RAD(JointConfig[1]))))*CartVel[0] + (sin(DEG2RAD(JointConfig[0] + JointConfig[1])) / (L3*sin(DEG2RAD(JointConfig[1]))))*CartVel[1];
  JointVel[1] = (-(L4*cos(DEG2RAD(JointConfig[0] + JointConfig[1])) + L3*cos(DEG2RAD(JointConfig[0]))) / (L3*L4*sin(DEG2RAD(JointConfig[1]))))*CartVel[0] + (-(L4*sin(DEG2RAD(JointConfig[0] + JointConfig[1])) + L3*sin(DEG2RAD(JointConfig[0]))) / (L3*L4*sin(DEG2RAD(JointConfig[1]))))* CartVel[1];
  JointVel[2] = -CartVel[2];
  JointVel[3] = JointVel[0] + JointVel[1] - CartVel[3];

  double det = (-L3*sin(DEG2RAD(JointConfig[0])) - L4*sin(DEG2RAD(JointConfig[0] + JointConfig[1])))
    *(L4*cos(DEG2RAD(JointConfig[0] + JointConfig[1]))) + L4*sin(DEG2RAD(JointConfig[0] + JointConfig[1]))
    *(L3*cos(DEG2RAD(JointConfig[0])) + L4*cos(DEG2RAD(JointConfig[0] + JointConfig[1])));

  // Check if at a singularity
  if (det == 0 || JointConfig[2] == -100 || JointConfig[2] == -200 || Theta1Check2(JointConfig[0]) || Theta2Check2(JointConfig[1]) || D3Check2(JointConfig[2]) || Theta2Check2(JointConfig[3]))
  {
    throw std::exception("Encountered Boundary singularity!");
  }
}

#endif // TrajectoryPlanning_h__
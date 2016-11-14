//
//  InverseKin.h
//  InverseKin
//
//  Created by Eric Raposo on 2016-11-02.
//  Copyright © 2016 Eric Raposo. All rights reserved.
//

#ifndef InverseKin_h__
#define InverseKin_h__

#include "matrix.h"
#include "RobotGlobals.h"
#include <math.h>
#include <limits.h>
#include "ensc-488.h"

// INVKIN - Inverse Kinematics Function 
// @input  WrelB   - T matrix of the tool frame relative to the base frame
// @input  current - current joint states {theta1,theta2,d3,theta4}  
// @output near    - nearest solution wrt current position {theta1,theta2,d3,theta4}
// @output far     - farthest solution wrt current position {theta1,theta2,d3,theta4}
// @output sol     - true of solutions exist otherwise false {theta1,theta2,d3,theta4}
void INVKIN(matrix WrelB, vect current, vect& near, vect& far, bool& sol)
{
  // Relative to the base Frame
  vect DesiredPosition = ITOU(WrelB);
  double x   = DesiredPosition[0];
  double y   = DesiredPosition[1];
  double z   = DesiredPosition[2];
  double phi = DesiredPosition[3];

  //if (sqrt(pow(x, 2) + pow(y, 2) > (L3 + L4)))
  //{
  //  sol = false;
  //  return;
  //}
  //if (sqrt(pow(x, 2) + pow(y, 2) < (L3 - L4)))
  //{
  //  sol = false;
  //  return;
  //}
  //if (z > 125 || z < 25)
  //{
  //  sol = false;
  //  return;
  //}

  // Calculated positions
  double theta1 = 0.0f, theta2 = 0.0f, d3 = 0.0f, theta4 = 0.0f;

  // Vector containing valid solutions
  std::vector< vect> solutions;

  // Inverse calculation
  for (int sign = -1; sign <= 1; sign += 2)
  {
    double costheta2 = (pow(x, 2) + pow(y, 2) - pow(L3, 2) - pow(L4, 2)) / (2*L3*L4);
    theta2 = RAD2DEG(atan2(sign*sqrt(1 - pow(costheta2,2)), costheta2));

  
    double cosAlpha = (pow(x, 2) + pow(y, 2) + pow(L3, 2) - pow(L4, 2)) / (2 * L3*sqrt(pow(x, 2) + pow(y, 2))); // DEG2RAD(acos((pow(x, 2) + pow(y, 2) + pow(L3, 2) - pow(L4, 2)) / (2 * L3*sqrt(pow(x, 2) + pow(y, 2)))));    
    double beta = RAD2DEG(atan2(y, x));
    double alpha = RAD2DEG(atan2(-1*sign*sqrt(1 - pow(cosAlpha, 2)), cosAlpha));
    theta1 = beta + alpha;
    d3 = L1 + L2 - z - 410 - L6 - (2*L7);
    theta4 = theta1 + theta2 -phi;

    vect solution = { theta1, theta2, d3, theta4 };

    // Make sure solution is valid
    if (Theta1Check(theta1) && Theta2Check(theta2) && Theta4Check(theta4) && D3Check(d3))
    {
      solutions.push_back(solution);
    }

  }
  
  if (solutions.size() == 0)
  {
    sol = false;
  }
  else
  {
    sol = true;

    // Find the Nearest and Farthest Solution wrt. the current joint config
    double nearest = std::numeric_limits<double>::max();
    double farthest = std::numeric_limits<double>::min();
    for (unsigned int i = 0; i < solutions.size(); i++)
    {
      double diffsum = VectorDiffSum(solutions[i], current);
      if (diffsum > farthest)
      {
        farthest = diffsum;
        far = solutions[i];
      }
      if (diffsum < nearest)
      {
        nearest = diffsum;
        near = solutions[i];
      }
    }
  }

  return;
}

//tool is attached to link 3 of the manipulator.This tool is described by T_WT, the
//tool frame relative to the wrist frame. Also, a user has described his work area, the
//station frame relative to the base of the robot, as T_BS .Write the subroutine

//Procedure SOLVE(VAR - trels: frame; VAR current, near, far: vec3, VAR sol : boolean);

//where "trels" is the{ T } frame specified relative to the{ S } frame.Other parameters
//are exactly as in the INVKIN subroutine.The defmitions of{ T } and{ S } should be
//globally defined variables or constants.SOLVE should use calls to TMULT, TINVERT,
//and INVKIN.

void SOLVE(matrix TRelS, vect& current, vect& near, vect& far, bool& sol)
{
  matrix WRelB = Multiply(Multiply(Inverse(BRelS), TRelS),Inverse(TRelW));
  INVKIN(WRelB, current, near, far, sol);
}



#endif // InverseKin_h__
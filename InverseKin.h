//
//  InverseKin.h
//  InverseKin
//
//  Created by Eric Raposo on 2016-11-02.
//  Copyright � 2016 Eric Raposo. All rights reserved.
//

#ifndef InverseKin_h__
#define InverseKin_h__

#include "matrix.h"
#include "RobotGlobals.h"
#include <math.h>
#include <limits.h>
#include "ensc-488.h"

#define MAX_NUM_SOLUTIONS 2

// INVKIN - Inverse Kinematics Function 
// @input  WrelB   - T matrix of the tool frame relative to the base frame
// @input  current - current joint states {theta1,theta2,d3,theta4}  
// @output near    - nearest solution wrt current position {theta1,theta2,d3,theta4}
// @output far     - farthest solution wrt current position {theta1,theta2,d3,theta4}
// @output sol     - true of solutions exist otherwise false {theta1,theta2,d3,theta4}
void INVKIN(matrix WrelB, vect current, vect& near, vect& far, bool& sol)
{
  sol = false;

  // Relative to the base Frame
  vect DesiredPosition = { 0, 0, 0, 0 };
  ITOU(WrelB,DesiredPosition);

  double x   = DesiredPosition[0];
  double y   = DesiredPosition[1];
  double z   = DesiredPosition[2];
  double phi = DesiredPosition[3];

  /*if (sqrt(pow(x, 2) + pow(y, 2)) > (L3 + L4))
  {
  sol = false;
  std::cout<<"position is outside of the workspace"<<std::endl;
  return;
  }
  if (sqrt(pow(x, 2) + pow(y, 2)) < (L3 - L4))
  {
  sol = false;
  std::cout << "position is outside of the workspace" << std::endl;
  return;
  }
  if ((z > 125) || (z < 25))
  {
  sol = false;
  std::cout << "position is outside of the workspace" << std::endl;
  return;
  }*/

  // Calculated positions
  double theta1 = 0.0f, theta2 = 0.0f, d3 = 0.0f, theta4 = 0.0f;

  // Vector containing valid solutions
  vect solutions[MAX_NUM_SOLUTIONS] = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }};

  // Inverse calculation
  for (int sign = -1; sign <= 1; sign += 2)
  {
    double costheta2 = (pow(x, 2) + pow(y, 2) - pow(L3, 2) - pow(L4, 2)) / (2*L3*L4);
    theta2 = atan2(sign*sqrt(1 - pow(costheta2,2)), costheta2);

    //double cosAlpha = (pow(x, 2) + pow(y, 2) + pow(L3, 2) - pow(L4, 2)) / (2 * L3*sqrt(pow(x, 2) + pow(y, 2))); // DEG2RAD(acos((pow(x, 2) + pow(y, 2) + pow(L3, 2) - pow(L4, 2)) / (2 * L3*sqrt(pow(x, 2) + pow(y, 2)))));
    //double beta = RAD2DEG(atan2(y, x));
    //double alpha = RAD2DEG(atan2(sqrt(1 - pow(cosAlpha, 2)), cosAlpha));
    // theta1 = beta - sign*alpha;
    theta1= RAD2DEG(atan2(((L3+L4*cos(theta2))*y-L4*sin(theta2)*x),((L3+L4*cos(theta2))*x+L4*sin(theta2)*y)));
    theta2= RAD2DEG(theta2);
    d3 = L1 + L2 - z - 410 - L6;
    theta4 = theta1 + theta2 -phi;

    vect solution = { theta1, theta2, d3, theta4 };

    // Make sure solution is valid
    if (Theta1Check(theta1) && Theta2Check(theta2) && Theta4Check(theta4) && D3Check(d3))
    {
      VectorCopy(solution, solutions[(sign + 1) / 2]);
      sol = true;
    }
  }
  
  //if (solutions.size() == 0)
  //{
  //  sol = false;
  //}
  if (sol == true)
  {
    // Find the Nearest and Farthest Solution wrt. the current joint config
    double nearest = 999999999; // Very Large Numbers for init purposes
    double farthest = -999999999; // Very Small number for init purposes 
    for (unsigned int i = 0; i < MAX_NUM_SOLUTIONS; i++)
    {
      // Invalid Solution if Vector is all 0 (d3 cannot be equal to 0)
      if (solutions[i][2] == 0)
      {
        continue;
      }

      double diffsum = VectorDiffSum(solutions[i], current);
      if (diffsum > farthest)
      {
        farthest = diffsum;
        VectorCopy(solutions[i], far);
      }
      if (diffsum < nearest)
      {
        nearest = diffsum;
        VectorCopy(solutions[i], near);
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
  matrix SRelB = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };
  matrix WRelT = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };

  matrix TRelB = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };

  matrix WRelB = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };


  // WRelB = (SRelB * TRelS) * WRelT 
  Inverse(BRelS, SRelB);
  Inverse(TRelW, WRelT);
  Multiply(SRelB, TRelS, TRelB);
  Multiply(TRelB, TRelW/* TODO: Should Be WRelT BUT ITS NOT??????? */, WRelB); //TODO: FIX THIS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  //matrix WRelB = Multiply(Multiply(Inverse(BRelS), TRelS),Inverse(TRelW));

  INVKIN(WRelB, current, near, far, sol);
}



#endif // InverseKin_h__
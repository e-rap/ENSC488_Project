#ifndef InverseKin_h__
#define InverseKin_h__

// Inverse Kin Functions

//INVKIN(VAR wreib : frame; VAR current, near, far: vec3, VAR sol : boolean);

//where "wreib," an input, is the wrist frame specified relative to the base frame;
//"current," an input, is the current position of the robot(given as a vector of joint
//  angles); "near" is the nearest solution; "far" is the second solution; and "sol" is
//  a flag that indicates whether solutions were found. (sol = FALSE if no solutions
//  were found).The link lengths(meters) are

#include "matrix.h"
#include "RobotGlobals.h"
#include <math.h>
#include <limits.h>

// INPUTS: DesiredPosition = {x,y,z,phi} | units (mm,deg)

void INVKIN(std::vector<float> DesiredPosition, std::vector<float> current, std::vector<float>& near, std::vector<float>& far, bool& sol)
{

  // Relative to the base Frame
  float x   = DesiredPosition[0];
  float y   = DesiredPosition[1];
  float z   = DesiredPosition[2];
  float phi = DesiredPosition[3];

  //current positions
  float cur_theta1 = current[0];
  float cur_theta2 = current[1];
  float cur_d3 = current[2];
  float cur_theta4 = current[3];

  // Calculated positions
  float theta1 = 0.0f;
  float theta2 = 0.0f;
  float d3     = 0.0f;
  float theta4 = 0.0f;

  std::vector< std::vector<float>> solutions;

  //TODO: Add Limit Defines

  // Calculate the Inverse Kin
  for (int sign = -1; sign <= 1; sign += 2)
  {
    float costheta2 = (pow(x, 2) + pow(y, 2) - pow(L3, 2) - pow(L4, 2)) / (2*L3*L4);

    theta2 = atan2f(sign*sqrtf(1 - costheta2), costheta2);

    float alpha = acosf(pow(x, 2) + pow(y, 2) + pow(L3, 2) - pow(L4, 2) / (2 * L3*sqrt(pow(x, 2) + pow(y, 2))));
    float beta = atan2f(y, x);
    for (int sign2 = -1; sign2 <= 1; sign2 += 2)
    {
      theta1 = beta + sign2*alpha;

      d3 = z + 2 * L6 + L7 + 410 - L1 - L2;

      theta4 = theta1 + theta2 -phi;

      std::vector<float> solution = { theta1, theta2, d3, theta4 };

      // If Valid Add to solutions
      if (Theta1Check(theta1) && Theta2Check(theta2) && Theta4Check(theta4) && D3Check(d3))
      {
        solutions.push_back(solution);
      }
      if (solutions.size() == 0)
      {
        sol = false;
      }
      else
      {
        sol = true;
        float nearest = std::numeric_limits<float>::max();
        float farthest = 0.0f;
        for (int i = 0; i < solutions.size(); i++)
        {
          float diffsum = VectorDiffSum(solutions[i], current);
          if (diffsum > farthest)
          {
            farthest = diffsum;
            far.clear();
            far = solutions[i];
          }
          if (diffsum < nearest)
          {
            nearest = diffsum;
            near.clear();
            near = solutions[i];
          }
        }
      }

    }
  }





}

//tool is attached to link 3 of the manipulator.This tool is described by the
//tool frame relative to the wrist frame.Also, a user has described his work area, the
//station frame relative to the base of the robot, as T.Write the subroutine

//Procedure SOLVE(VAR - brels: frame; VAR current, near, far: vec3, VAR sol : boolean);

//where "trels" is the{ T } frame specified relative to the{ S } frame.Other parameters
//are exactly as in the INVKIN subroutine.The defmitions of{ T } and{ S } should be
//globally defined variables or constants.SOLVE should use calls to TMULT, TINVERT,
//and INVKIN.

void SOLVE(matrix BRelS, std::vector<float> current, std::vector<float> near, std::vector<float> far, bool sol)
{

}



#endif // InverseKin_h__
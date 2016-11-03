//
//  WHERE.h
//  WHERE
//
//  Created by Clara Tsang on 2016-11-01.
//  Copyright © 2016 Clara Tsang. All rights reserved.
//

#ifndef WHERE_h
#define WHERE_h

#include <stdio.h>
#include "matrix.h"
#include "RobotGlobals.h"
#include "ensc-488.h"

// Outputs the Matrix relative to the base
matrix KIN(double theta1, double theta2, double d3, double theta4)
{
  theta1 = DEG2RAD(theta1);
  theta2 = DEG2RAD(theta2);
  theta4 = DEG2RAD(theta4);

    matrix T01(4, vect(4));
    matrix T12(4, vect(4));
    matrix T23(4, vect(4));
    matrix T34(4, vect(4));
    matrix T04(4, vect(4));
    
    //assigning T matrices
    
    T01 = { {cos(theta1), -sin(theta1), 0, 0}, {sin(theta1), cos(theta1), 0, 0}, {0, 0, 1, L1}, {0, 0, 0, 1} };
    T12 = { {cos(theta2), -sin(theta2), 0, L3}, {sin(theta2), cos(theta2), 0, 0}, {0, 0, 1, L2}, {0, 0, 0, 1} };
    T23 = { {1, 0, 0, L4}, {0, -1, 0, 0}, {0, 0, -1, 0}, {0, 0, 0, 1} };
    T34 = { {cos(theta4), -sin(theta4), 0, 0}, {sin(theta4), cos(theta4), 0, 0}, {0, 0, 1, L6 + L5 + d3}, {0, 0, 0, 1} };
    
    //multiplying them to achieve T05
    
    T04 = Multiply(T01, T12);
    T04 = Multiply(T04, T23);
    T04 = Multiply(T04, T34);
    
    //DisplayM(T04);
    
    return T04;
    
}

// Outputs Vector relative to the station frame
vect WHERE(double theta1, double theta2, double d3, double theta4)
{
  vect v(4);
  matrix output;

  if (Theta1Check(theta1) && Theta2Check(theta2) && Theta4Check(theta4) && D3Check(d3))
  {
    //calling function KIN to calculate T05
    output = Multiply(Multiply(BRelS, KIN(theta1, theta2, d3, theta4)), TRelW);
  }
  else
  {
    vect x;
    std::cout << "Invalid Input Params.\n";
    return x;
  }

  //converting matrix into vector for user
  v = ITOU(output);

  return v;
}

#endif /* WHERE_h */

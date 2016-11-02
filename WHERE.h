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

matrix KIN(float theta1, float theta2, float d3, float theta4);

std::vector<float> WHERE()
{
    std::vector<float> v(4);
    float theta1, theta2, d3, theta4;
    matrix output(4, std::vector<float>(4));
    
    //prompting user for joint variables
    
    std::cout << "Please enter the first joint variable THETA1" << std::endl;
    std::cin >> theta1;
    std::cout << "Please enter the second joint variable THETA2" << std::endl;
    std::cin >> theta2;
    std::cout << "Please enter the third joint variable D3" << std::endl;
    std::cin >> d3;
    std::cout << "Please enter the fourth joint variable THETA4" << std::endl;
    std::cin >> theta4;
    std::cout << std::endl << std::endl;
    
    //calling function KIN to calculate T05
    
    output = KIN(theta1, theta2, d3, theta4);
    
    //converting matrix into vector for user
    
    v = ITOU(output);
    DisplayV(v);
    
    return v;
}


matrix KIN(float theta1, float theta2, float d3, float theta4)
{
    matrix T01(4, std::vector<float>(4));
    matrix T12(4, std::vector<float>(4));
    matrix T23(4, std::vector<float>(4));
    matrix T34(4, std::vector<float>(4));
    matrix T45(4, std::vector<float>(4));
    matrix T05(4, std::vector<float>(4));
    
    //assigning T matrices
    
    T01 = { {cos(theta1), -sin(theta1), 0, 0}, {sin(theta1), cos(theta1), 0, 0}, {0, 0, 1, L1}, {0, 0, 0, 1} };
    T12 = { {cos(theta2), -sin(theta2), 0, L3}, {sin(theta2), cos(theta2), 0, 0}, {0, 0, 1, L2}, {0, 0, 0, 1} };
    T23 = { {1, 0, 0, L4}, {0, -1, 0, 0}, {0, 0, -1, 0}, {0, 0, 0, 1} };
    T34 = { {cos(theta4), -sin(theta4), 0, 0}, {sin(theta4), cos(theta4), 0, 0}, {0, 0, 1, L6 + L5 + d3}, {0, 0, 0, 1} };
    T45 = { {1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, L7}, {0, 0, 0, 1} };
    
    //multiplying them to achieve T05
    
    T05 = Multiply(T01, T12);
    T05 = Multiply(T05, T23);
    T05 = Multiply(T05, T34);
    T05 = Multiply(T05, T45);
    
    DisplayM(T05);
    
    return T05;
    
}

#endif /* WHERE_h */

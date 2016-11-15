//
//  PickAndPlace.h
//  
//
//  Created by Christian Liedl on 02.11.16.
//  Copyright © 2016 Christian Liedl. All rights reserved.
//

#ifndef PickAndPlace_h
#define PickAndPlace_h

#include "InverseKin.h"
#include "RobotGlobals.h"

//moves Object from pos0 to pos1; positions wrt station frame T -> specify Base relative to S (User input)
//void PickAndPlace(vect pos0 , vect pos1, double objectHeight){
//    
//    
//
//    matrix T_0(4, vect(4));
//    matrix T_1(4, vect(4));
//    matrix T_0i(4, vect(4));
//    matrix T_1i(4, vect(4));
//    vect temp = { 0, 0, objectHeight + PICK_PLACE_TOLERANCE, 0 };  
//    vect pos0i = VectorAdd(pos0, temp);
//    vect pos1i = VectorAdd(pos1, temp);
//
//    vect temp1 = { 0, 0, objectHeight - GRIPPER_OFFSET, 0};
//    pos0 = VectorAdd(pos0, temp1);
//    pos1 = VectorAdd(pos1, temp1);
//
//    T_0=UTOI(pos0);
//    T_1=UTOI(pos1);
//    T_0i = UTOI(pos0i);
//    T_1i = UTOI(pos1i);
//
//    vect current(4);
//    vect cur_solution(4);
//    JOINT solution;
//    JOINT velocity = { 5, 5, 5, 5 };
//    JOINT accel = { 0, 0, 0, 0 };
//    bool sol;
//    
//    //get current config
//    for (int i=0; i<4; i++){
//        current[i]=GetCurrentConfig()[i];
//    }
//    //find sol0i
//    SOLVE(T_0i,current,cur_solution,vect(),sol);
//    if (sol == false)
//    {
//      std::cout << "ERROR: No solutions found.\n";
//      return;
//    }
//    //move to size mm above sol0
//    VectToJoint(cur_solution, solution);
//    MoveToConfiguration(solution, true);
//
//    //get current config
//    for (int i = 0; i < 4; i++){
//      current[i] = GetCurrentConfig()[i];
//    }
//    //find sol0
//    SOLVE(T_0, current, cur_solution, vect(), sol);
//    if (sol == false)
//    {
//      std::cout << "ERROR: No solutions found.\n";
//      return;
//    }
//    //move to size mm above sol0
//    VectToJoint(cur_solution, solution);
//    MoveToConfiguration(solution, true);
//    //TODO: Add delay maybe?
//    Grasp(true);
//
//    //slowly move down to pos0 with velocity 5mm/s
//    //get current config
//    for (int i = 0; i < 4; i++){
//      current[i] = GetCurrentConfig()[i];
//    }
//    //find sol0
//    SOLVE(T_1i, current, cur_solution, vect(), sol);
//    if (sol == false)
//    {
//      std::cout << "ERROR: No solutions found.\n";
//      return;
//    }
//    //move to size mm above sol0
//    VectToJoint(cur_solution, solution);
//    MoveToConfiguration(solution,true);
//
//    //slowly move down to pos1 with vel 5mm/s
//    //get current config
//    for (int i = 0; i < 4; i++){
//      current[i] = GetCurrentConfig()[i];
//    }
//    //find sol0
//    SOLVE(T_1, current, cur_solution, vect(), sol);
//    if (sol == false)
//    {
//      std::cout << "ERROR: No solutions found.\n";
//      return;
//    }
//    //move to size mm above sol1
//    VectToJoint(cur_solution, solution);
//    MoveToConfiguration(solution, true);
//    //TODO: Add delay?
//    Grasp(false);
//
//    std::cout << "Object Successfully moved.\n";
//}

#endif /* PickAndPlace_h */

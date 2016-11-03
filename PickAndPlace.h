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

//moves Object from pos0 to pos1; positions wrt station frame T -> specify Base relative to S (User input)
void PickAndPlace(vector<double> pos0 , vector<double> pos1, double size, vector<double> BRelSuser){
    
    matrix T_0(4, vector<double>(4));
    matrix T_1(4, vector<double>(4));
    matrix BRelS(4, vector<double>(4));
    BRelS=UTOI(BRelSuser);
    T_0=UTOI(pos0);
    T_1=UTOI(pos1);
    vector<double> current0(4);
    vector<double> near0(4);
    vector<double> far0(4);
    bool sol0;
    vector<double> current1(4);
    vector<double> near1(4);
    vector<double> far1(4);
    bool sol1;
    
    //get current config
    for (int i=0; i<4; i++){
        current0[i]=GetConfiguration()[i];
    }
    //find sol0
    SOLVE(T_0,current0,near0,far0,sol0);
    
    //move to size mm above sol0
    MoveToConfiguration(near0[0],near0[1],near0[2]-size,near0[3]);
    //slowly move down to pos0 with velocity 5mm/s
    MoveWithConfVelAcc(near0[0],near0[1],near0[2],near0[3],5,0);
    //grab object
    Grasp(true);
    //slowly move back up
    MoveWithConfVelAcc(near0[0],near0[1],near0[2]-size,near0[3],5,0);
    //move to size mm above pos1
    
    //get current config
    for (int i=0; i<4; i++){
        current1[i]=GetConfiguration()[i];
    }
    //find sol1
    SOLVE(T_1,current1,near1,far1,sol1);
    
    MoveToConfiguration(near1[0],near1[1],near1[2]-size,near1[3]);
    ////slowly move down to pos1 with velocity 5mm/s
    MoveWithConfVelAcc(near1[0],near1[1],near1[2],near1[3],5);
    //release object
    Grasp(false);
    MoveWithConfVelAcc(near1[0],near1[1],near1[2]-size,near1[3],5,0);
    
    
}

#endif /* PickAndPlace_h */

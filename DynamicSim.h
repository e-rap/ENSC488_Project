//
//  DynamicSim.h
//  test
//
//  Created by Christian Liedl on 30.11.16.
//  Copyright © 2016 Christian Liedl. All rights reserved.
//




#include "matrix.h"
#include "RobotGlobals.h"
#include <exception>
#include <cmath>
#include "InverseKin.h"
#include <fstream>

#ifndef DynamicSim_h
#define DynamicSim_h

void Torque2Joint(vect Torque, vect JointPos, vect JointVel, vect JointAccel, vect& JointPosOut, vect& JointVelOut, vect& JointAccelOut){
    
    
    double theta1=DEG2RAD(JointPos[0]);
    double theta2=DEG2RAD(JointPos[1]);
    double d3=JointPos[2];
    double theta4=DEG2RAD(JointPos[3]);
    
    double thetav1=DEG2RAD(JointVel[0]);
    double thetav2=DEG2RAD(JointVel[1]);
    double dv3=JointVel[2];
    double thetav4=DEG2RAD(JointVel[4]);
    
    double thetaa1=DEG2RAD(JointAccel[0]);
    double thetaa2=DEG2RAD(JointAccel[1]);
    double da3=JointAccel[2];
    double thetaa4=DEG2RAD(JointAccel[4]);
    
    double T1=Torque[0];
    double T2=Torque[1];
    double f3=Torque[2];
    double T4=Torque[3];
    
    JointAccelOut[0] = (1.0/(L3*L3)*(L4*L8*T1*m3*2.0+L4*L8*T1*m4*2.0-L4*L8*T2*m3*2.0-L4*L8*T2*m4*2.0-L3*L8*T2*m3*cos(theta2)*2.0-L3*L8*T2*m4*cos(theta2)*2.0-L3*L8*T4*m3*cos(theta2)*2.0-L3*L8*T4*m4*cos(theta2)*2.0-L4*L8*T1*m4*pow(cos(theta4),2.0)+L4*L8*T2*m4*pow(cos(theta4),2.0)+L3*(L4*L4)*L8*(m3*m3)*(thetav1*thetav1)*sin(theta2)*2.0+L3*(L4*L4)*L8*(m3*m3)*(thetav2*thetav2)*sin(theta2)*2.0+L3*(L4*L4)*L8*(m4*m4)*(thetav1*thetav1)*sin(theta2)+L3*(L4*L4)*L8*(m4*m4)*(thetav2*thetav2)*sin(theta2)+L3*L4*T4*m3*sin(theta2)*sin(theta4)+L3*L4*T4*m4*sin(theta2)*sin(theta4)+(L3*L3)*L4*L8*(m3*m3)*(thetav1*thetav1)*sin(theta2*2.0)+(L3*L3)*L4*L8*(m4*m4)*(thetav1*thetav1)*sin(theta2*2.0)*(1.0/2.0)+L3*L8*T2*m4*cos(theta2)*pow(cos(theta4),2.0)+L3*L8*T4*m4*cos(theta2)*pow(cos(theta4),2.0)+L3*(L4*L4)*L8*m3*m4*(thetav1*thetav1)*sin(theta2)*3.0+L3*(L4*L4)*L8*m3*m4*(thetav2*thetav2)*sin(theta2)*3.0+L3*(L4*L4)*L8*(m3*m3)*thetav1*thetav2*sin(theta2)*4.0+L3*(L4*L4)*L8*(m4*m4)*thetav1*thetav2*sin(theta2)*2.0+L3*L8*T2*m4*cos(theta4)*sin(theta2)*sin(theta4)+L3*L8*T4*m4*cos(theta4)*sin(theta2)*sin(theta4)+(L3*L3)*L4*L8*m3*m4*(thetav1*thetav1)*sin(theta2*2.0)*(3.0/2.0)-L3*L4*(L8*L8)*(m4*m4)*thetav1*sin(theta2)*sin(theta4)-L3*L4*(L8*L8)*(m4*m4)*thetav2*sin(theta2)*sin(theta4)+L3*L4*(L8*L8)*(m4*m4)*thetav4*sin(theta2)*sin(theta4)+L3*(L4*L4)*L8*m3*m4*thetav1*thetav2*sin(theta2)*6.0-L3*L4*(L8*L8)*m3*m4*thetav1*sin(theta2)*sin(theta4)-L3*L4*(L8*L8)*m3*m4*thetav2*sin(theta2)*sin(theta4)+L3*L4*(L8*L8)*m3*m4*thetav4*sin(theta2)*sin(theta4)))/(L4*L8*(m2*m3*2.0+m2*m4*2.0+m3*m4*3.0-(m3*m3)*pow(cos(theta2),2.0)*2.0-(m4*m4)*pow(cos(theta2),2.0)+(m3*m3)*2.0+m4*m4-m3*m4*pow(cos(theta2),2.0)*3.0-m2*m4*pow(cos(theta4),2.0)));
    
    JointAccelOut[1] = -(1.0/(L3*L3)*1.0/(L4*L4)*((L3*L3)*L8*T2*m2*-4.0-(L3*L3)*L8*T2*m3*4.0+(L4*L4)*L8*T1*m3*4.0-(L3*L3)*L8*T2*m4*3.0-(L3*L3)*L8*T4*m2*4.0+(L4*L4)*L8*T1*m4*3.0-(L4*L4)*L8*T2*m3*4.0-(L3*L3)*L8*T4*m3*4.0-(L4*L4)*L8*T2*m4*3.0-(L3*L3)*L8*T4*m4*3.0+L3*(L4*L4)*T4*m3*cos(theta2-theta4)+L3*(L4*L4)*T4*m4*cos(theta2-theta4)-(L4*L4)*L8*T1*m4*cos(theta4*2.0)+(L4*L4)*L8*T2*m4*cos(theta4*2.0)+(L3*L3)*L4*T4*m3*cos(theta2*2.0-theta4)+(L3*L3)*L4*T4*m4*cos(theta2*2.0-theta4)+(L3*L3)*L8*T2*m4*cos(theta2*2.0-theta4*2.0)+(L3*L3)*L8*T4*m4*cos(theta2*2.0-theta4*2.0)-L3*(L4*L4)*T4*m3*cos(theta2+theta4)-L3*(L4*L4)*T4*m4*cos(theta2+theta4)-(L3*L3)*L4*T4*m2*cos(theta4)*2.0-(L3*L3)*L4*T4*m3*cos(theta4)-(L3*L3)*L4*T4*m4*cos(theta4)-L3*L4*L8*T1*m4*cos(theta2-theta4*2.0)+L3*L4*L8*T2*m4*cos(theta2-theta4*2.0)*2.0+L3*L4*L8*T4*m4*cos(theta2-theta4*2.0)+(L3*L3)*L4*(L8*L8)*(m4*m4)*thetav1*cos(theta4)+(L3*L3)*L4*(L8*L8)*(m4*m4)*thetav2*cos(theta4)-(L3*L3)*L4*(L8*L8)*(m4*m4)*thetav4*cos(theta4)+L3*(L4*L4*L4)*L8*(m3*m3)*(thetav1*thetav1)*sin(theta2)*4.0+(L3*L3*L3)*L4*L8*(m3*m3)*(thetav1*thetav1)*sin(theta2)*4.0+L3*(L4*L4*L4)*L8*(m3*m3)*(thetav2*thetav2)*sin(theta2)*4.0+L3*(L4*L4*L4)*L8*(m4*m4)*(thetav1*thetav1)*sin(theta2)*2.0+(L3*L3*L3)*L4*L8*(m4*m4)*(thetav1*thetav1)*sin(theta2)*2.0+L3*(L4*L4*L4)*L8*(m4*m4)*(thetav2*thetav2)*sin(theta2)*2.0-L3*(L4*L4)*(L8*L8)*(m4*m4)*thetav1*cos(theta2-theta4)-L3*(L4*L4)*(L8*L8)*(m4*m4)*thetav2*cos(theta2-theta4)+L3*(L4*L4)*(L8*L8)*(m4*m4)*thetav4*cos(theta2-theta4)-(L3*L3)*L4*(L8*L8)*(m4*m4)*thetav1*cos(theta2*2.0-theta4)-(L3*L3)*L4*(L8*L8)*(m4*m4)*thetav2*cos(theta2*2.0-theta4)+(L3*L3)*L4*(L8*L8)*(m4*m4)*thetav4*cos(theta2*2.0-theta4)+L3*L4*L8*T1*m3*cos(theta2)*4.0+L3*L4*L8*T1*m4*cos(theta2)*3.0-L3*L4*L8*T2*m3*cos(theta2)*8.0-L3*L4*L8*T2*m4*cos(theta2)*6.0-L3*L4*L8*T4*m3*cos(theta2)*4.0-L3*L4*L8*T4*m4*cos(theta2)*3.0+(L3*L3)*(L4*L4)*L8*(m3*m3)*(thetav1*thetav1)*sin(theta2*2.0)*4.0+(L3*L3)*(L4*L4)*L8*(m3*m3)*(thetav2*thetav2)*sin(theta2*2.0)*2.0+(L3*L3)*(L4*L4)*L8*(m4*m4)*(thetav1*thetav1)*sin(theta2*2.0)*2.0+(L3*L3)*(L4*L4)*L8*(m4*m4)*(thetav2*thetav2)*sin(theta2*2.0)+L3*(L4*L4)*(L8*L8)*(m4*m4)*thetav1*cos(theta2+theta4)+L3*(L4*L4)*(L8*L8)*(m4*m4)*thetav2*cos(theta2+theta4)-L3*(L4*L4)*(L8*L8)*(m4*m4)*thetav4*cos(theta2+theta4)+(L3*L3)*L4*(L8*L8)*m2*m4*thetav1*cos(theta4)*2.0+(L3*L3)*L4*(L8*L8)*m2*m4*thetav2*cos(theta4)*2.0+(L3*L3)*L4*(L8*L8)*m3*m4*thetav1*cos(theta4)+(L3*L3)*L4*(L8*L8)*m3*m4*thetav2*cos(theta4)-(L3*L3)*L4*(L8*L8)*m2*m4*thetav4*cos(theta4)*2.0-(L3*L3)*L4*(L8*L8)*m3*m4*thetav4*cos(theta4)+(L3*L3*L3)*L4*L8*m2*m3*(thetav1*thetav1)*sin(theta2)*4.0+(L3*L3*L3)*L4*L8*m2*m4*(thetav1*thetav1)*sin(theta2)*3.0+L3*(L4*L4*L4)*L8*m3*m4*(thetav1*thetav1)*sin(theta2)*6.0+(L3*L3*L3)*L4*L8*m3*m4*(thetav1*thetav1)*sin(theta2)*6.0+L3*(L4*L4*L4)*L8*m3*m4*(thetav2*thetav2)*sin(theta2)*6.0+L3*(L4*L4*L4)*L8*(m3*m3)*thetav1*thetav2*sin(theta2)*8.0+L3*(L4*L4*L4)*L8*(m4*m4)*thetav1*thetav2*sin(theta2)*4.0-L3*(L4*L4)*(L8*L8)*m3*m4*thetav1*cos(theta2-theta4)-L3*(L4*L4)*(L8*L8)*m3*m4*thetav2*cos(theta2-theta4)+L3*(L4*L4)*(L8*L8)*m3*m4*thetav4*cos(theta2-theta4)-(L3*L3*L3)*L4*L8*m2*m4*(thetav1*thetav1)*sin(theta2-theta4*2.0)-(L3*L3)*L4*(L8*L8)*m3*m4*thetav1*cos(theta2*2.0-theta4)-(L3*L3)*L4*(L8*L8)*m3*m4*thetav2*cos(theta2*2.0-theta4)+(L3*L3)*L4*(L8*L8)*m3*m4*thetav4*cos(theta2*2.0-theta4)+(L3*L3)*(L4*L4)*L8*m3*m4*(thetav1*thetav1)*sin(theta2*2.0)*6.0+(L3*L3)*(L4*L4)*L8*m2*m4*(thetav1*thetav1)*sin(theta4*2.0)+(L3*L3)*(L4*L4)*L8*m3*m4*(thetav2*thetav2)*sin(theta2*2.0)*3.0+(L3*L3)*(L4*L4)*L8*m2*m4*(thetav2*thetav2)*sin(theta4*2.0)+L3*(L4*L4)*(L8*L8)*m3*m4*thetav1*cos(theta2+theta4)+L3*(L4*L4)*(L8*L8)*m3*m4*thetav2*cos(theta2+theta4)-L3*(L4*L4)*(L8*L8)*m3*m4*thetav4*cos(theta2+theta4)+(L3*L3)*(L4*L4)*L8*(m3*m3)*thetav1*thetav2*sin(theta2*2.0)*4.0+(L3*L3)*(L4*L4)*L8*(m4*m4)*thetav1*thetav2*sin(theta2*2.0)*2.0+(L3*L3)*(L4*L4)*L8*m3*m4*thetav1*thetav2*sin(theta2*2.0)*6.0+(L3*L3)*(L4*L4)*L8*m2*m4*thetav1*thetav2*sin(theta4*2.0)*2.0+L3*(L4*L4*L4)*L8*m3*m4*thetav1*thetav2*sin(theta2)*1.2E1))/(L8*(m2*m3*4.0+m2*m4*3.0+m3*m4*3.0-(m3*m3)*cos(theta2*2.0)*2.0-(m4*m4)*cos(theta2*2.0)+(m3*m3)*2.0+m4*m4-m3*m4*cos(theta2*2.0)*3.0-m2*m4*cos(theta4*2.0)));
    
    JointAccelOut[2] = (f3+g*(m3+m4))/(m3+m4);
    
    JointAccelOut[3] = -(1.0/(L4*L4)*1.0/(L8*L8)*(-L3*(L4*L4)*T4*(m3*m3)-L3*(L4*L4)*T4*(m4*m4)-L3*(L8*L8)*T2*(m4*m4)*3.0-L3*(L8*L8)*T4*(m4*m4)*3.0+(L4*L4)*L8*T1*(m4*m4)*cos(theta2+theta4)-(L4*L4)*L8*T2*(m4*m4)*cos(theta2+theta4)+L4*(L8*L8)*T1*(m4*m4)*cos(theta2)*3.0-L4*(L8*L8)*T2*(m4*m4)*cos(theta2)*3.0-(L4*L4)*L8*T1*(m4*m4)*cos(theta2-theta4)-L4*(L8*L8)*T1*(m4*m4)*cos(theta2-theta4*2.0)+(L4*L4)*L8*T2*(m4*m4)*cos(theta2-theta4)+L4*(L8*L8)*T2*(m4*m4)*cos(theta2-theta4*2.0)-L3*(L4*L4)*T4*m2*m3*2.0-L3*(L4*L4)*T4*m2*m4*2.0-L3*(L4*L4)*T4*m3*m4*2.0-L3*(L8*L8)*T2*m2*m4*4.0-L3*(L8*L8)*T2*m3*m4*4.0-L3*(L8*L8)*T4*m2*m4*4.0-L3*(L8*L8)*T4*m3*m4*4.0+L3*(L4*L4)*T4*(m3*m3)*cos(theta2*2.0)+L3*(L4*L4)*T4*(m4*m4)*cos(theta2*2.0)+L3*(L8*L8)*T2*(m4*m4)*cos(theta2*2.0-theta4*2.0)+L3*(L8*L8)*T4*(m4*m4)*cos(theta2*2.0-theta4*2.0)+(L4*L4)*L8*T1*m3*m4*cos(theta2+theta4)-(L4*L4)*L8*T2*m3*m4*cos(theta2+theta4)-L3*L4*L8*T2*(m4*m4)*cos(theta4)-L3*L4*L8*T4*(m4*m4)*cos(theta4)*2.0-L3*L4*(L8*L8*L8)*(m4*m4*m4)*thetav1*cos(theta2*2.0-theta4)-L3*L4*(L8*L8*L8)*(m4*m4*m4)*thetav2*cos(theta2*2.0-theta4)+L3*L4*(L8*L8*L8)*(m4*m4*m4)*thetav4*cos(theta2*2.0-theta4)+L4*(L8*L8)*T1*m3*m4*cos(theta2)*4.0-L4*(L8*L8)*T2*m3*m4*cos(theta2)*4.0+(L3*L3)*L4*(L8*L8)*(m4*m4*m4)*(thetav1*thetav1)*sin(theta2)*2.0-(L4*L4)*L8*T1*m3*m4*cos(theta2-theta4)+(L4*L4)*L8*T2*m3*m4*cos(theta2-theta4)-L3*(L4*L4)*(L8*L8)*m2*(m4*m4)*thetav1-L3*(L4*L4)*(L8*L8)*m2*(m4*m4)*thetav2-L3*(L4*L4)*(L8*L8)*m3*(m4*m4)*thetav1-L3*(L4*L4)*(L8*L8)*(m3*m3)*m4*thetav1-L3*(L4*L4)*(L8*L8)*m3*(m4*m4)*thetav2-L3*(L4*L4)*(L8*L8)*(m3*m3)*m4*thetav2+L3*(L4*L4)*(L8*L8)*m2*(m4*m4)*thetav4+L3*(L4*L4)*(L8*L8)*m3*(m4*m4)*thetav4+L3*(L4*L4)*(L8*L8)*(m3*m3)*m4*thetav4+L3*L4*(L8*L8*L8)*(m4*m4*m4)*thetav1*cos(theta4)+L3*L4*(L8*L8*L8)*(m4*m4*m4)*thetav2*cos(theta4)-L3*L4*(L8*L8*L8)*(m4*m4*m4)*thetav4*cos(theta4)+L3*(L4*L4)*T4*m3*m4*cos(theta2*2.0)*2.0+L3*L4*L8*T2*(m4*m4)*cos(theta2*2.0-theta4)+L3*L4*L8*T4*(m4*m4)*cos(theta2*2.0-theta4)*2.0+L3*(L4*L4)*(L8*L8)*(m4*m4*m4)*(thetav1*thetav1)*sin(theta2*2.0)+L3*(L4*L4)*(L8*L8)*(m4*m4*m4)*(thetav2*thetav2)*sin(theta2*2.0)+L3*L4*(L8*L8*L8)*m2*(m4*m4)*thetav1*cos(theta4)*2.0+L3*L4*(L8*L8*L8)*m2*(m4*m4)*thetav2*cos(theta4)*2.0+L3*L4*(L8*L8*L8)*m3*(m4*m4)*thetav1*cos(theta4)+L3*L4*(L8*L8*L8)*m3*(m4*m4)*thetav2*cos(theta4)-L3*L4*(L8*L8*L8)*m2*(m4*m4)*thetav4*cos(theta4)*2.0-L3*L4*(L8*L8*L8)*m3*(m4*m4)*thetav4*cos(theta4)-(L3*L3)*(L4*L4)*L8*m2*(m4*m4)*(thetav1*thetav1)*sin(theta2-theta4)-(L3*L3)*L4*(L8*L8)*m2*(m4*m4)*(thetav1*thetav1)*sin(theta2-theta4*2.0)-L3*L4*L8*T2*m2*m4*cos(theta4)*2.0-L3*L4*L8*T2*m3*m4*cos(theta4)-L3*L4*L8*T4*m2*m4*cos(theta4)*4.0-L3*L4*L8*T4*m3*m4*cos(theta4)*2.0+L3*(L4*L4)*(L8*L8)*m3*(m4*m4)*(thetav1*thetav1)*sin(theta2*2.0)*3.0+L3*(L4*L4)*(L8*L8)*(m3*m3)*m4*(thetav1*thetav1)*sin(theta2*2.0)*2.0+L3*(L4*L4)*(L8*L8)*m2*(m4*m4)*(thetav1*thetav1)*sin(theta4*2.0)+L3*(L4*L4)*(L8*L8)*m3*(m4*m4)*(thetav2*thetav2)*sin(theta2*2.0)*3.0+L3*(L4*L4)*(L8*L8)*(m3*m3)*m4*(thetav2*thetav2)*sin(theta2*2.0)*2.0+L3*(L4*L4)*(L8*L8)*m2*(m4*m4)*(thetav2*thetav2)*sin(theta4*2.0)+L3*(L4*L4*L4)*L8*m2*(m4*m4)*(thetav1*thetav1)*sin(theta4)*2.0+L3*(L4*L4*L4)*L8*m2*(m4*m4)*(thetav2*thetav2)*sin(theta4)*2.0-L3*L4*(L8*L8*L8)*m3*(m4*m4)*thetav1*cos(theta2*2.0-theta4)-L3*L4*(L8*L8*L8)*m3*(m4*m4)*thetav2*cos(theta2*2.0-theta4)+L3*L4*(L8*L8*L8)*m3*(m4*m4)*thetav4*cos(theta2*2.0-theta4)-L3*(L4*L4)*(L8*L8)*m2*m3*m4*thetav1*2.0-L3*(L4*L4)*(L8*L8)*m2*m3*m4*thetav2*2.0+L3*(L4*L4)*(L8*L8)*m2*m3*m4*thetav4*2.0+(L3*L3)*(L4*L4)*L8*m2*(m4*m4)*(thetav1*thetav1)*sin(theta2+theta4)+L3*L4*L8*T2*m3*m4*cos(theta2*2.0-theta4)+L3*L4*L8*T4*m3*m4*cos(theta2*2.0-theta4)*2.0+L3*(L4*L4)*(L8*L8)*m3*(m4*m4)*thetav1*cos(theta2*2.0)+L3*(L4*L4)*(L8*L8)*(m3*m3)*m4*thetav1*cos(theta2*2.0)+L3*(L4*L4)*(L8*L8)*m2*(m4*m4)*thetav1*cos(theta4*2.0)+L3*(L4*L4)*(L8*L8)*m3*(m4*m4)*thetav2*cos(theta2*2.0)+L3*(L4*L4)*(L8*L8)*(m3*m3)*m4*thetav2*cos(theta2*2.0)+L3*(L4*L4)*(L8*L8)*m2*(m4*m4)*thetav2*cos(theta4*2.0)-L3*(L4*L4)*(L8*L8)*m3*(m4*m4)*thetav4*cos(theta2*2.0)-L3*(L4*L4)*(L8*L8)*(m3*m3)*m4*thetav4*cos(theta2*2.0)-L3*(L4*L4)*(L8*L8)*m2*(m4*m4)*thetav4*cos(theta4*2.0)+(L3*L3)*L4*(L8*L8)*m2*(m4*m4)*(thetav1*thetav1)*sin(theta2)*3.0+(L3*L3)*L4*(L8*L8)*m3*(m4*m4)*(thetav1*thetav1)*sin(theta2)*6.0+(L3*L3)*L4*(L8*L8)*(m3*m3)*m4*(thetav1*thetav1)*sin(theta2)*4.0+L3*(L4*L4)*(L8*L8)*(m4*m4*m4)*thetav1*thetav2*sin(theta2*2.0)*2.0+(L3*L3)*L4*(L8*L8)*m2*m3*m4*(thetav1*thetav1)*sin(theta2)*4.0-(L3*L3)*(L4*L4)*L8*m2*m3*m4*(thetav1*thetav1)*sin(theta2-theta4)+L3*(L4*L4)*(L8*L8)*m3*(m4*m4)*thetav1*thetav2*sin(theta2*2.0)*6.0+L3*(L4*L4)*(L8*L8)*(m3*m3)*m4*thetav1*thetav2*sin(theta2*2.0)*4.0+L3*(L4*L4)*(L8*L8)*m2*(m4*m4)*thetav1*thetav2*sin(theta4*2.0)*2.0+L3*(L4*L4*L4)*L8*m2*m3*m4*(thetav1*thetav1)*sin(theta4)*2.0+L3*(L4*L4*L4)*L8*m2*m3*m4*(thetav2*thetav2)*sin(theta4)*2.0+L3*(L4*L4*L4)*L8*m2*(m4*m4)*thetav1*thetav2*sin(theta4)*4.0+(L3*L3)*(L4*L4)*L8*m2*m3*m4*(thetav1*thetav1)*sin(theta2+theta4)+L3*(L4*L4*L4)*L8*m2*m3*m4*thetav1*thetav2*sin(theta4)*4.0))/(L3*m4*(m2*m3*4.0+m2*m4*3.0+m3*m4*3.0-(m3*m3)*cos(theta2*2.0)*2.0-(m4*m4)*cos(theta2*2.0)+(m3*m3)*2.0+m4*m4-m3*m4*cos(theta2*2.0)*3.0-m2*m4*cos(theta4*2.0)));
    
    for(int i=0;i<VECTOR_SIZE; i++){
        if(i!=2){
            JointAccelOut[i]=RAD2DEG(JointAccelOut[i]);}
    }
    


    
}

void Joint2Torque(vect &TorqueOut, vect JointPos, vect JointVel, vect JointAccel){
    
    
    double theta1=DEG2RAD(JointPos[0]);
    double theta2=DEG2RAD(JointPos[1]);
    double d3=JointPos[2];
    double theta4=DEG2RAD(JointPos[3]);
    
    double thetav1=DEG2RAD(JointVel[0]);
    double thetav2=DEG2RAD(JointVel[1]);
    double dv3=JointVel[2];
    double thetav4=DEG2RAD(JointVel[4]);
    
    double thetaa1=DEG2RAD(JointAccel[0]);
    double thetaa2=DEG2RAD(JointAccel[1]);
    double da3=JointAccel[2];
    double thetaa4=DEG2RAD(JointAccel[4]);
    
    
    
    TorqueOut[0] = (L3*L3)*m2*thetaa1+(L3*L3)*m3*thetaa1+(L3*L3)*m4*thetaa1+(L4*L4)*m3*thetaa1+(L4*L4)*m3*thetaa2+(L4*L4)*m4*thetaa1+(L4*L4)*m4*thetaa2+(L8*L8)*m4*thetaa1*2.0+(L8*L8)*m4*thetaa2*2.0-(L8*L8)*m4*thetaa4*2.0+(L8*L8)*m4*thetav1+(L8*L8)*m4*thetav2-(L8*L8)*m4*thetav4-L3*L4*m3*(thetav2*thetav2)*sin(theta2)-L3*L4*m4*(thetav2*thetav2)*sin(theta2)-L4*L8*m4*(thetav1*thetav1)*sin(theta4)-L4*L8*m4*(thetav2*thetav2)*sin(theta4)+L3*L8*m4*(thetav1*thetav1)*sin(theta2-theta4)+L3*L4*m3*thetaa1*cos(theta2)*2.0+L3*L4*m3*thetaa2*cos(theta2)+L3*L4*m4*thetaa1*cos(theta2)*2.0+L3*L4*m4*thetaa2*cos(theta2)+L4*L8*m4*thetaa1*cos(theta4)*2.0+L4*L8*m4*thetaa2*cos(theta4)*2.0-L4*L8*m4*thetaa4*cos(theta4)+L4*L8*m4*thetav1*cos(theta4)+L4*L8*m4*thetav2*cos(theta4)-L4*L8*m4*thetav4*cos(theta4)+L3*L8*m4*thetaa1*cos(theta2-theta4)*2.0+L3*L8*m4*thetaa2*cos(theta2-theta4)-L3*L8*m4*thetaa4*cos(theta2-theta4)+L3*L8*m4*thetav1*cos(theta2-theta4)+L3*L8*m4*thetav2*cos(theta2-theta4)-L3*L8*m4*thetav4*cos(theta2-theta4)-L3*L4*m3*thetav1*thetav2*sin(theta2)*2.0-L3*L4*m4*thetav1*thetav2*sin(theta2)*2.0-L4*L8*m4*thetav1*thetav2*sin(theta4)*2.0;
    
    TorqueOut[1] = L4*(L4*m3*thetaa1+L4*m3*thetaa2+L4*m4*thetaa1+L4*m4*thetaa2+L3*m3*thetaa1*cos(theta2)+L3*m4*thetaa1*cos(theta2)+L8*m4*thetaa1*cos(theta4)+L8*m4*thetaa2*cos(theta4)-L8*m4*thetaa4*cos(theta4)+L8*m4*thetav1*cos(theta4)+L8*m4*thetav2*cos(theta4)-L8*m4*thetav4*cos(theta4)+L3*m3*(thetav1*thetav1)*sin(theta2)+L3*m4*(thetav1*thetav1)*sin(theta2))+(L8*L8)*m4*(thetaa1+thetaa2-thetaa4)+L8*m4*(cos(theta4)*(L4*thetaa1+L4*thetaa2+L3*(thetav1*thetav1)*sin(theta2)+L3*thetaa1*cos(theta2))-sin(theta4)*(L4*(thetav1*thetav1)+L4*(thetav2*thetav2)-L3*thetaa1*sin(theta2)+L3*(thetav1*thetav1)*cos(theta2)+L4*thetav1*thetav2*2.0)+L8*(thetaa1+thetaa2-thetaa4)+L8*(thetav1+thetav2-thetav4));
    
    TorqueOut[2] = (m3+m4)*(da3-g);
    
    
    TorqueOut[3] = -L8*m4*(-sin(theta4)*(L4*pow(thetav1+thetav2,2.0)-L3*thetaa1*sin(theta2)+L3*(thetav1*thetav1)*cos(theta2))+L8*(thetaa1+thetaa2-thetaa4)+L8*(thetav1+thetav2-thetav4)+cos(theta4)*(L4*(thetaa1+thetaa2)+L3*(thetav1*thetav1)*sin(theta2)+L3*thetaa1*cos(theta2)))-(L8*L8)*m4*(thetaa1+thetaa2-thetaa4);
    
    
    
    
}




#endif /* DynamicSim_h */

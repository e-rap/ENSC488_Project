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

void Torque2Joint(vect Torque, vect JointPos, vect JointVel, vect& JointPosOut, vect& JointVelOut, vect& JointAccelOut, double DeltaT){
  vect tempT = { 0, 0, 0, 0 };
  VectorCopy(Torque, tempT);

  for (int i = 0; i < VECTOR_SIZE; i++){
    tempT[i] = 1000.0*tempT[i];
  }

  //    double theta1=DEG2RAD(JointPos[0]);
  double theta2 = DEG2RAD(JointPos[1]);
  //    double d3=JointPos[2];
  double theta4 = DEG2RAD(JointPos[3]);

  double thetav1 = DEG2RAD(JointVel[0]);
  double thetav2 = DEG2RAD(JointVel[1]);
  double dv3 = JointVel[2];
  double thetav4 = DEG2RAD(JointVel[3]);


  double T1 = tempT[0];
  double T2 = tempT[1];
  double f3 = tempT[2];
  double T4 = tempT[3];

  JointAccelOut[0] = (1.0/(L3*L3)*(L4*L8*T1*m3*2.0+L4*L8*T1*m4-L4*L8*T2*m3*2.0-L4*L8*T2*m4+FrictionCoef*L4*L8*m3*thetav1*2.0-FrictionCoef*L4*L8*m3*thetav2*2.0+FrictionCoef*L4*L8*m4*thetav1-FrictionCoef*L4*L8*m4*thetav2-L3*L4*T4*m3*cos(theta2+theta4)-L3*L4*T4*m4*cos(theta2+theta4)-L3*L8*T2*m3*cos(theta2)*2.0-L3*L8*T2*m4*cos(theta2)-L3*L8*T4*m3*cos(theta2)*2.0-L3*L8*T4*m4*cos(theta2)+L3*L4*T4*m3*cos(theta2-theta4)+L3*L4*T4*m4*cos(theta2-theta4)+L3*L8*T2*m4*cos(theta2-theta4*2.0)+L3*L8*T4*m4*cos(theta2-theta4*2.0)-L4*L8*T1*m4*cos(theta4*2.0)+L4*L8*T2*m4*cos(theta4*2.0)+FrictionCoef*L3*L4*m3*thetav4*cos(theta2-theta4)+FrictionCoef*L3*L4*m4*thetav4*cos(theta2-theta4)+FrictionCoef*L3*L8*m4*thetav2*cos(theta2-theta4*2.0)+FrictionCoef*L3*L8*m4*thetav4*cos(theta2-theta4*2.0)+L3*(L4*L4)*L8*(m3*m3)*(thetav1*thetav1)*sin(theta2)*2.0+L3*(L4*L4)*L8*(m3*m3)*(thetav2*thetav2)*sin(theta2)*2.0-FrictionCoef*L4*L8*m4*thetav1*cos(theta4*2.0)+FrictionCoef*L4*L8*m4*thetav2*cos(theta4*2.0)+(L3*L3)*L4*L8*(m3*m3)*(thetav1*thetav1)*sin(theta2*2.0)-FrictionCoef*L3*L4*m3*thetav4*cos(theta2+theta4)-FrictionCoef*L3*L4*m4*thetav4*cos(theta2+theta4)-FrictionCoef*L3*L8*m3*thetav2*cos(theta2)*2.0-FrictionCoef*L3*L8*m4*thetav2*cos(theta2)-FrictionCoef*L3*L8*m3*thetav4*cos(theta2)*2.0-FrictionCoef*L3*L8*m4*thetav4*cos(theta2)+L3*(L4*L4)*L8*m3*m4*(thetav1*thetav1)*sin(theta2)*2.0+L3*(L4*L4)*L8*m3*m4*(thetav2*thetav2)*sin(theta2)*2.0+L3*(L4*L4)*L8*(m3*m3)*thetav1*thetav2*sin(theta2)*4.0+(L3*L3)*L4*L8*m3*m4*(thetav1*thetav1)*sin(theta2*2.0)+L3*(L4*L4)*L8*m3*m4*thetav1*thetav2*sin(theta2)*4.0))/(L4*L8*(m2*m3*2.0+m2*m4+m3*m4-(m3*m3)*cos(theta2*2.0)+m3*m3-m3*m4*cos(theta2*2.0)-m2*m4*cos(theta4*2.0)));
    
  JointAccelOut[1] = -(1.0/(L3*L3)*1.0/(L4*L4)*((L3*L3)*L8*T2*m2*-2.0-(L3*L3)*L8*T2*m3*2.0+(L4*L4)*L8*T1*m3*2.0-(L3*L3)*L8*T2*m4-(L3*L3)*L8*T4*m2*2.0+(L4*L4)*L8*T1*m4-(L4*L4)*L8*T2*m3*2.0-(L3*L3)*L8*T4*m3*2.0-(L4*L4)*L8*T2*m4-(L3*L3)*L8*T4*m4+L3*(L4*L4)*T4*m3*cos(theta2-theta4)+L3*(L4*L4)*T4*m4*cos(theta2-theta4)-(L4*L4)*L8*T1*m4*cos(theta4*2.0)+(L4*L4)*L8*T2*m4*cos(theta4*2.0)-FrictionCoef*(L3*L3)*L8*m2*thetav2*2.0-FrictionCoef*(L3*L3)*L8*m3*thetav2*2.0+FrictionCoef*(L4*L4)*L8*m3*thetav1*2.0-FrictionCoef*(L3*L3)*L8*m2*thetav4*2.0-FrictionCoef*(L3*L3)*L8*m4*thetav2-FrictionCoef*(L4*L4)*L8*m3*thetav2*2.0+FrictionCoef*(L4*L4)*L8*m4*thetav1-FrictionCoef*(L3*L3)*L8*m3*thetav4*2.0-FrictionCoef*(L4*L4)*L8*m4*thetav2-FrictionCoef*(L3*L3)*L8*m4*thetav4+(L3*L3)*L4*T4*m3*cos(theta2*2.0-theta4)+(L3*L3)*L4*T4*m4*cos(theta2*2.0-theta4)+(L3*L3)*L8*T2*m4*cos(theta2*2.0-theta4*2.0)+(L3*L3)*L8*T4*m4*cos(theta2*2.0-theta4*2.0)-L3*(L4*L4)*T4*m3*cos(theta2+theta4)-L3*(L4*L4)*T4*m4*cos(theta2+theta4)-(L3*L3)*L4*T4*m2*cos(theta4)*2.0-(L3*L3)*L4*T4*m3*cos(theta4)-(L3*L3)*L4*T4*m4*cos(theta4)-L3*L4*L8*T1*m4*cos(theta2-theta4*2.0)+L3*L4*L8*T2*m4*cos(theta2-theta4*2.0)*2.0+L3*L4*L8*T4*m4*cos(theta2-theta4*2.0)-FrictionCoef*L3*(L4*L4)*m3*thetav4*cos(theta2+theta4)-FrictionCoef*L3*(L4*L4)*m4*thetav4*cos(theta2+theta4)+L3*(L4*L4*L4)*L8*(m3*m3)*(thetav1*thetav1)*sin(theta2)*2.0+(L3*L3*L3)*L4*L8*(m3*m3)*(thetav1*thetav1)*sin(theta2)*2.0+L3*(L4*L4*L4)*L8*(m3*m3)*(thetav2*thetav2)*sin(theta2)*2.0-FrictionCoef*(L3*L3)*L4*m2*thetav4*cos(theta4)*2.0-FrictionCoef*(L3*L3)*L4*m3*thetav4*cos(theta4)-FrictionCoef*(L3*L3)*L4*m4*thetav4*cos(theta4)+FrictionCoef*L3*(L4*L4)*m3*thetav4*cos(theta2-theta4)+FrictionCoef*L3*(L4*L4)*m4*thetav4*cos(theta2-theta4)-FrictionCoef*(L4*L4)*L8*m4*thetav1*cos(theta4*2.0)+FrictionCoef*(L4*L4)*L8*m4*thetav2*cos(theta4*2.0)+L3*L4*L8*T1*m3*cos(theta2)*2.0+L3*L4*L8*T1*m4*cos(theta2)-L3*L4*L8*T2*m3*cos(theta2)*4.0-L3*L4*L8*T2*m4*cos(theta2)*2.0-L3*L4*L8*T4*m3*cos(theta2)*2.0-L3*L4*L8*T4*m4*cos(theta2)+FrictionCoef*(L3*L3)*L4*m3*thetav4*cos(theta2*2.0-theta4)+FrictionCoef*(L3*L3)*L4*m4*thetav4*cos(theta2*2.0-theta4)+FrictionCoef*(L3*L3)*L8*m4*thetav2*cos(theta2*2.0-theta4*2.0)+FrictionCoef*(L3*L3)*L8*m4*thetav4*cos(theta2*2.0-theta4*2.0)+(L3*L3)*(L4*L4)*L8*(m3*m3)*(thetav1*thetav1)*sin(theta2*2.0)*2.0+(L3*L3)*(L4*L4)*L8*(m3*m3)*(thetav2*thetav2)*sin(theta2*2.0)+(L3*L3*L3)*L4*L8*m2*m3*(thetav1*thetav1)*sin(theta2)*2.0+(L3*L3*L3)*L4*L8*m2*m4*(thetav1*thetav1)*sin(theta2)+L3*(L4*L4*L4)*L8*m3*m4*(thetav1*thetav1)*sin(theta2)*2.0+(L3*L3*L3)*L4*L8*m3*m4*(thetav1*thetav1)*sin(theta2)*2.0+L3*(L4*L4*L4)*L8*m3*m4*(thetav2*thetav2)*sin(theta2)*2.0+L3*(L4*L4*L4)*L8*(m3*m3)*thetav1*thetav2*sin(theta2)*4.0+FrictionCoef*L3*L4*L8*m3*thetav1*cos(theta2)*2.0-FrictionCoef*L3*L4*L8*m3*thetav2*cos(theta2)*4.0+FrictionCoef*L3*L4*L8*m4*thetav1*cos(theta2)-FrictionCoef*L3*L4*L8*m4*thetav2*cos(theta2)*2.0-FrictionCoef*L3*L4*L8*m3*thetav4*cos(theta2)*2.0-FrictionCoef*L3*L4*L8*m4*thetav4*cos(theta2)-(L3*L3*L3)*L4*L8*m2*m4*(thetav1*thetav1)*sin(theta2-theta4*2.0)-FrictionCoef*L3*L4*L8*m4*thetav1*cos(theta2-theta4*2.0)+FrictionCoef*L3*L4*L8*m4*thetav2*cos(theta2-theta4*2.0)*2.0+FrictionCoef*L3*L4*L8*m4*thetav4*cos(theta2-theta4*2.0)+(L3*L3)*(L4*L4)*L8*m3*m4*(thetav1*thetav1)*sin(theta2*2.0)*2.0+(L3*L3)*(L4*L4)*L8*m2*m4*(thetav1*thetav1)*sin(theta4*2.0)+(L3*L3)*(L4*L4)*L8*m3*m4*(thetav2*thetav2)*sin(theta2*2.0)+(L3*L3)*(L4*L4)*L8*m2*m4*(thetav2*thetav2)*sin(theta4*2.0)+(L3*L3)*(L4*L4)*L8*(m3*m3)*thetav1*thetav2*sin(theta2*2.0)*2.0+(L3*L3)*(L4*L4)*L8*m3*m4*thetav1*thetav2*sin(theta2*2.0)*2.0+(L3*L3)*(L4*L4)*L8*m2*m4*thetav1*thetav2*sin(theta4*2.0)*2.0+L3*(L4*L4*L4)*L8*m3*m4*thetav1*thetav2*sin(theta2)*4.0))/(L8*(m2*m3*2.0+m2*m4+m3*m4-(m3*m3)*cos(theta2*2.0)+m3*m3-m3*m4*cos(theta2*2.0)-m2*m4*cos(theta4*2.0)));
    
  JointAccelOut[2] = (f3+FrictionCoef*dv3+g*(m3+m4))/(m3+m4);


  JointAccelOut[3] = (1.0/(L4*L4)*(T2+FrictionCoef*thetav2-(L8*L8)*m4*thetav1-(L8*L8)*m4*thetav2+(L8*L8)*m4*thetav4-L3*L4*m3*(thetav1*thetav1)*sin(theta2)-L3*L4*m4*(thetav1*thetav1)*sin(theta2)+L4*L8*m4*(thetav1*thetav1)*sin(theta4)+L4*L8*m4*(thetav2*thetav2)*sin(theta4)-L3*L8*m4*(thetav1*thetav1)*sin(theta2-theta4)-L4*L8*m4*thetav1*cos(theta4)-L4*L8*m4*thetav2*cos(theta4)+L4*L8*m4*thetav4*cos(theta4)+L4*L8*m4*thetav1*thetav2*sin(theta4)*2.0)*(L3*L8*m2+L3*L8*m3+L3*L8*m4-(L4*L4)*m3*cos(theta2-theta4)-(L4*L4)*m4*cos(theta2-theta4)+(L4*L4)*m3*cos(theta2)*cos(theta4)+(L4*L4)*m4*cos(theta2)*cos(theta4)+L3*L4*m2*cos(theta4)+L3*L4*m3*cos(theta4)+L3*L4*m4*cos(theta4)+L4*L8*m3*cos(theta2)+L4*L8*m4*cos(theta2)-L3*L8*m4*pow(cos(theta2-theta4),2.0)-L3*L4*m3*cos(theta2-theta4)*cos(theta2)-L3*L4*m4*cos(theta2-theta4)*cos(theta2)-L4*L8*m4*cos(theta2-theta4)*cos(theta4)))/(L3*L8*(m2*m3+(m3*m3)*pow(sin(theta2),2.0)+m3*m4*pow(sin(theta2),2.0)+m2*m4*pow(sin(theta4),2.0)))+(1.0/(L4*L4)*1.0/(L8*L8)*(T4+FrictionCoef*thetav4+(L8*L8)*m4*thetav1+(L8*L8)*m4*thetav2-(L8*L8)*m4*thetav4-L4*L8*m4*(thetav1*thetav1)*sin(theta4)-L4*L8*m4*(thetav2*thetav2)*sin(theta4)+L3*L8*m4*(thetav1*thetav1)*sin(theta2-theta4)-L4*L8*m4*thetav1*thetav2*sin(theta4)*2.0)*((L4*L4)*(m3*m3)+(L4*L4)*(m4*m4)+(L8*L8)*(m4*m4)-(L4*L4)*(m3*m3)*pow(cos(theta2),2.0)-(L4*L4)*(m4*m4)*pow(cos(theta2),2.0)-(L8*L8)*(m4*m4)*pow(cos(theta2-theta4),2.0)+(L4*L4)*m2*m3+(L4*L4)*m2*m4+(L4*L4)*m3*m4*2.0+(L8*L8)*m2*m4+(L8*L8)*m3*m4-(L4*L4)*m3*m4*pow(cos(theta2),2.0)*2.0+L4*L8*(m4*m4)*cos(theta4)*2.0-L4*L8*(m4*m4)*cos(theta2-theta4)*cos(theta2)*2.0+L4*L8*m2*m4*cos(theta4)*2.0+L4*L8*m3*m4*cos(theta4)*2.0-L4*L8*m3*m4*cos(theta2-theta4)*cos(theta2)*2.0))/(m4*(m2*m3+(m3*m3)*pow(sin(theta2),2.0)+m3*m4*pow(sin(theta2),2.0)+m2*m4*pow(sin(theta4),2.0)))+((-L8*m3*cos(theta2)-L8*m4*cos(theta2)+L8*m4*cos(theta2)*pow(cos(theta4),2.0)+L4*m3*sin(theta2)*sin(theta4)+L4*m4*sin(theta2)*sin(theta4)+L8*m4*cos(theta4)*sin(theta2)*sin(theta4))*(T1+FrictionCoef*thetav1-(L8*L8)*m4*thetav1-(L8*L8)*m4*thetav2+(L8*L8)*m4*thetav4+L3*L4*m3*(thetav2*thetav2)*sin(theta2)+L3*L4*m4*(thetav2*thetav2)*sin(theta2)+L4*L8*m4*(thetav1*thetav1)*sin(theta4)+L4*L8*m4*(thetav2*thetav2)*sin(theta4)-L3*L8*m4*(thetav1*thetav1)*sin(theta2-theta4)-L4*L8*m4*thetav1*cos(theta4)-L4*L8*m4*thetav2*cos(theta4)+L4*L8*m4*thetav4*cos(theta4)-L3*L8*m4*thetav1*cos(theta2-theta4)-L3*L8*m4*thetav2*cos(theta2-theta4)+L3*L8*m4*thetav4*cos(theta2-theta4)+L3*L4*m3*thetav1*thetav2*sin(theta2)*2.0+L3*L4*m4*thetav1*thetav2*sin(theta2)*2.0+L4*L8*m4*thetav1*thetav2*sin(theta4)*2.0))/(L3*L4*L8*(m2*m3+(m3*m3)*pow(sin(theta2),2.0)+m3*m4*pow(sin(theta2),2.0)+m2*m4*pow(sin(theta4),2.0)));


  for (int i = 0; i < VECTOR_SIZE; i++){
    if (i != 2){
      JointAccelOut[i] = RAD2DEG(JointAccelOut[i]);
    }
  }


  for (int i = 0; i < VECTOR_SIZE; i++){
    JointVelOut[i] = JointVel[i] + JointAccelOut[i] * DeltaT;
    JointPosOut[i] = JointPos[i] + JointVel[i] * DeltaT + 0.5*JointAccelOut[i] * pow(DeltaT, 2);
  }
    
    
    for (int i = 0; i < VECTOR_SIZE; i++){
      tempT[i] = 0.001*tempT[i];
    }
    

}

void Joint2Torque(vect &TorqueOut, vect JointPos, vect JointVel, vect JointAccel){


  //    double theta1=DEG2RAD(JointPos[0]);
  double theta2 = DEG2RAD(JointPos[1]);
  //    double d3=JointPos[2];
  double theta4 = DEG2RAD(JointPos[3]);

  double thetav1 = DEG2RAD(JointVel[0]);
  double thetav2 = DEG2RAD(JointVel[1]);
  double dv3 = JointVel[2];
  double thetav4 = DEG2RAD(JointVel[3]);

  double thetaa1 = DEG2RAD(JointAccel[0]);
  double thetaa2 = DEG2RAD(JointAccel[1]);
  double da3 = JointAccel[2];
  double thetaa4 = DEG2RAD(JointAccel[3]);



  TorqueOut[0] = -FrictionCoef*thetav1+(L3*L3)*m2*thetaa1+(L3*L3)*m3*thetaa1+(L3*L3)*m4*thetaa1+(L4*L4)*m3*thetaa1+(L4*L4)*m3*thetaa2+(L4*L4)*m4*thetaa1+(L4*L4)*m4*thetaa2+(L8*L8)*m4*thetaa1+(L8*L8)*m4*thetaa2-(L8*L8)*m4*thetaa4+(L8*L8)*m4*thetav1+(L8*L8)*m4*thetav2-(L8*L8)*m4*thetav4-L3*L4*m3*(thetav2*thetav2)*sin(theta2)-L3*L4*m4*(thetav2*thetav2)*sin(theta2)-L4*L8*m4*(thetav1*thetav1)*sin(theta4)-L4*L8*m4*(thetav2*thetav2)*sin(theta4)+L3*L8*m4*(thetav1*thetav1)*sin(theta2-theta4)+L3*L4*m3*thetaa1*cos(theta2)*2.0+L3*L4*m3*thetaa2*cos(theta2)+L3*L4*m4*thetaa1*cos(theta2)*2.0+L3*L4*m4*thetaa2*cos(theta2)+L4*L8*m4*thetaa1*cos(theta4)*2.0+L4*L8*m4*thetaa2*cos(theta4)*2.0-L4*L8*m4*thetaa4*cos(theta4)+L4*L8*m4*thetav1*cos(theta4)+L4*L8*m4*thetav2*cos(theta4)-L4*L8*m4*thetav4*cos(theta4)+L3*L8*m4*thetaa1*cos(theta2-theta4)*2.0+L3*L8*m4*thetaa2*cos(theta2-theta4)-L3*L8*m4*thetaa4*cos(theta2-theta4)+L3*L8*m4*thetav1*cos(theta2-theta4)+L3*L8*m4*thetav2*cos(theta2-theta4)-L3*L8*m4*thetav4*cos(theta2-theta4)-L3*L4*m3*thetav1*thetav2*sin(theta2)*2.0-L3*L4*m4*thetav1*thetav2*sin(theta2)*2.0-L4*L8*m4*thetav1*thetav2*sin(theta4)*2.0;

  TorqueOut[1] = L4*(L4*m3*thetaa1+L4*m3*thetaa2+L4*m4*thetaa1+L4*m4*thetaa2+L3*m3*thetaa1*cos(theta2)+L3*m4*thetaa1*cos(theta2)+L8*m4*thetaa1*cos(theta4)+L8*m4*thetaa2*cos(theta4)-L8*m4*thetaa4*cos(theta4)+L8*m4*thetav1*cos(theta4)+L8*m4*thetav2*cos(theta4)-L8*m4*thetav4*cos(theta4)+L3*m3*(thetav1*thetav1)*sin(theta2)+L3*m4*(thetav1*thetav1)*sin(theta2))-FrictionCoef*thetav2+L8*m4*(cos(theta4)*(L4*thetaa1+L4*thetaa2+L3*(thetav1*thetav1)*sin(theta2)+L3*thetaa1*cos(theta2))-sin(theta4)*(L4*(thetav1*thetav1)+L4*(thetav2*thetav2)-L3*thetaa1*sin(theta2)+L3*(thetav1*thetav1)*cos(theta2)+L4*thetav1*thetav2*2.0)+L8*(thetaa1+thetaa2-thetaa4)+L8*(thetav1+thetav2-thetav4));

  TorqueOut[2] = -FrictionCoef*dv3+m3*(da3-g)+m4*(da3-g);


  TorqueOut[3] = -FrictionCoef*thetav4-L8*m4*(-sin(theta4)*(L4*pow(thetav1+thetav2,2.0)-L3*thetaa1*sin(theta2)+L3*(thetav1*thetav1)*cos(theta2))+L8*(thetaa1+thetaa2-thetaa4)+L8*(thetav1+thetav2-thetav4)+cos(theta4)*(L4*(thetaa1+thetaa2)+L3*(thetav1*thetav1)*sin(theta2)+L3*thetaa1*cos(theta2)));
    
  //convert to Nm
  for (int i = 0; i < VECTOR_SIZE; i++){
    TorqueOut[i] = 0.001*TorqueOut[i];
  }
}

void DynamicSim(vect& TorqueIn, vect JointPosIn, vect JointVelIn, 
                vect& JointPosOut, vect& JointVelOut, vect& JointAccelOut, 
                vect* JointPosArray, vect* JointVelArray, vect* JointAccelArray,
                double SimDuration, double SampleRate)
{

  vect temp = { 0, 0, 0, 0 };
  VectorCopy(JointPosIn,temp );

  DisplayConfiguration(temp);

  // calc number of samples
  int num_samples = SimDuration * SampleRate;

  double sample_time = (1.0 / SampleRate);

  // Run until done sim time
  //std::cout << "GET READY!" << std::endl;
  //std::cout << "3" << std::endl;
  //microsleep(1*S_TO_MILIS);
  //std::cout << "2" << std::endl;
  //microsleep(1 * S_TO_MILIS);
  //std::cout << "1" << std::endl;
  //microsleep(1 * S_TO_MILIS);
  //std::cout << "GO!" << std::endl;

  for (int i = 0; i < num_samples; i++)
  {


    //calculate JointAccel from Torque for given JointPos, JointVel

    Torque2Joint(TorqueIn, JointPosIn, JointVelIn, JointPosOut, JointVelOut, JointAccelOut, sample_time);

    // Update Display
    DisplayConfiguration(JointPosOut);

    //Save calculated JointPos and JointVel in Array
    for (int j = 0; j < VECTOR_SIZE; j++)
    {
      JointPosArray[i][j] = JointPosOut[j];
      JointVelArray[i][j] = JointVelOut[j];
      JointAccelArray[i][j] = JointVelOut[j];
    }

    //update JointPos and JointVel
    for (int i = 0; i < VECTOR_SIZE; i++)
    {
      JointPosIn[i] = JointPosOut[i];
      JointVelIn[i] = JointVelOut[i];
    }

    microsleep(sample_time* S_TO_MILIS);
  }
}


   

//////////// TESTING CODE ///////////////////////////
   

//void DynamicSim(vect TorqueIn, vect JointPosIn, vect JointVelIn, vect& JointPosOut, vect& JointVelOut, vect& JointAccelOut, vect* JointPosArray, vect* JointVelArray, vect* JointAccelArray){
//
//  int max = T2 / T3; //number of integrations during one T2 cycle
//
//
//  //convert to Nmm
//  for (int i = 0; i < VECTOR_SIZE; i++){
//    TorqueIn[i] = 1000.0*TorqueIn[i];
//  }
//
//  for (int i = 0; i < max; i++){
//
//    //calculate JointAccel from Torque for given JointPos, JointVel
//    Torque2Joint(TorqueIn, JointPosIn, JointVelIn, JointPosOut, JointVelOut, JointAccelOut, T3);
//
//    //Save caluclated JointPos and JointVel in Array
//    for (int j = 0; j < VECTOR_SIZE; j++){
//      JointPosArray[i][j] = JointPosOut[j];
//      JointVelArray[i][j] = JointVelOut[j];
//    }
//    //update JointPos and JointVel
//    for (int i = 0; i < VECTOR_SIZE; i++){
//      JointPosIn[i] = JointPosOut[i];
//      JointVelIn[i] = JointVelOut[i];
//
//    }
//  }
//

    //write JointPosArray and JointVelArray to files

/* std::ofstream file(filenameDynSimPos);
 if (file.is_open())
 {
 for (int i = 0; i < max; i++)
 {
 for (int j = 0; j < VECTOR_SIZE; j++)
 {
 file << JointPosArray[i][j] << " ";
 }
 file << std::endl;
 }
 file.close();
 }

 std::ofstream file2(filenameDynSimVel);
 if (file2.is_open())
 {
 for (int i = 0; i < max; i++)
 {
 for (int j = 0; j < VECTOR_SIZE; j++)
 {
 file2 << JointVelArray[i][j] << " ";
 }
 file2 << std::endl;

 }
 file2.close();
 }
 }*/



#endif /* DynamicSim_h */

#ifndef Controller_h__
#define Controller_h__

#include "matrix.h"
#include "RobotGlobals.h"
#include <exception>
#include <cmath>
#include "InverseKin.h"
#include <fstream>

void MCalc(vect JointPos, vect JointAccel, vect& Mc)
{
  double theta1=DEG2RAD(JointPos[0]);
  double theta2 = DEG2RAD(JointPos[1]);
  double d3=JointPos[2];
  double theta4 = DEG2RAD(JointPos[3]);

  double thetaa1 = DEG2RAD(JointAccel[0]);
  double thetaa2 = DEG2RAD(JointAccel[1]);
  double da3 = JointAccel[2];
  double thetaa4 = DEG2RAD(JointAccel[3]);
 


  Mc[0] = thetaa1*((L3*L3)*m2 + (L3*L3)*m3 + (L3*L3)*m4 + (L4*L4)*m3 + (L4*L4)*m4 + (L8*L8)*m4 + L3*L4*m3*cos(theta2)*2.0 + L3*L4*m4*cos(theta2)*2.0 + L4*L8*m4*cos(theta4)*2.0 + L3*L8*m4*cos(theta2 - theta4)*2.0) + thetaa2*((L4*L4)*m3 + (L4*L4)*m4 + (L8*L8)*m4 + L3*L4*m3*cos(theta2) + L3*L4*m4*cos(theta2) + L4*L8*m4*cos(theta4)*2.0 + L3*L8*m4*cos(theta2 - theta4)) - L8*m4*thetaa4*(L8 + L4*cos(theta4) + L3*cos(theta2 - theta4));
  Mc[1] = thetaa1*(L4*(L4*m3 + L4*m4 + L3*m3*cos(theta2) + L3*m4*cos(theta2) + L8*m4*cos(theta4)) + L8*m4*(L8 + L4*cos(theta4) + L3*cos(theta2 - theta4))) + thetaa2*((L4*L4)*m3 + (L4*L4)*m4 + (L8*L8)*m4 + L4*L8*m4*cos(theta4)*2.0) - L8*m4*thetaa4*(L8 + L4*cos(theta4));
  Mc[2] = da3*(m3 + m4);
  Mc[3] = (L8*L8)*m4*thetaa4 - L8*m4*thetaa1*(L8 + L4*cos(theta4) + L3*cos(theta2 - theta4)) - L8*m4*thetaa2*(L8 + L4*cos(theta4));
}
void VCalc(vect JointPos, vect JointVel, vect &V)
{
  double theta1 = DEG2RAD(JointPos[0]);
  double theta2 = DEG2RAD(JointPos[1]);
  double d3 = JointPos[2];
  double theta4 = DEG2RAD(JointPos[3]);

  double thetav1 = DEG2RAD(JointVel[0]);
  double thetav2 = DEG2RAD(JointVel[1]);
  double dv3 = JointVel[2];
  double thetav4 = DEG2RAD(JointVel[3]);
  
  V[0] = (L8*L8)*m4*thetav1 + (L8*L8)*m4*thetav2 - (L8*L8)*m4*thetav4 - L3*L4*m3*(thetav2*thetav2)*sin(theta2) - L3*L4*m4*(thetav2*thetav2)*sin(theta2) - L4*L8*m4*(thetav1*thetav1)*sin(theta4) - L4*L8*m4*(thetav2*thetav2)*sin(theta4) + L3*L8*m4*(thetav1*thetav1)*sin(theta2 - theta4) + L4*L8*m4*thetav1*cos(theta4) + L4*L8*m4*thetav2*cos(theta4) - L4*L8*m4*thetav4*cos(theta4) + L3*L8*m4*thetav1*cos(theta2 - theta4) + L3*L8*m4*thetav2*cos(theta2 - theta4) - L3*L8*m4*thetav4*cos(theta2 - theta4) - L3*L4*m3*thetav1*thetav2*sin(theta2)*2.0 - L3*L4*m4*thetav1*thetav2*sin(theta2)*2.0 - L4*L8*m4*thetav1*thetav2*sin(theta4)*2.0;
  V[2] = 0.0;
  V[1] = (L8*L8)*m4*thetav1 + (L8*L8)*m4*thetav2 - (L8*L8)*m4*thetav4 + L3*L4*m3*(thetav1*thetav1)*sin(theta2) + L3*L4*m4*(thetav1*thetav1)*sin(theta2) - L4*L8*m4*(thetav1*thetav1)*sin(theta4) - L4*L8*m4*(thetav2*thetav2)*sin(theta4) + L3*L8*m4*(thetav1*thetav1)*sin(theta2 - theta4) + L4*L8*m4*thetav1*cos(theta4) + L4*L8*m4*thetav2*cos(theta4) - L4*L8*m4*thetav4*cos(theta4) - L4*L8*m4*thetav1*thetav2*sin(theta4)*2.0;
  V[3] = L8*m4*(-L8*thetav1 - L8*thetav2 + L8*thetav4 + L4*(thetav1*thetav1)*sin(theta4) + L4*(thetav2*thetav2)*sin(theta4) - L3*(thetav1*thetav1)*sin(theta2 - theta4) + L4*thetav1*thetav2*sin(theta4)*2.0);
}
void GCalc(vect& G)
{
  G[0] = 0;
  G[1] = 0;
  G[2] = -g*(m3 + m4);
  G[3] = 0;
}

void FCalc(vect JointVel, vect &Friction)
{
  double thetav1 = DEG2RAD(JointVel[0]);
  double thetav2 = DEG2RAD(JointVel[1]);
  double dv3 = JointVel[2];
  double thetav4 = DEG2RAD(JointVel[3]);

  Friction[0] = -FrictionCoef*thetav1;
  Friction[1] = -FrictionCoef*thetav2;
  Friction[2] = -FrictionCoef*dv3;
  Friction[3] = -FrictionCoef*thetav4;
}

void FeebackControl(vect DesPos, vect DesVel, vect DesAccel, vect FBPos, vect FBVel, vect& TorqueOut)
{
  vect EPos = { 0, 0, 0, 0 };
  vect EVel = { 0, 0, 0, 0};
  vect EAccel = { 0, 0, 0, 0 };

  vect EPosGain = { 0, 0, 0, 0 };
  vect EVelGain = { 0, 0, 0, 0 }; 
  vect ESum = { 0, 0, 0, 0 };

  vect Mc = { 0, 0, 0, 0 };
  vect V = { 0, 0, 0, 0 };
  vect G = { 0, 0, 0, 0 };
  vect F = { 0, 0, 0, 0 };

  VectorSub(DesPos, FBPos, EPos); // E = DPos- FBPos
  VectorSub(DesVel, FBVel, EVel); // EDot = DVel - FBVel
  VectorMultElement(EPos, kp, EPosGain); // EPosGain = E * Kp
  VectorMultElement(EVel, kv, EVelGain); // EVelGain = Edot * Kv
  VectorAdd(EPosGain, EVelGain, ESum); // ESum = EPosGain + EVelGain
  VectorAdd(DesAccel, ESum, EAccel); // EAccel = DesAccel + ESum

  MCalc(FBPos, EAccel, Mc);
  VCalc(FBPos, FBVel, V);
  GCalc(G);
  FCalc(FBVel, F);

  for (int i = 0; i < VECTOR_SIZE; i++)
  {
    TorqueOut[i] = (Mc[i] + V[i] + G[i] + F[i])/ 1000.0;
  }

}
#endif // Controller_h__
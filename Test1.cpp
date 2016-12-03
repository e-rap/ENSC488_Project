#include "stateid.h"
#include "ensc-488.h"
#include "WHERE.h"
#include "InverseKin.h"
#include "PickAndPlace.h"
#include "TrajectoryPlanning.h"
#include "RobotGlobals.h"
#include "DynamicSim.h"
#include "Controller.h"
#include <fstream>

using namespace std;

#define SAMPLE_RATE_X 60.0
#define SIM_DUR 10.0

#define NUM_DYNAMIC_SAMPLES (int)(SAMPLE_RATE_X*SIM_DUR)

// Globals //
vect gCurrentConfig; // Current_Robot Configuration
bool gGrasp = false;

//////////////////////
// Helper Functions //
//////////////////////

void RoboSim()
{
  std::ofstream simP = OpenFile("simP.txt");
  std::ofstream simV = OpenFile("simV.txt");
  std::ofstream simA = OpenFile("simA.txt");
  std::ofstream plannedP = OpenFile("plannedP.txt");
  std::ofstream plannedV = OpenFile("plannedV.txt");
  std::ofstream plannedA = OpenFile("plannedA.txt");
  if (!simP.is_open())
  {
    cout << "HELP!\n";
  }


  int num_via;

  cout << "Input Number of Via points in file";
  cin >> num_via;
  cout << endl;

  matrix param1;
  matrix param2;
  matrix param3;
  matrix param4;
  MatrixInit(param1);
  MatrixInit(param2);
  MatrixInit(param3);
  MatrixInit(param4);


  double times[5] = { 0, 0, 0, 0, 0 };
  double viax[5] = { 0, 0, 0, 0, 0 };
  double viay[5] = { 0, 0, 0, 0, 0 };
  double viaz[5] = { 0, 0, 0, 0, 0 };
  double viaphi[5] = { 0, 0, 0, 0, 0 };

  ReadViaPoints(times, viax, viay, viaz, viaphi, num_via);
  vect jointPosVector[5] = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };

  for (int i = 0; i < num_via; i++)
  {
    vect tempConfig = { 0, 0, 0, 0 };
    vect closestSol = { 0, 0, 0, 0 };
    bool sol = false;

    matrix tempConfigMat;
    MatrixInit(tempConfigMat);

    tempConfig[0] = viax[i];
    tempConfig[1] = viay[i];
    tempConfig[2] = viaz[i];
    tempConfig[3] = viaphi[i];
    U2I(tempConfig, tempConfigMat);
    vect zeroVect = { 0, 0, -200, 0 };
    if (i == 0)
    {
      SOLVE(tempConfigMat, zeroVect, closestSol, zeroVect, sol);
      if (!sol)
      {
        std::cout << "TRAJPLAN: No solutions found for input config" << std::endl;
      }
      VectorCopy(closestSol, jointPosVector[i]);
      continue;
    }

    SOLVE(tempConfigMat, jointPosVector[i - 1], closestSol, zeroVect, sol);
    if (!sol)
    {
      std::cout << "TRAJPLAN: No solutions found for input config" << std::endl;
    }
    VectorCopy(closestSol, jointPosVector[i]);
  }

  double via1[5] = { 0, 0, 0, 0, 0 };
  double via2[5] = { 0, 0, 0, 0, 0 };
  double via3[5] = { 0, 0, 0, 0, 0 };
  double via4[5] = { 0, 0, 0, 0, 0 };

  for (int i = 0; i < num_via; i++)
  {
    via1[i] = jointPosVector[i][0];
    via2[i] = jointPosVector[i][1];
    via3[i] = jointPosVector[i][2];
    via4[i] = jointPosVector[i][3];
  }

  vect JointPosArray[MAX_DATA_POINTS];
  vect JointVelArray[MAX_DATA_POINTS];
  vect JointAccelArray[MAX_DATA_POINTS];

  int num_samples = 0;

  // Init Vectors
  for (int i = 0; i < MAX_DATA_POINTS; i++)
  {
    VectorInit(JointPosArray[i]);
    VectorInit(JointVelArray[i]);
    VectorInit(JointAccelArray[i]);
  }

  TraGen(times, via1, via2, via3, via4, param1, param2, param3, param4, 5);
  TraCalc(times, param1, param2, param3, param4, num_via, SAMPLING_RATE_T1, JointPosArray, JointVelArray, JointAccelArray, num_samples);

  // Display Init
  DisplayConfiguration(JointPosArray[0]);

  vect DPos;
  vect DVel;
  vect DAccel;
  vect FBPos;
  vect FBVel;
  vect Torque;
  vect TempFBPos;
  vect TempFBVel;
  vect AccelOut;

  VectorInit(DPos);
  VectorInit(DVel);
  VectorInit(DAccel);
  VectorInit(FBPos);
  VectorInit(FBVel);
  VectorInit(Torque);
  VectorInit(TempFBPos);
  VectorInit(TempFBVel);
  VectorInit(AccelOut);

  VectorCopy(JointPosArray[0], FBPos);
  VectorCopy(JointVelArray[0], FBVel);

  for (int counter = 0; counter < num_samples * 100; counter++)
  {
    if ((counter % 100) == 0)
    {
      VectorCopy(JointPosArray[counter / 100], DPos);
      VectorCopy(JointVelArray[counter / 100],DVel);
      VectorCopy(JointAccelArray[counter / 100], DAccel);



    }
    if ((counter % 10) == 0)
    {
      FeebackControl(DPos, DVel, DAccel, FBPos, FBVel, Torque);
    }
    VectorCopy(FBPos, TempFBPos);
    VectorCopy(FBVel, TempFBVel);

    Torque2Joint(Torque, TempFBPos, TempFBVel, FBPos, FBVel, AccelOut, DELTA_T3);

    // Display the new Position
    DisplayConfiguration(FBPos);

    // Save point
    Write2File(simP, FBPos);
    microsleep(DELTA_T3*S_TO_MILIS);
  }

  CloseFile(simP);

}

void ForwardKin()
{
  double theta1 = 0, theta2 = 0, d3 = 0, theta4 = 0;

  std::cout << "Please enter the first joint variable THETA1" << std::endl;
  std::cin >> theta1;
  std::cout << "Please enter the second joint variable THETA2" << std::endl;
  std::cin >> theta2;
  std::cout << "Please enter the third joint variable D3" << std::endl;
  std::cin >> d3;
  std::cout << "Please enter the fourth joint variable THETA4" << std::endl;
  std::cin >> theta4;
  std::cout << std::endl << std::endl;

  vect CurPositionVect = { 0, 0, 0, 0 };
  WHERE(theta1, theta2, d3, theta4,CurPositionVect);
  if (CurPositionVect[2] == 0)
  {
    return;
  }
  cout << "Calculated tool frame relative to the station \n";
  DisplayV(CurPositionVect);

  char answer = 0x0;
  cout << "Would you like the robot to move there?\n Y or N\n>";
  cin >> answer;

  if (answer == 'y' || answer == 'Y')
  {

    cout << "Moving Robot \n";
    JOINT jointConfig = { theta1, theta2, d3, theta4 };
    MoveToConfiguration(jointConfig,true);
  }
  else if (answer == 'n' || answer == 'N')
  {
    cout << "okay then goodbye! :)\n";
    return;
  }
  else
  {
    cout << "Invalid Input!\n";
    return;
  }
}

void InverseKin()
{
  vect DesiredPosition = { 0, 0, 0, 0 };
  vect near_sol = { 0, 0, 0, 0 };
  vect far_sol = { 0, 0, 0, 0 };
  bool sol = false;
  int select = 0;
  JOINT config = { 0, 0, 0, 0 };

  cout << "Input the configuration for the tool frame relative to the station frame." << endl;
  cout << "x position >";
  cin >> DesiredPosition[0];
  cout << endl;
  cout << "y position >";
  cin >> DesiredPosition[1];
  cout << endl;
  cout << "z position >";
  cin >> DesiredPosition[2];
  cout << endl;
  cout << "phi angle >";
  cin >> DesiredPosition[3];
  cout << endl << endl;
  cout << "Calculating Joint Parameters" << endl;
  GetCurrentConfig(gCurrentConfig);
  matrix DesiredPositionMatrix = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };
  U2I(DesiredPosition, DesiredPositionMatrix);
  SOLVE(DesiredPositionMatrix, gCurrentConfig, near_sol, far_sol, sol);

  if (sol == false)
  {
    cout << "Desired Position Is not Possible. " <<
      "Please input a valid Position and Orientation." << endl;
    return;
  }

  cout << "Nearest Solution" << endl;
  DisplayV(near_sol);

  cout << "Farthest Solution" << endl;
  DisplayV(far_sol);

  cout << "0 : Don't move the robot and Exit\n";
  cout << "1 : Move using nearest solution" << 
    endl <<"2 : Move using farthest solution" << endl << ">";
  cin >> select;
  cout << endl << endl;

  if (select == 0)
  {
    return;
  }
  else if (select == 1)
  {
    VectToJoint(near_sol,config);
    MoveToConfiguration(config, true);
  }
  else if (select == 2)
  {
    VectToJoint(far_sol, config);
    MoveToConfiguration(config, true);
  }
  else
  {
    cout << "Invalid Selection." << endl;
    return;
  }

  //StopRobot();
  //ResetRobot();
}

void PickAndPlaceHelper()
{
  //vect desiredPosition1 = { 0, 0, 0, 0 };
  //vect desiredPosition2 = { 0, 0, 0, 0 };
  //double height = 0.0f;

  //cout << "Input the current object frame." << endl;
  //cout << "x position >";
  //cin >> desiredPosition1[0];
  //cout << endl;
  //cout << "y position >";
  //cin >> desiredPosition1[1];
  //cout << endl;
  //cout << "z position >";
  //cin >> desiredPosition1[2];
  //cout << endl;
  //cout << "phi angle >";
  //cin >> desiredPosition1[3];
  //cout << endl << endl;

  //cout << "Input the goal frame." << endl;
  //cout << "x position >";
  //cin >> desiredPosition2[0];
  //cout << endl;
  //cout << "y position >";
  //cin >> desiredPosition2[1];
  //cout << endl;
  //cout << "z position >";
  //cin >> desiredPosition2[2];
  //cout << endl;
  //cout << "phi angle >";
  //cin >> desiredPosition2[3];
  //cout << endl << endl;

  //cout << "Input Object height\n>";
  //cin >> height;

  //PickAndPlace(desiredPosition1, desiredPosition2, height);
}

void TrajectoryPlanning()
{
  int num_via;

  cout << "Input Number of Via points in file";
  cin >> num_via;
  cout << endl;

  matrix param1;
  matrix param2;
  matrix param3;
  matrix param4;
  MatrixInit(param1);
  MatrixInit(param2);
  MatrixInit(param3);
  MatrixInit(param4);


  double times[5] = { 0, 0, 0, 0, 0 };
  double viax[5] = { 0, 0, 0, 0, 0 };
  double viay[5] = { 0, 0, 0, 0, 0 };
  double viaz[5] = { 0, 0, 0, 0, 0 };
  double viaphi[5] = { 0, 0, 0, 0, 0 };

  ReadViaPoints(times, viax, viay, viaz, viaphi,num_via);

  vect jointPosVector[5] = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };
  for (int i = 0; i < num_via; i++)
  {
    vect tempConfig = { 0, 0, 0, 0 };
    vect closestSol = { 0, 0, 0, 0 };
    bool sol = false;

    matrix tempConfigMat;
    MatrixInit(tempConfigMat);

    tempConfig[0] = viax[i];
    tempConfig[1] = viay[i];
    tempConfig[2] = viaz[i];
    tempConfig[3] = viaphi[i];
    U2I(tempConfig, tempConfigMat);
    vect zeroVect = { 0, 0, -200, 0 };
    if (i == 0)
    {
      SOLVE(tempConfigMat, zeroVect, closestSol, zeroVect, sol);
      if (!sol)
      {
        std::cout << "TRAJPLAN: No solutions found for input config" << std::endl;
      }
      VectorCopy(closestSol, jointPosVector[i]);
      continue;
    }

    SOLVE(tempConfigMat, jointPosVector[i - 1], closestSol, zeroVect, sol);
    if (!sol)
    {
      std::cout << "TRAJPLAN: No solutions found for input config" << std::endl;
    }
    VectorCopy(closestSol, jointPosVector[i]);
  }

  double via1[5] = { 0, 0, 0, 0, 0 };
  double via2[5] = { 0, 0, 0, 0, 0 };
  double via3[5] = { 0, 0, 0, 0, 0 };
  double via4[5] = { 0, 0, 0, 0, 0 };

  for (int i = 0; i < num_via; i++)
  {
    via1[i] = jointPosVector[i][0];
    via2[i] = jointPosVector[i][1];
    via3[i] = jointPosVector[i][2];
    via4[i] = jointPosVector[i][3];
  }

  vect JointPosArray[MAX_DATA_POINTS];
  vect JointVelArray[MAX_DATA_POINTS];
  vect JointAccelArray[MAX_DATA_POINTS];
  int num_samples= 0;

  // Init Vectors
  for (int i = 0; i < MAX_DATA_POINTS; i++)
  {
    VectorInit(JointPosArray[i]);
    VectorInit(JointVelArray[i]);
    VectorInit(JointAccelArray[i]);
  }

  TraGen(times, via1, via2, via3, via4, param1, param2, param3, param4, 5);
  TraCalc(times, param1, param2, param3, param4, num_via, SAMPLING_RATE_T1, JointPosArray, JointVelArray, JointAccelArray,num_samples);

  TraExec(JointPosArray, JointVelArray, JointAccelArray, SAMPLING_RATE_T1, num_samples);

  StopRobot();
  ResetRobot();
  JOINT config = { 0, 0, 0, 0 };
  GetConfiguration(config);
  WHERE(config[0], config[1], config[2], config[3], config);
  DisplayV(config);
}

void DynamicSimHelper()
{
  //vect T = { 35.5334, 13.5407, -26.487, -1.6337};
  vect T = { 10, 0, -26.487, 0};
  vect pos = { -100, 100, -180, 0 };
  vect vel = { 0, 0, 0, 0 };
  vect accel = { 0, 0, 0, 0 };

  vect posOut = { 0, 0, 0, 0 };
  vect velOut = { 0, 0, 0, 0 };
  vect accelOut = { 0, 0, 0, 0 };

  vect posArray[NUM_DYNAMIC_SAMPLES];
  vect velArray[NUM_DYNAMIC_SAMPLES];
  vect accelArray[NUM_DYNAMIC_SAMPLES];

  DynamicSim(T, pos, vel, posOut, velOut, accelOut, posArray, velArray, accelArray, SIM_DUR, SAMPLE_RATE_X);


}

void InitRobot()
{
  // Reset the Robot
  StopRobot();
  ResetRobot();

  // Update Current Config
  GetCurrentConfig(gCurrentConfig);
}

void EndSession()
{
  StopRobot();
  CloseMonitor();
}

void main()
{
  // Initialize the Robot
  InitRobot();

  OpenMonitor();

  bool main_loop = true;

  while (main_loop)
  {
    cout << "Pick From the list of options\n\t0 : Exit\n\t1 : ForwardKin\n\t2 : InverseKin\n\t3 : Pick and Place\n\t4 : Toggle Grasp\n";
    cout << endl << ">";
    int user_input;
    cin >> user_input;
    cout << endl;
    if (user_input == 0)
    {
      main_loop = 0;
      return;
    }
    else if (user_input == 1)
    {
      ForwardKin();
    }
    else if (user_input == 2)
    {
      InverseKin();
    }
    else if (user_input == 3)
    {
      PickAndPlaceHelper();
    }

    else if (user_input == 4)
    {
      gGrasp = !gGrasp;
      Grasp(gGrasp);
    }
    else if (user_input == 5)
    {
      TrajectoryPlanning();
    }
    else if (user_input == 6)
    {
      DynamicSimHelper();
    }
    else if (user_input == 7)
    {
      RoboSim();
    }
    else
    {
      cout << "Invalid Input.\n";
    }
  }

  EndSession();
}
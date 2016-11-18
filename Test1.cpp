#include "stateid.h"
#include "ensc-488.h"
#include "WHERE.h"
#include "InverseKin.h"
#include "PickAndPlace.h"
#include "TrajectoryPlanning.h"

using namespace std;

// Globals //
vect gCurrentConfig; // Current_Robot Configuration
bool gGrasp = false;

//////////////////////
// Helper Functions //
//////////////////////

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
  matrix paramx;
  matrix paramy;
  matrix paramz;
  matrix paramphi;
  MatrixInit(paramx);
  MatrixInit(paramy);
  MatrixInit(paramz);
  MatrixInit(paramphi);

  vect CartConfigArray[MAX_DATA_POINTS];
  vect JointConfigArray[MAX_DATA_POINTS];
  vect JointVelArray[MAX_DATA_POINTS];

  // Init Vectors
  for (int i = 0; i < MAX_DATA_POINTS; i++)
  {
    VectorInit(CartConfigArray[i]);
    VectorInit(JointConfigArray[i]);
    VectorInit(JointVelArray[i]);
  }

  TraGen(via_times, via_x, via_y, via_z, via_phi, paramx, paramy, paramz, paramphi, 5);
  TraCalc(via_times, paramx, paramy, paramz, paramphi, 5, SAMPLING_RATE, CartConfigArray, JointConfigArray, JointVelArray);

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
    else
    {
      cout << "Invalid Input.\n";
    }
  }

  EndSession();
}
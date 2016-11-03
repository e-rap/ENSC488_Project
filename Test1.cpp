#include "stateid.h"
#include "ensc-488.h"
#include "WHERE.h"
#include "InverseKin.h"

using namespace std;

// Globals //
vect gCurrentConfig; // Current_Robot Configuration
//////////////////////
// Helper Functions //
//////////////////////
void JointToVect(JOINT joint, vect& vector)
{
  vect result;
  for (int i = 0; i < NUM_OF_LINK_VARS; i++)
  {
    result.push_back(joint[i]);
  }
  vector = result;
}

void VectToJoint(vect vector, JOINT& joint)
{
  for (int i = 0; i < NUM_OF_LINK_VARS; i++)
  {
    joint[i] = vector[i];
  }
}

void GetCurrentConfig(vect& cur_config)
{
  JOINT config;
  GetConfiguration(config);
  JointToVect(config, cur_config);
}

void InitRobot()
{
  // Reset the Robot
  StopRobot();
  ResetRobot();

  // Update Current Config
  GetCurrentConfig(gCurrentConfig);
}

void ForwardKin()
{
  vect CurPositionVect = WHERE();

}

void InverseKin()
{
  vect DesiredPosition(4);
  vect near_sol;
  vect far_sol;
  bool sol = false;
  int select = 0;
  JOINT config;


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
  SOLVE(UTOI(DesiredPosition), gCurrentConfig, near_sol, far_sol, sol);

  if (sol == false)
  {
    cout << "Desired Position Is not Possible." <<
      "Please input a valid Position and Orientation." << endl;
    return;
  }

  cout << "Nearest Solution" << endl;
  DisplayV(near_sol);

  cout << "Farthest Solution" << endl;
  DisplayV(far_sol);

  cout << "1 : Move using nearest solution" << 
    endl <<"2 : Move using farthest solution" << endl << ">";
  cin >> select;
  cout << endl << endl;

  if (select == 1)
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

void PickAndPlace()
{

}


void main()
{
  // Initialize the Robot
  InitRobot();

  OpenMonitor();

  bool main_loop = true;

  while (main_loop)
  {
    cout << "Pick From the list of options\n\t0 : Exit\n\t1 : ForwardKin\n\t2 : InverseKin";
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
    else
    {
      cout << "Invalid Input.\n";
    }
  }
}
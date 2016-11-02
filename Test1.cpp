#include "stateid.h"
#include "ensc-488.h"
#include "WHERE.h"
#include "InverseKin.h"

using namespace std;

// Globals //
vect gCurrentConfig; // Current_Robot Configuration

void GetCurrentConfig(vect& cur_config)
{
  JOINT config;
  GetConfiguration(config);
  for (int i = 0; i < NUM_OF_LINK_VARS; i++)
  {
    cur_config.push_back(config[i]);
  }
}

void InitRobot()
{
  StopRobot();
  ResetRobot();
  OpenMonitor();
}


void ForwardKin()
{

}

void InverseKin()
{

}

void PickAndPlace()
{

}


void main()
{
  InitRobot();


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
      main_loop == 0;
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
      cout << "Invalid Input!" << endl;
    }
  }
}
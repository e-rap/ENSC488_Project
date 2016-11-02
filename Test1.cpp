#include "matrix.h"
#include "WHERE.h"
#include "ensc-488.h"
#include "InverseKin.h"

using namespace std;

void main()
{
  float theta1 = 0.0f, theta2 = 0.0f, d3 = -150.0f, theta4 = 0.0f;
  vector<float> config = ITOU(KIN(theta1, theta2, d3, theta4));

  DisplayV(config);

  vector<float> near, far;
  bool sol;
  INVKIN(config, config, near, far, sol);

  cout << sol << endl << endl;

  //DisplayV(near);

  int x;
  cin >> x;
}
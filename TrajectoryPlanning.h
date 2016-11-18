#ifndef TrajectoryPlanning_h__
#define TrajectoryPlanning_h__

#include "matrix.h"
#include "RobotGlobals.h"
#include <exception>
#include <cmath>
#include "InverseKin.h"
#include <fstream>
#include <windows.h>

#define S_TO_MICROS 1000000.0

#define MAX_VIA_POINTS 5
#define SAMPLING_RATE 60
#define MAX_TIME 60
#define MAX_DATA_POINTS SAMPLING_RATE*MAX_TIME

void usleep(__int64 usec)
{
	HANDLE timer;
	LARGE_INTEGER ft;

	ft.QuadPart = -(10 * usec); // Convert to 100 nanosecond interval, negative value indicates relative time

	timer = CreateWaitableTimer(NULL, TRUE, NULL);
	SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0);
	WaitForSingleObject(timer, INFINITE);
	CloseHandle(timer);
}

//Read Via points from text file "viapoints"
void ReadViaPoints(double via_times[5], double x_via[5], double y_via[5], double z_via[5], double phi_via[5]) {
    double temp[5][5];
    std::ifstream in("viapoints.txt");
    
    if (!in) {
        std::cout << "Cannot open file.\n";
        return;
    }
    
    for (int line = 0; line < 5; line++) {
        for (int row = 0; row < 5; row++) {
            in >> temp[line][row];
        }
    }
    in.close();
    for(int i=0;i<5;i++){
        via_times[i]=temp[0][i];
    }
    
    for(int i=0;i<5;i++){
        x_via[i]=temp[1][i];
    }
    for(int i=0;i<5;i++){
        y_via[i]=temp[2][i];
    }
    for(int i=0;i<5;i++){
        z_via[i]=temp[3][i];
    }
    for(int i=0;i<5;i++){
        phi_via[i]=temp[4][i];
    }
}

// Description : Uses the Jacobian to convert joint space velocities to Cartesian space velocities
// Inputs:
//   JointConfig - Joint Configuration State (theta1,theta2,d3,theta4)
//   JointVel    - Current Joint Velocities (theta1dot, theta2dot, V3, theta4dot)
// Outputs:
//   CartVel     - Cartesian velocities (Vx, Vy, Vz, phidot)
// Throws Exception At singularities
void JointVel2CartVel(vect JointConfig, vect JointVel, vect& CartVel)
{
    
    double a= -L3*sin(DEG2RAD(JointConfig[0])) - L4*sin(DEG2RAD(JointConfig[0] + JointConfig[1]));
    double b= -L4*sin(DEG2RAD(JointConfig[0] + JointConfig[1]));
    double c= L3*cos(DEG2RAD(JointConfig[0])) + L4*cos(DEG2RAD(JointConfig[0] + JointConfig[1]));
    double d= L4*cos(DEG2RAD(JointConfig[0] + JointConfig[1]));
    
  // calculated the 2x2 det for x y theta1 theta2
    double det = a*d-b*c;
  // Check if at a singularity
  if (det == 0 || JointConfig[2] == -100 || JointConfig[2] == -200 || !Theta1Check2(JointConfig[0]) || !Theta2Check2(JointConfig[1]) || !D3Check2(JointConfig[2]) || !Theta4Check2(JointConfig[3]))
  {
    std::cout<<"Encountered Boundary singularity!";
    return;
  }
  //apply Jacobian
  CartVel[0] = a * JointVel[0]+b * JointVel[1];
  CartVel[1] = a * JointVel[0]+ b * JointVel[1];
  CartVel[2] = -JointVel[2];
  CartVel[3] = JointVel[0] + JointVel[1] - JointVel[3];
}


// Description : Uses the Inverse of the Jacobian to convert Cartesian space velocities to joint space velocities
// Inputs:
//   CartConfig  - Cartesian configuration vector (x,y,z,phi)
//   CartVel     - Cartesian velocity vector (Vx, Vy, Vz, phidot)
// Outputs:
//   JointVel    - Current joint velocity vector (theta1dot, theta2dot, V3, theta4dot)
// Throws Exception At singularities
void CartVel2JointVel(vect CartConfig, vect CartVel, vect& JointConfig, vect& JointVel)
{
  matrix CartConfigMatrix = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };
  U2I(CartConfig, CartConfigMatrix);
  vect _JointConfig = { 0, 0, 0, 0 };
  vect far_sol = { 0, 0, 0, 0 };
  bool sol = false;
  SOLVE(CartConfigMatrix, CartConfig, _JointConfig, far_sol, sol);

  if (sol == false)
  {
    std::cout << "No solution possible for specified position and orientation" << std::endl;
  }

    //calculate Jacobian
    double a= -L3*sin(DEG2RAD(_JointConfig[0])) - L4*sin(DEG2RAD(_JointConfig[0] + _JointConfig[1]));
    double b= -L4*sin(DEG2RAD(_JointConfig[0] + _JointConfig[1]));
    double c= L3*cos(DEG2RAD(_JointConfig[0])) + L4*cos(DEG2RAD(_JointConfig[0] + _JointConfig[1]));
    double d= L4*cos(DEG2RAD(_JointConfig[0] + _JointConfig[1]));
    
    double det = (a*d)-(b*c);
    
    //compute Inverse of 2x2 Jacobian
    double A=a/det;
    double B=-b/det;
    double C=-c/det;
    double D=d/det;
    
    
    //Check for singularities
#ifndef DEBUG
    if (det == 0 || !Theta1Check2(_JointConfig[0]) || !Theta2Check2(_JointConfig[1]) || !D3Check2(_JointConfig[2]) || !Theta2Check2(_JointConfig[3]))
    {
      throw std::exception("Encountered Boundary singularity!");
      return;
    }
#endif
    //apply inverse Jacobian

    JointVel[0] = A * CartVel[0] + B * CartVel[1];
    JointVel[1] = C * CartVel[0] + D * CartVel[1];
    JointVel[2] = -CartVel[2];
    JointVel[3] = JointVel[0] + JointVel[1] - CartVel[3];

    VectorCopy(_JointConfig, JointConfig);
}

// Generate Trajectory Parameters
void TraGen(double via_times[5], double x_via[5], double y_via[5], double z_via[5], double phi_via[5], matrix& paramx, matrix& paramy, matrix& paramz, matrix& paramphi, int nofvia){




  double h[4];
  for (int i = 0; i < 4; i++){
    h[i] = via_times[i + 1] - via_times[i];
  }


  //via pt velocities
  double vx_via[MAX_VIA_POINTS];
  double vy_via[MAX_VIA_POINTS];
  double vz_via[MAX_VIA_POINTS];
  double vphi_via[MAX_VIA_POINTS];

  for (int i = 0; i < nofvia; i++){
    vx_via[i] = 0;
    vy_via[i] = 0;
    vz_via[i] = 0;
    vphi_via[i] = 0;

  }

  //calculate via point velocities as average//
  //inital and final velocity remain 0 to obtain a smooth start/stop

  for (int i = 1; i < nofvia - 1; i++){
    vx_via[i] = (((x_via[i] - x_via[i - 1]) / h[i - 1]) + ((x_via[i + 1] - x_via[i]) / h[i])) / 2;
  }
  for (int i = 1; i < nofvia - 1; i++){
    vy_via[i] = (((y_via[i] - y_via[i - 1]) / h[i - 1]) + ((y_via[i + 1] - y_via[i]) / h[i])) / 2;
  }
  for (int i = 1; i < nofvia - 1; i++){
    vz_via[i] = (((z_via[i] - z_via[i - 1]) / h[i - 1]) + ((z_via[i + 1] - z_via[i]) / h[i])) / 2;
  }
  for (int i = 1; i < nofvia - 1; i++){
    vphi_via[i] = (((phi_via[i] - phi_via[i - 1]) / h[i - 1]) + ((phi_via[i + 1] - phi_via[i]) / h[i])) / 2;
  }

  //calculate parameters//

  // for x:
  for (int i = 0; i < nofvia - 1; i++){
    paramx[i][0] = x_via[i];  //parameter a
  }
  for (int i = 0; i < nofvia - 1; i++){
    paramx[i][1] = h[i] * vx_via[i]; //parameter b
  }
  for (int i = 0; i < nofvia - 1; i++){
    paramx[i][3] = h[i] * vx_via[i + 1] + 2 * paramx[i][0] + paramx[i][1] - 2 * x_via[i + 1]; //parameter d
  }
  for (int i = 0; i < nofvia - 1; i++){
    paramx[i][2] = x_via[i + 1] - paramx[i][0] - paramx[i][1] - paramx[i][3]; //parameter c
  }


  //for y:
  for (int i = 0; i < nofvia - 1; i++){
    paramy[i][0] = y_via[i];  //parameter a
  }
  for (int i = 0; i < nofvia - 1; i++){
    paramy[i][1] = h[i] * vy_via[i]; //parameter b
  }
  for (int i = 0; i < nofvia - 1; i++){
    paramy[i][3] = h[i] * vy_via[i + 1] + 2 * paramy[i][0] + paramy[i][1] - 2 * y_via[i + 1]; //parameter d
  }
  for (int i = 0; i < nofvia - 1; i++){
    paramy[i][2] = y_via[i + 1] - paramy[i][0] - paramy[i][1] - paramy[i][3]; //parameter c
  }


  //for z:
  for (int i = 0; i < nofvia - 1; i++){
    paramz[i][0] = z_via[i];  //parameter a
  }
  for (int i = 0; i < nofvia - 1; i++){
    paramz[i][1] = h[i] * vz_via[i]; //parameter b
  }
  for (int i = 0; i < nofvia - 1; i++){
    paramz[i][3] = h[i] * vz_via[i + 1] + 2 * paramz[i][0] + paramz[i][1] - 2 * z_via[i + 1]; //parameter d
  }
  for (int i = 0; i < nofvia - 1; i++){
    paramz[i][2] = z_via[i + 1] - paramz[i][0] - paramz[i][1] - paramz[i][3]; //parameter c
  }


  //for phi:
  for (int i = 0; i < nofvia - 1; i++){
    paramphi[i][0] = phi_via[i];  //parameter a
  }
  for (int i = 0; i < nofvia - 1; i++){
    paramphi[i][1] = h[i] * vphi_via[i]; //parameter b
  }
  for (int i = 0; i < nofvia - 1; i++){
    paramphi[i][3] = h[i] * vphi_via[i + 1] + 2 * paramphi[i][0] + paramphi[i][1] - 2 * phi_via[i + 1]; //parameter d
  }
  for (int i = 0; i < nofvia - 1; i++){
    paramphi[i][2] = phi_via[i + 1] - paramphi[i][0] - paramphi[i][1] - paramphi[i][3]; //parameter c
  }




  return;

}

// Calculate Trajectory Joint Value and Velocitiy and Acceleration
void TraCalc(double via_times[5], matrix paramx, matrix paramy, matrix paramz, matrix paramphi, int nofvia, unsigned int sampling_rate,
  vect* CartConfigArray,
  vect* JointConfigArray,
  vect* JointVelArray)
{
  vect num_seg_samples = { 0, 0, 0, 0 };
  double seg_offset[5] = { 0, 0, 0, 0, 0};
  int num_samples  = (via_times[nofvia-1] - via_times[0])*SAMPLING_RATE;
   
  // Calculating Sample Offsets and
  for (int i = 0; i < VECTOR_SIZE; i++)
  {
    num_seg_samples[i] = (via_times[i + 1] - via_times[i]) * SAMPLING_RATE;
    seg_offset[i + 1] = seg_offset[i] + num_seg_samples[i];
  }
  
  // Loop through segments
  for (int cur_seg = 0; cur_seg < nofvia-1; cur_seg++)
  {
    // Sample within segments
    for (int cur_sample = seg_offset[cur_seg]; cur_sample < seg_offset[cur_seg + 1]; cur_sample++)
    {
      // Parameterized Time 
      double tau = (cur_sample - seg_offset[cur_seg]) / num_seg_samples[cur_seg];

      // Calculating Polynomials for each sample

        CartConfigArray[cur_sample][0] = paramx[cur_seg][0] + paramx[cur_seg][1] * tau + paramx[cur_seg][2] * pow(tau, 2) + paramx[cur_seg][3] * pow(tau, 3);
        CartConfigArray[cur_sample][1] = paramy[cur_seg][0] + paramy[cur_seg][1] * tau + paramy[cur_seg][2] * pow(tau, 2) + paramy[cur_seg][3] * pow(tau, 3);
        CartConfigArray[cur_sample][2] = paramz[cur_seg][0] + paramz[cur_seg][1] * tau + paramz[cur_seg][2] * pow(tau, 2) + paramz[cur_seg][3] * pow(tau, 3);
        CartConfigArray[cur_sample][3] = paramphi[cur_seg][0] + paramphi[cur_seg][1] * tau + paramphi[cur_seg][2] * pow(tau, 2) + paramphi[cur_seg][3] * pow(tau, 3);
      
      // Use First Via Point
      if (cur_sample == 0)
      {
        matrix CartConfigMatrix = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };
        vect far_sol = { 0, 0, 0, 0 };
        bool sol = false;
        U2I(CartConfigArray[cur_sample], CartConfigMatrix);
        SOLVE(CartConfigMatrix, CartConfigArray[0], JointConfigArray[0], far_sol, sol);
      }

      // Calculate Next Via sample point
      else
      {
        vect curCartVel = { 0, 0, 0, 0 };
        VectorSub(CartConfigArray[cur_sample], CartConfigArray[cur_sample - 1], curCartVel);
        VectorMulS(curCartVel, sampling_rate, curCartVel);
        CartVel2JointVel(CartConfigArray[cur_sample], curCartVel, JointConfigArray[cur_sample], JointVelArray[cur_sample]);
      }
        

//testing
//      std::cout <<cur_sample <<"      " << CartConfigArray[cur_sample][0]<<"   " << CartConfigArray[cur_sample][1] <<                 std::endl << "      "  << JointConfigArray[cur_sample][0] << "   " << JointConfigArray[cur_sample][1]<< std::endl <<std::endl;
//        
//        std::cout<< cur_sample << "  " <<JointVelArray[cur_sample][0] << std::endl;
    }
//      std::cout<<std::endl;
      
  }
    
    // find maximum Joint Velocities
    vect JointVel_MAX={0,0,0,0};
    for(int i=0; i<4; i++){
        for(int j=0;j<num_samples;j++){
        
            if(fabs(JointVelArray[j][i])>JointVel_MAX[i]){
            JointVel_MAX[i]=fabs(JointVelArray[j][i]);
            }
        }
//        std:: cout <<JointVel_MAX[i] << "  ";
    }
    
//    std::cout <<std::endl;
    
    //Check if maximum Joint Velocities are valid
    
    if (!VelTheta1Check(JointVel_MAX[0]) || !VelTheta2Check(JointVel_MAX[1]) || !VelTheta4Check(JointVel_MAX[3]) || !VelD3Check(JointVel_MAX[2]))
    {
        std::cout<< "the required Joint Velocities for this trajectory are too high";
        return;
    }

    
    //Save sampled trajectory into txt file
    std::ofstream file ("data.txt");
    if(file.is_open()){
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < num_samples; j++){
                file << CartConfigArray[j][i] << " ";
            }
            file << "\n";
            
        }
        file.close();
    }
    else{
        std::cout<<"unable to open file";
    }

    
    

}

// Execute Trajectory
void TraExec( vect* JointConfigArray, vect* JointVelArray, vect* JointAccArray, double sampling_rate, unsigned int num_of_samples)
{
	// Move Robot to First Point
	std::cout << "Moving Robot to the initial Trajectory Point" << std::endl;
	MoveToConfiguration(JointConfigArray[0], true);

	// Calculate step duration
	double microseconds = (1.0 / sampling_rate) * S_TO_MICROS;

	for (int i = 0; i < num_of_samples; i++)
	{
		bool success = false;
		success = MoveWithConfVelAcc(JointConfigArray[i], JointVelArray[i], JointAccArray[i]);
		if (!success)
		{
			std::cout << "Could Not Move to Desired Position" << std::endl;
			return;
		}

		// Wait Until next step
		usleep(microseconds);
	}
	std::cout << "Done!" << std::endl;
}


#endif // TrajectoryPlanning_h__










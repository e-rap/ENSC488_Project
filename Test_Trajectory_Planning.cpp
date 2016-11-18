#include <iostream>
#include "TrajectoryPlanning.h"
#include "InverseKin.H"
#include "matrix.h"


using namespace std;




int main(int argc, const char * argv[]) {
    
      matrix paramx;
      matrix paramy;
      matrix paramz;
      matrix paramphi;
      MatrixInit(paramx);
      MatrixInit(paramy);
      MatrixInit(paramz);
      MatrixInit(paramphi);
    
 
 
      double times[5] = { 0, 0, 0, 0, 0};
      double viax[5] = { 0, 0, 0, 0, 0 };
      double viay[5] = { 0, 0, 0, 0, 0 };
      double viaz[5] = { 0, 0, 0, 0, 0 };
      double viaphi[5] = { 0, 0, 0, 0, 0 };
      ReadViaPoints(times, viax, viay, viaz, viaphi);
    
    

    
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

      TraGen(times, viax, viay, viaz, viaphi, paramx, paramy, paramz, paramphi, 5);
      TraCalc(via_times, paramx, paramy, paramz, paramphi, 5, SAMPLING_RATE, CartConfigArray, JointConfigArray, JointVelArray);
    

    
    
//////test SOLVE function
//    matrix test_matrix = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };
//    vect far_sol = { 0, 0, 0, 0 };
//    vect near_sol = { 0, 0, 0, 0 };
//    
//    vect test_vector={0,337,30,0};
//    bool sol = false;
//    U2I(test_vector, test_matrix);
//    SOLVE(test_matrix,test_vector, near_sol, far_sol, sol);
//    for(int i=0; i<4; i++){
//        cout << near_sol[i]<<" ";
//    }
//        cout << endl <<endl;
//
//
//////test output of TraGen
//    DisplayM(paramy);
    
    
    
    


}

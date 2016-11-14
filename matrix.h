#ifndef matrix_h__
#define matrix_h__

//basic 4x4 matrix functions for Robotics project
//by Chris Liedl
//
//
//How to use functions:
//
//  UTOI: User to Interface UTOI(configuration std::vector (x,y,z,phi))=T matrix of frame
//  ITOU: Interface to User ITOU(matrix A)=configuration std::vector (x,y,z,phi)
//  DisplayM(matrix A)-> output A
//  DisplayV(std::vector v)-> output v
//  Multiply(matrix A,matrix B) = A * B
//  Add(matrix A, matrix B) = A + B
//  Inverse(matrix A)=A^-1
//
//
//How to initialize a 4x4 matrix A:
//
//  matrix A(4, std::vector<double>(4));
//  A={{x,x,x,x},{x,x,x,x},{x,x,x,x},{x,x,x,x}};
//
//
//How to initialize a 4 std::vector v:
//
//  std::vector<double> v(4);
//  v={x,x,x,x};




#include <iostream>
#include <vector>
#include <cmath>

using vect = std::vector<double>;
using matrix = std::vector<std::vector<double>>; //matrix is std::vector of std::vector

// Add two Vectors
vect VectorAdd(vect a, vect b)
{
  if (a.size() != b.size())
  {
    throw std::exception("ERROR: Cannot add vectors of different sizes");
  }
  vect result;
  for (unsigned int i = 0; i < a.size(); i++)
  {
    result.push_back(a[i] + b[i]);
  }
  return result;
}

// Subtract two vectors
vect VectorSub(vect a, vect b)
{
  if (a.size() != b.size())
  {
    throw std::exception("ERROR: Cannot add vectors of different sizes");
    return vect();
  }
  vect result;
  for (unsigned int i = 0; i < a.size(); i++)
  {
    result.push_back(a[i] - b[i]);
  }
  return result;
}

// Multiply vector by scalar
vect VectorMulS(vect a, double constant)
{
  for (unsigned int i = 0; i < a.size(); i++)
  {
    a[i] = a[i] * constant;
  }
  return a;
}

// Magnitude of the vector
double VectorMag(vect a)
{
  double sum = 0.0f;
  for (unsigned int i = 0; i < a.size(); i++)
  {
    sum += pow(a[1], 2);
  }
  return sqrt(sum);
}

// The sum of the difference of each vector component
double VectorDiffSum(vect a, vect b)
{
  if (a.size() != b.size())
  {
    throw std::exception("ERROR: Cannot add vectors of different sizes");
    return 0.0f;
  }

  double sum = 0.0f;
  for (unsigned int i = 0; i < a.size(); i++)
  {
    sum += abs(a[i] - b[i]);
  }
  return sum;
}

//converts User input to internal representation of frames (std::vector (x,y,z,phi) to matrix T)
matrix UTOI(vect config, matrix result){

    config[3] = DEG2RAD(config[3]);

    
    result=  {{cos(config[3]),-sin(config[3]),0,config[0]},
            {sin(config[3]),cos(config[3]),0,config[1]},
            {0,0,1,config[2]},
            {0,0,0,1}};
    
 
}

//converts internal to user representation of frames (matrix T to std::vector (x,y,z,phi)
void ITOU(matrix frame, vector result){
    
  
    result={frame[0][3],frame[1][3],frame[2][3],RAD2DEG(atan2(frame[1][0],frame[0][0]))};
    
    
}

//Displays matrix
void DisplayM(matrix A){
    
    for(unsigned int i=0;i<A.size();i++){
        for(int j=0;j<A[0].size();j++){
            std::cout << A[i][j] << "   ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

//Displays std::vector
void DisplayV(vect vector){
    
    for(unsigned int i=0; i<VECTOR_SIZE; i++){
        std::cout<<vector[i]<< "   ";
    }
    std::cout<<std::endl<<std::endl;
}

//Multiplies 2 matrices
void Multiply(matrix A, matrix B, matrix result){
    
    matrix result(4, vect(4));
    
    for(unsigned int i=0;i<VECTOR_SIZE;i++){
        for(int j=0;j<VECTOR_SIZE;j++){
            for(int k=0;k<VECTOR_SIZE;k++){
                result[i][j]+=A[i][k]*B[k][j];
            }            
        }
    }
    
}

//Adds two matrices
void Add(matrix A, matrix B, matrix result){
    
    for(unsigned int i=0;i<VECTOR_SIZE;i++){
        for(int j=0;j<VECTOR_SIZE;j++){
            result[i][j]=A[i][j]+B[i][j];
        }
    }
}

//gives Inverse of matrix
void Inverse(matrix A, matrix result){
    
    
    //Rotation part of T is just transposing
    for (unsigned int i=0; i<VECTOR_SIZE;i++){
        for (int j=0; j<VECTOR_SIZE;j++){
            result[i][j]=A[j][i];
        }
    }
    
    //new origin std::vector is -R_T*v_old
    for(int k=0; k<VECTOR_SIZE; k++){
        for(unsigned int i=0; i<VECTOR_SIZE;i++){
            result[k][3]+=(-A[i][k])*A[i][3];
        }
    }
    //last row doesn't change
    for(unsigned int i=0; i<VECTOR_SIZE; i++){
        result[3][i]=A[3][i];
    }
}

#endif // matrix_h__

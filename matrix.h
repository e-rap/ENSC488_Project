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
//  matrix A(4, std::vector<float>(4));
//  A={{x,x,x,x},{x,x,x,x},{x,x,x,x},{x,x,x,x}};
//
//
//How to initialize a 4 std::vector v:
//
//  std::vector<float> v(4);
//  v={x,x,x,x};




#include <iostream>
#include <vector>
#include <cmath>

using matrix = std::vector<std::vector<float>>; //matrix is std::vector of std::vector

// Add two Vectors
std::vector<float> VectorAdd(std::vector<float> a, std::vector<float> b)
{
  if (a.size() != b.size())
  {
    throw std::exception("ERROR: Cannot add vectors of different sizes");
  }
  std::vector<float> result;
  for (int i = 0; i < a.size(); i++)
  {
    result.push_back(a[i] + b[i]);
  }
  return result;
}

// Subtract two vectors
std::vector<float> VectorSub(std::vector<float> a, std::vector<float> b)
{
  if (a.size() != b.size())
  {
    throw std::exception("ERROR: Cannot add vectors of different sizes");
    return 0.0f;
  }
  std::vector<float> result;
  for (int i = 0; i < a.size(); i++)
  {
    result.push_back(a[i] - b[i]);
  }
  return result;
}

// Multiply vector by scalar
std::vector<float> VectorMulS(std::vector<float> a, float constant)
{
  for (int i = 0; i < a.size(); i++)
  {
    a[i] = a[i] * constant;
  }
  return a;
}

// Magnitude of the vector
float VectorMag(std::vector<float> a)
{
  float sum = 0.0f;
  for (int i = 0; i < a.size(); i++)
  {
    sum += pow(a[1], 2);
  }
  return sqrt(sum);
}

// The sum of the difference of each vector component
float VectorDiffSum(std::vector<float> a, std::vector<float> b)
{
  if (a.size() != b.size())
  {
    throw std::exception("ERROR: Cannot add vectors of different sizes");
    return 0.0f;
  }

  float sum = 0.0f;
  for (int i = 0; i < a.size(); i++)
  {
    sum += abs(a[i] - b[i]);
  }
  return sum;
}

//converts User input to internal representation of frames (std::vector (x,y,z,phi) to matrix T)
matrix UTOI(std::vector<float> config){
    
    matrix frame(4, std::vector<float>(4));
    
    frame=  {{cos(config[3]),-sin(config[3]),0,config[0]},
            {sin(config[3]),cos(config[3]),0,config[1]},
            {0,0,1,config[2]},
            {0,0,0,1}};
    
    return frame;
}

//converts internal to user representation of frames (matrix T to std::vector (x,y,z,phi)
std::vector<float> ITOU(matrix frame){
    
    std::vector<float> config(4);
    config={frame[0][3],frame[1][3],frame[2][3],atan2(frame[1][0],frame[0][0])};
    
    return config;
}

//Displays matrix
void DisplayM(matrix A){
    
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            std::cout << A[i][j] << "   ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

//Displays std::vector
void DisplayV(std::vector<float> vector){
    
    for(int i=0; i<4; i++){
        std::cout<<vector[i]<< "   ";
    }
    std::cout<<std::endl<<std::endl;
}

//Multiplies 2 matrices
matrix Multiply(matrix A, matrix B){
    
    matrix result(4, std::vector<float>(4));
    
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            for(int k=0;k<4;k++){
                result[i][j]+=A[i][k]*B[k][j];
            }            
        }
    }
    return result;
}

//Adds two matrices
matrix Add(matrix A, matrix B){
    
    matrix result(4, std::vector<float>(4));
    
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            result[i][j]=A[i][j]+B[i][j];
        }
    }
    return result;
}

//gives Inverse of matrix
matrix Inverse(matrix A){
    
    matrix result(4, std::vector<float>(4));
    
    //Rotation part of T is just transposing
    for (int i=0; i<3;i++){
        for (int j=0; j<3;j++){
            result[i][j]=A[j][i];
        }
    }
    
    //new origin std::vector is -R_T*v_old
    for(int k=0; k<3; k++){
        for(int i=0; i<3;i++){
            result[k][3]+=-A[i][k]*A[i][3];
        }
    }
    //last row doesn't change
    for(int i=0; i<4; i++){
        result[3][i]=A[3][i];
    }
    return result;
}

#endif // matrix_h__

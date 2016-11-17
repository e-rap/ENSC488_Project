//Trajectory Generation
//  TraGen function
//
//  Created by Christian Liedl on 16.11.16.
//  Copyright © 2016 Christian Liedl. All rights reserved.
//
//
//  input parameters:
//  times: times of via points
//  x_via, y_via....: x,y,z,phi values of via points
//
//  output:
//  paramx, paramy, paramz, paramphi: matrices containing the parameters for the interpolating functions
//  paramx={{a1,b1,c1,d1},{a2,b2,c2,d2},{a3,b3,c3,d4},{a4,b4,c4,d4}}
//  the corresponding functions are: fi(t)=ai+bi*(t-times[i])/h[i]+c*((t-times[i])/h[i])^2+d*((t-times[i])/h[i])^3
//  fi(t) is defined between times[i-1] and times[i]




#include <cmath>
#include "matrix.h"


//typedef double vect[4];
//typedef double matrix[4][4];




#ifndef TraGen_h
#define TraGen_h



void TraGen(vect times, vect x_via, vect y_via, vect z_via, vect phi_via, matrix paramx, matrix paramy, matrix paramz, matrix paramphi, int nofvia){
    
    
    
   
    double h[4];
    for (int i=0;i<4;i++){
        h[i]=times[i+1]-times[i];
    }
    
    
    //via pt velocities
    double vx_via[nofvia];
    double vy_via[nofvia];
    double vz_via[nofvia];
    double vphi_via[nofvia];
    
    for(int i=0;i<nofvia;i++){
        vx_via[i]=0;
        vy_via[i]=0;
        vz_via[i]=0;
        vphi_via[i]=0;
        
    }
    
    //calculate via point velocities as average//
    //inital and final velocity remain 0 to obtain a smooth start/stop
    
    for (int i=1;i<nofvia-1;i++){
        vx_via[i]=(((x_via[i]-x_via[i-1])/h[i-1])+((x_via[i+1]-x_via[i])/h[i]))/2;
    }
    for (int i=1;i<nofvia-1;i++){
        vy_via[i]=(((y_via[i]-y_via[i-1])/h[i-1])+((y_via[i+1]-y_via[i])/h[i]))/2;
    }
    for (int i=1;i<nofvia-1;i++){
        vz_via[i]=(((z_via[i]-z_via[i-1])/h[i-1])+((z_via[i+1]-z_via[i])/h[i]))/2;
    }
    for (int i=1;i<nofvia-1;i++){
        vphi_via[i]=(((phi_via[i]-phi_via[i-1])/h[i-1])+((phi_via[i+1]-phi_via[i])/h[i]))/2;
    }

    //calculate parameters//
    
    // for x:
    for(int i=0;i<nofvia-1;i++){
        paramx[i][0]=x_via[i];  //parameter a
    }
    for(int i=0;i<nofvia-1;i++){
        paramx[i][1]=h[i]*vx_via[i]; //parameter b
    }
    for(int i=0;i<nofvia-1;i++){
        paramx[i][3]=h[i]*vx_via[i+1]+2*paramx[i][0]+paramx[i][1]-2*x_via[i+1]; //parameter d
    }
    for(int i=0;i<nofvia-1;i++){
        paramx[i][2]=x_via[i+1]-paramx[i][0]-paramx[i][1]-paramx[i][3]; //parameter c
    }
    
    
    //for y:
    for(int i=0;i<nofvia-1;i++){
        paramy[i][0]=y_via[i];  //parameter a
    }
    for(int i=0;i<nofvia-1;i++){
        paramy[i][1]=h[i]*vy_via[i]; //parameter b
    }
    for(int i=0;i<nofvia-1;i++){
        paramy[i][3]=h[i]*vy_via[i+1]+2*paramy[i][0]+paramy[i][1]-2*y_via[i+1]; //parameter d
    }
    for(int i=0;i<nofvia-1;i++){
        paramy[i][2]=y_via[i+1]-paramy[i][0]-paramy[i][1]-paramy[i][3]; //parameter c
    }
    
    
    //for z:
    for(int i=0;i<nofvia-1;i++){
        paramz[i][0]=z_via[i];  //parameter a
    }
    for(int i=0;i<nofvia-1;i++){
        paramz[i][1]=h[i]*vz_via[i]; //parameter b
    }
    for(int i=0;i<nofvia-1;i++){
        paramz[i][3]=h[i]*vz_via[i+1]+2*paramz[i][0]+paramz[i][1]-2*z_via[i+1]; //parameter d
    }
    for(int i=0;i<nofvia-1;i++){
        paramz[i][2]=z_via[i+1]-paramz[i][0]-paramz[i][1]-paramz[i][3]; //parameter c
    }
    
    
    //for phi:
    for(int i=0;i<nofvia-1;i++){
        paramphi[i][0]=phi_via[i];  //parameter a
    }
    for(int i=0;i<nofvia-1;i++){
        paramphi[i][1]=h[i]*vphi_via[i]; //parameter b
    }
    for(int i=0;i<nofvia-1;i++){
        paramphi[i][3]=h[i]*vphi_via[i+1]+2*paramphi[i][0]+paramphi[i][1]-2*phi_via[i+1]; //parameter d
    }
    for(int i=0;i<nofvia-1;i++){
        paramphi[i][2]=phi_via[i+1]-paramphi[i][0]-paramphi[i][1]-paramphi[i][3]; //parameter c
    }
    

    
    
    return;

}


#endif /* TraGen_h */

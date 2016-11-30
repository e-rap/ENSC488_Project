clear all
clc

%%%%%%%%%%%
% Constants
%%%%%%%%%%%

syms L1 L2 L3 L4 L5 L6 L7 L8 theta1 theta2 theta4 d3 m1 m2 m3 m4 g thetav1 thetav2 dv3 thetav4 thetaa1 thetaa2 da3 thetaa4 T1 T2 f3 T4

%center of mass
P1c1 = sym([0;0;0]);
P2c2 = sym([0;0;0]);
P3c3 = sym([0;0;L2]);
P4c4 = sym([L8;0;0]);

CMArray = sym(zeros(3,4));
CMArray(:,1) = P1c1;
CMArray(:,2) = P2c2;
CMArray(:,3) = P3c3;
CMArray(:,4) = P4c4;

%mass array
MArray = [m1;m2;m3;m4];

%frame origins
P01 = [0;0;L1];
P12 = [L3;0;L2];
P23 = [L4;0;0];
P34 = [0;0;L5+L6+d3];
P45 = [0;0;L7];
FrameOriginArray(:,1) = P01;
FrameOriginArray(:,2) = P12; 
FrameOriginArray(:,3) = P23; 
FrameOriginArray(:,4) = P34; 
FrameOriginArray(:,5) = P45; 

%Rotation Matricies
R01 = [cos(theta1) -sin(theta1) 0; sin(theta1) cos(theta1) 0; 0 0 1];
R12 = [cos(theta2) -sin(theta2) 0; sin(theta2) cos(theta2) 0; 0 0 1];
R23 = [1 0 0; 0 -1 0; 0 0 -1];
R34 = [cos(theta4) -sin(theta4) 0; sin(theta4) cos(theta4) 0; 0 0 1];
R45 = [1 0 0; 0 1 0; 0 0 1];

RotationArray(:,:,1) = R01;
RotationArray(:,:,2) = R12;
RotationArray(:,:,3) = R23;
RotationArray(:,:,4) = R34;
RotationArray(:,:,5) = R45;

%Moments of Inertia
Izeros = sym(zeros(3,3));
InertiaArray(:,:,1) = Izeros;
InertiaArray(:,:,2) = Izeros;
InertiaArray(:,:,3) = Izeros;
InertiaArray(:,:,4) = [ 0 0 0; 0 0 0; 0 0 m4*L8^2];


%angular vel/accel | vel/accel
AngVelArray = sym(zeros(3,5));
AngAccelArray = sym(zeros(3,5));
VelArray = sym(zeros(3,5));

AccelArray = sym(zeros(3,5));
AccelArray(:,1) = [0;0;g];

%Joint AngVel
JointAngVelArray(1) = thetav1;
JointAngVelArray(2) = thetav2;
JointAngVelArray(3) = 0;
JointAngVelArray(4) = thetav4;

%Joint Vel
JointVelArray(1) = sym(0);
JointVelArray(2) = sym(0);
JointVelArray(3) = dv3;
JointVelArray(4) = sym(0);

%Joint AngAccel
JointAngAccelArray(1) = thetaa1;
JointAngAccelArray(2) = thetaa2;
JointAngAccelArray(3) = 0;
JointAngAccelArray(4) = thetaa4;


%Joint Accel
JointAccelArray(1) = sym(0);
JointAccelArray(2) = sym(0);
JointAccelArray(3) = da3;
JointAccelArray(4) = sym(0);

% Center of Mass Accel Array
AccelCMArray = sym(zeros(3,4));

%Zvector
Zvec = [0;0;1];

%equations
for i = 1:4
    AngVelArray(:,i+1) = (RotationArray(:,:,i).') * AngVelArray(:,i) + (JointAngVelArray(i)*Zvec); %JointANgVelArray(i) because Sizes
    AngAccelArray(:,i+1) = (RotationArray(:,:,i).') * AngAccelArray(:,i) + cross(((RotationArray(:,:,i).')*AngVelArray(:,i)),(JointAngVelArray(i))* Zvec) + JointAngAccelArray(i)*Zvec;
    AccelArray(:,i+1) = (RotationArray(:,:,i).')*(cross(AngAccelArray(:,i), FrameOriginArray(:,i)) + cross(AngVelArray(:,i),cross(AngVelArray(:,i),FrameOriginArray(:,i)))+ AccelArray(:,i)) + 2*cross(AngVelArray(:,i+1),JointVelArray(i)*Zvec) + JointAccelArray(i)*Zvec;
    AccelCMArray(:,i) = cross(AngAccelArray(:,i+1),CMArray(:,i)) + cross(AngVelArray(:,i+1),AngVelArray(:,i+1)+CMArray(:,i)) + AccelArray(:,i+1);
end


% Joint Torque
JointnArray = sym(zeros(3,5));
% JointnArray(:,2) = T1*Zvec;
% JointnArray(:,3) = T2*Zvec;
% JointnArray(:,5) = T4*Zvec;

% Joint Force
JointfArray = sym(zeros(3,5));
% JointfArray(:,4) = f3*Zvec;

FArray = sym(zeros(3,4));
NArray = sym(zeros(3,4));

 for i = 1:4
    FArray(:,i) = MArray(i) * AccelCMArray(:,i);  
 end
 NArray(:,4) = InertiaArray(:,:,4) * AngAccelArray(:,5) + cross(AngVelArray(:,5),InertiaArray(:,:,4)*AngVelArray(:,5));
 
for i = 4:-1:1
   JointfArray(:,i) =  FArray(:,i) + RotationArray(:,:,i+1)*JointfArray(:,i+1);
   JointnArray(:,i) = NArray(:,i) + RotationArray(:,:,i+1)*JointnArray(:,i+1) + cross(CMArray(:,i), FArray(:,i)) + cross(FrameOriginArray(:,i+1),RotationArray(:,:,i+1)*JointfArray(:,i+1));
end

%solve for torques 
T = sym(zeros(1,4));
T(1) = JointnArray(:,1).' * Zvec;
T(2) = JointnArray(:,2).' * Zvec;
T(3) = JointfArray(:,3).' * Zvec;
T(4) = JointnArray(:,4).' * Zvec;

eqn1 = simplify(T1 == T(1));
eqn2 = simplify(T2 == T(2));
eqn3 = simplify(f3 == T(3));
eqn4 = simplify(T4 == T(4));




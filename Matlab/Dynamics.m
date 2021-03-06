clear all
clc

%%%%%%%%%%%
% Constants
%%%%%%%%%%%

syms L1 L2 L3 L4 L5 L6 L7 L8 theta1 theta2 theta4 d3 m1 m2 m3 m4 g thetav1 thetav2 dv3 thetav4 thetaa1 thetaa2 da3 thetaa4 T1 T2 f3 T4 FrictionCoef

%center of mass
P1c1 = sym([0;0;0]);
P2c2 = sym([0;0;0]);
P3c3 = sym([0;0;L2]);
P4c4 = sym([L8;0;0]);

CM = sym(zeros(3,4));
CM(:,1) = P1c1;
CM(:,2) = P2c2;
CM(:,3) = P3c3;
CM(:,4) = P4c4;

%mass array
Mass = [m1;m2;m3;m4];

%frame origins
P01 = [0;0;L1];
P12 = [L3;0;L2];
P23 = [L4;0;0];
P34 = [0;0;L5+L6+d3];
P45 = [0;0;L7];
FrameOrigin(:,1) = P01;
FrameOrigin(:,2) = P12; 
FrameOrigin(:,3) = P23; 
FrameOrigin(:,4) = P34; 
FrameOrigin(:,5) = P45; 

%Rotation Matricies
R01 = [cos(theta1) -sin(theta1) 0; sin(theta1) cos(theta1) 0; 0 0 1];
R12 = [cos(theta2) -sin(theta2) 0; sin(theta2) cos(theta2) 0; 0 0 1];
R23 = [1 0 0; 0 -1 0; 0 0 -1];
R34 = [cos(theta4) -sin(theta4) 0; sin(theta4) cos(theta4) 0; 0 0 1];
R45 = [1 0 0; 0 1 0; 0 0 1];

Rotation(:,:,1) = R01;
Rotation(:,:,2) = R12;
Rotation(:,:,3) = R23;
Rotation(:,:,4) = R34;
Rotation(:,:,5) = R45;

%Moments of Inertia
Izeros = sym(zeros(3,3));
Inertia(:,:,1) = Izeros;
Inertia(:,:,2) = Izeros;
Inertia(:,:,3) = Izeros;
Inertia(:,:,4) = Izeros;


%angular vel/accel | vel/accel
Omega = sym(zeros(3,5));
OmegaDot = sym(zeros(3,5));
Vel = sym(zeros(3,5));

Accel = sym(zeros(3,5));
Accel(:,1) = [0;0;g];

%Joint AngVel
JointOmega(1) = thetav1;
JointOmega(2) = thetav2;
JointOmega(3) = 0;
JointOmega(4) = thetav4;

%Joint Vel
JointVel(1) = sym(0);
JointVel(2) = sym(0);
JointVel(3) = dv3;
JointVel(4) = sym(0);

%Joint AngAccel
JointOmegaDot(1) = thetaa1;
JointOmegaDot(2) = thetaa2;
JointOmegaDot(3) = 0;
JointOmegaDot(4) = thetaa4;


%Joint Accel
JointAccel(1) = sym(0);
JointAccel(2) = sym(0);
JointAccel(3) = da3;
JointAccel(4) = sym(0);

% Center of Mass Accel Array
AccelCM = sym(zeros(3,4));

%Zvector
Zvec = [0;0;1];

%equations
for i = 1:4
    Omega(:,i+1) = (Rotation(:,:,i).') * Omega(:,i) + (JointOmega(i)*Zvec); %JointANgVelArray(i) because Sizes
    OmegaDot(:,i+1) = (Rotation(:,:,i).') * OmegaDot(:,i) + cross(((Rotation(:,:,i).')*Omega(:,i)),(JointOmega(i))* Zvec) + JointOmegaDot(i)*Zvec;
    Accel(:,i+1) = (Rotation(:,:,i).')*(cross(OmegaDot(:,i), FrameOrigin(:,i)) + cross(Omega(:,i),cross(Omega(:,i),FrameOrigin(:,i)))+ Accel(:,i)) + 2*cross(Omega(:,i+1),JointVel(i)*Zvec) + JointAccel(i)*Zvec;
    AccelCM(:,i) = cross(OmegaDot(:,i+1),CM(:,i)) + cross(Omega(:,i+1),Omega(:,i+1)+CM(:,i)) + Accel(:,i+1);
end


% Joint Torque
Jointn = sym(zeros(3,5));
% JointnArray(:,2) = T1*Zvec;
% JointnArray(:,3) = T2*Zvec;
% JointnArray(:,5) = T4*Zvec;

% Joint Force
Jointf = sym(zeros(3,5));
% JointfArray(:,4) = f3*Zvec;

F = sym(zeros(3,4));
N = sym(zeros(3,4));

 for i = 1:4
    F(:,i) = Mass(i) * AccelCM(:,i);  
 end
 
for i = 4:-1:1
   Jointf(:,i) =  F(:,i) + Rotation(:,:,i+1)*Jointf(:,i+1);
   Jointn(:,i) = N(:,i) + Rotation(:,:,i+1)*Jointn(:,i+1) + cross(CM(:,i), F(:,i)) + cross(FrameOrigin(:,i+1),Rotation(:,:,i+1)*Jointf(:,i+1));
end

%solve for torques 
T = sym(zeros(1,4));
T(1) = Jointn(:,1).' * Zvec;
T(2) = Jointn(:,2).' * Zvec;
T(3) = Jointf(:,3).' * Zvec;
T(4) = Jointn(:,4).' * Zvec;
T=T.';

Friction(1) = -FrictionCoef*thetav1;
Friction(2) = -FrictionCoef*thetav2;
Friction(3) = -FrictionCoef*dv3;
Friction(4) = -FrictionCoef*thetav4;
Friction=Friction.';


c=[thetaa1; thetaa2; da3; thetaa4];
[M,b] = equationsToMatrix([T(1), T(2), T(3), T(4)], [thetaa1 thetaa2 da3 thetaa4]);
EqnVector = [T(1);T(2);T(3);T(4)]-M*c;
M=simplify(M);

V(1)=simplify(EqnVector(1));
V(2)=simplify(EqnVector(2));
V(3)=simplify(EqnVector(3)+(m3+m4)*g);
V(4)=simplify(EqnVector(4));
V=V.';

G(1)=sym(0);
G(2)=sym(0);
G(3)=-(m3+m4)*g;
G(4)=sym(0);
G=G.';

Tsol=[T1;T2;f3;T4];
Thetaa=simplify(inv(M)*(Tsol-V-G-Friction));
T=simplify(T+Friction);

% Opening files

plannedPFile = fopen('plannedP.txt', 'r');
plannedVFile = fopen('plannedV.txt', 'r');
plannedAFile = fopen('plannedA.txt', 'r');

simPFile = fopen('simP.txt', 'r');
simVFile = fopen('simV.txt', 'r');
simAFile = fopen('simA.txt', 'r');

XYFile = fopen('XY.txt', 'r');

% Setting format to read data (input file should be 'theta1 theta2 d3 theta4 /n')

formatSpec1 = '%f %f %f %f %f '; % for joint variable files
formatSpec2 = '%f %f '; % for x-y mapping file

% Set up size of array where we'll be putting data

size1 = [5 Inf];
size2 = [2 Inf];

% Putting file contents into array of column vectors

PP = fscanf(plannedPFile, formatSpec1, size1);
PV = fscanf(plannedVFile, formatSpec1, size1);
PA = fscanf(plannedAFile, formatSpec1, size1);

SP = fscanf(simPFile, formatSpec1, size1);
SV = fscanf(simVFile, formatSpec1, size1);
SA = fscanf(simAFile, formatSpec1, size1);

XY = fscanf(XYFile, formatSpec2, size2);

% Closing files

fclose(plannedPFile);
fclose(plannedVFile);
fclose(plannedAFile);

fclose(simPFile);
fclose(simVFile);
fclose(simAFile);

fclose(XYFile);

% Extracting info from files for plottable variables

[m,n] = size(PP);
[a,b] = size(SP);
[c,d] = size(XY);

% Time 

tPlanned = PP(1, 1:n);
tSim = SP(1, 1:b);

% Planned position

theta1PP = PP(2, 1:n);
theta2PP = PP(3, 1:n);
d3PP = PP(4, 1:n);
theta4PP = PP(5, 1:n);

% Planned velocity

theta1PV = PV(2, 1:n);
theta2PV = PV(3, 1:n);
d3PV = PV(4, 1:n);
theta4PV = PV(5, 1:n);

% Planned accel

theta1PA = PA(2, 1:n);
theta2PA = PA(3, 1:n);
d3PA = PA(4, 1:n);
theta4PA = PA(5, 1:n);

% Simulator Output position

theta1SP = SP(2, 1:b);
theta2SP = SP(3, 1:b);
d3SP = SP(4, 1:b);
theta4SP = SP(5, 1:b);

% Simulator Output velocity

theta1SV = SV(2, 1:b);
theta2SV = SV(3, 1:b);
d3SV = SV(4, 1:b);
theta4SV = SV(5, 1:b);

% Simulator Output accel

theta1SA = SA(2, 1:b);
theta2SA = SA(3, 1:b);
d3SA = SA(4, 1:b);
theta4SA = SA(5, 1:b);

% X-Y Mapping

x = XY(1,1:d);
y = XY(2,1:d);


% PLOTTING!!!!!!! -----------------------------------

figure

% Joint Positions

subplot(3,4,1)
plot(tPlanned, theta1PP, tSim, theta1SP)
title('Joint 1 Position')
xlabel('Time')
ylabel('Position (\circ)')

subplot(3,4,2)
plot(tPlanned, theta2PP, tSim, theta2SP)
title('Joint 2 Position')
xlabel('Time')
ylabel('Position (\circ))')

subplot(3,4,3)
plot(tPlanned, d3PP, tSim, d3SP)
title('Joint 3 Position')
xlabel('Time')
ylabel('Position (mm)')

subplot(3,4,4)
plot(tPlanned, theta4PP, tSim, theta4SP)
title('Joint 4 Position')
xlabel('Time')
ylabel('Position (\circ))')

% Joint Velocities

subplot(3,4,5)
plot(tPlanned, theta1PV, tSim, theta1SV)
title('Joint 1 Velocity')
xlabel('Time')
ylabel('Velocity (\circ)/s)')

subplot(3,4,6)
plot(tPlanned, theta2PV, tSim, theta2SV)
title('Joint 2 Velocity')
xlabel('Time')
ylabel('Velocity (\circ)/s)')

subplot(3,4,7)
plot(tPlanned, d3PV, tSim, d3SV)
title('Joint 3 Velocity')
xlabel('Time')
ylabel('Velocity (mm/s)')

subplot(3,4,8)
plot(tPlanned, theta4PV, tSim, theta4SV)
title('Joint 4 Velocity')
xlabel('Time')
ylabel('Velocity (\circ)/s)')

% Joint Accelerations

subplot(3,4,9)
plot(tPlanned, theta1PA, tSim, theta1SA)
title('Joint 1 Acceleration')
xlabel('Time')
ylabel('Acceleration (\circ/s^{2})')

subplot(3,4,10)
plot(tPlanned, theta2PA, tSim, theta2SA)
title('Joint 2 Acceleration')
xlabel('Time')
ylabel('Acceleration (\circ/s^{2})')

subplot(3,4,11)
plot(tPlanned, d3PA, tSim, d3SA)
title('Joint 3 Acceleration')
xlabel('Time')
ylabel('Acceleration (mm/s^{2})')

subplot(3,4,12)
plot(tPlanned, theta4PA, tSim, theta4SA)
title('Joint 4 Acceleration')
xlabel('Time')
ylabel('Acceleration (\circ/s^{2})')

% X-Y Map

figure
plot(x,y)
title('X-Y Path of Tool')
xlabel('X')
ylabel('Y')



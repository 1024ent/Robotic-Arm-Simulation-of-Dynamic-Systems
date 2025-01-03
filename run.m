% This is the Matlab script file which sets up and runs the three link
% planar robot simulation

clear all
close all
clc

% Make robot parameters global for the three link function
global a1 a2 a3 m1 m2 m3 g

rtd = 180.0/pi;

% Set up parameters
a1 = 0.8; % meters
a2 = 1.1; % meters
a3 = 0.5; % meters
m1 = 15; % kg
m2 = 12; % kg
m3 = 5; % kg
g = 9.81; % gravity m/s^2

% Set up the values for ode23 call
t0 = 0; % seconds
tf = 2; % seconds
Y0 = [pi/2 0 0 0 0 0]; % Include 6 elements for angles and velocities
tspan = [t0 tf];
[T, Y] = ode23('planarRR', tspan, Y0);

Q1 = Y(:,1);
Q2 = Y(:,2);
Q3 = Y(:,3);
Q1d = Y(:,4);
Q2d = Y(:,5);
Q3d = Y(:,6);

% Plot graph position vs time
plot(T, Q1*rtd, 'r-', T, Q2*rtd, 'b:', T, Q3*rtd, 'g-');
legend('theta-1', 'theta-2', 'theta-3');
ylabel('Position (Degrees)');
xlabel('Time (sec)');
title('Planar 3R Robot Simulation');
pause;

% Plot graph position vs time
plot(T, Q1d*rtd, 'r-', T, Q2d*rtd, 'b:', T, Q3d*rtd, 'g-');
legend('omega-1', 'omega-2', 'omega-3');
ylabel('Velocity (deg/sec)');
xlabel('Time (sec)');
title('Planar 3R Robot Simulation');
pause;

close;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Info
% Author: Taylor Smith, Christian Williams
% Version: 1.0
% Created: 5/3/2023
% 
% Description
%   Script to model the hanging down of a pendulum
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
close all;
% pkg load control
% pkg load signal
set(gcf,'Visible','on')

% Measured Parameters
mass_of_copter = 0.0132;
mass_of_rod = 0.0234;
length_of_rod = 0.5021;
length_of_rod_to_pivot = 0.29;
radius_of_copter = 0.01;

% Damping Coefficient
b = 0.60;

% Constants and Basic Calculated Parameters
g = 9.81;                         
density_of_rod = mass_of_rod/length_of_rod;                
length_of_back_rod = length_of_rod - length_of_rod_to_pivot; 
mass_of_rod_to_pivot = density_of_rod * length_of_rod_to_pivot; 
total_mass_of_rod = density_of_rod * length_of_rod;
mass_of_back_rod = density_of_rod * length_of_back_rod;

% Moment of Inertia
inertia = 1/3*mass_of_rod_to_pivot*length_of_rod_to_pivot^2 + 1/3*mass_of_back_rod*length_of_back_rod^2 + 2/5 * mass_of_copter * radius_of_copter^2 + mass_of_copter * (length_of_rod_to_pivot + radius_of_copter)^2;

% State Space Model
A = [[0,1];[-(g*length_of_rod_to_pivot*(mass_of_copter+mass_of_rod_to_pivot/2))/inertia,-b]];
B = [0; length_of_rod_to_pivot/inertia];
C = [[1, 0]; [0, 1]];
D = 0;

CopterSys = ss(A, B, C, D);
dCopterSys = c2d(CopterSys,.005);

% Full Order Observer

Vd = .1*eye(2);
Vn = 1*eye(2);

% Disturbance and noise included
Bf= [B Vd 0*B];

disp("Kf")
[kf,p,e] = lqe(A,Vd,C,Vd,Vn)
kf = (lqr(A',C',Vd,Vn))
disp("Discrete Kf")
[dkf,dp,de] = dlqe(dCopterSys.a,Vd,C,Vd,Vn)

disp("Simulation")
sysC = ss(A,Bf,C,[0*Vn Vn])

disp("System without noise")
sysFull = ss(A,Bf,eye(2),zeros(2,size(Bf,2)))

disp("System with noise")
sysKF = ss(A-kf*C,[B kf],eye(2),0*[B kf])

disp("Discrete system simulation")
dsysC = c2d(sysC,.005)

disp("Discrete system without noise")
dsysFull = c2d(sysFull,.005)

disp("Discrete system with noise")
dsysKF = c2d(sysKF,.005)

dt = .01;
t = dt:dt:50;
disturbance = randn(2,size(t,2));
noise = randn(size(t));
u=0*t;
u(100:105) = 3.14;
u(1500:1505) = -1.32;
augment = [u; Vd*Vd*disturbance;noise]';

[y,t,x] = lsim(sysC,augment,t);
[xtrue,t] = lsim(sysFull,augment,t);
[x,t] = lsim(sysKF,[u;y']',t);
plot(t,y(:,2))
hold on
plot(t,xtrue(:,2),'b','LineWidth',2.0)
plot(t,x(:,2)','k--','LineWidth',2.0)
set(gca, "fontsize", 12)
title("Full Order Observer")
xlabel("Time (s)");
ylabel("Thetadot (rads/s)");
legend("Noise","Actual","Estimate")
legend("show")



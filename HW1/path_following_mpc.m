%% params
% params for the car
m = 1615; % kg
l = 2.6; % m
w = 1.5; % m
r = 0.2; % m
Cr = 0.015; % rolling resistance
G = 7; % gear ratio
A = 2; % m^2
rho = 1.2; % kg/m^3
Cd = 0.45; % drag coefficient
yita = 0.9; % efficiency of the motor

% params for the controller
mode = 1; % 1 for efficient mode, 2 for sport mode
line = 1; % 1 for straight line path, 2 for spline path, 3 for circle path

h = 0.2; % sampling time
N = 20; % prediction horizon

kappa = 0;
Q = zeros(3,3);
R = zeros(2,2);
E = 0;

mode = 1; 
line = 1;
choose_mode;

% params for the simulation
t_cont = 0.01; % continuous time
t_sim = 20; % simulation time
t = 0:t_cont:t_sim; % time 

%% models
import casadi.*

% system model
% states
s_dt = SX.sym('dt'); % dt
s_delta = SX.sym('delta'); % steering angle
s_v = SX.sym('v'); % velocity
s_x = SX.sym('x'); % x position
s_y = SX.sym('y'); % y position
s_theta = SX.sym('theta'); % heading angle
s_states = [s_x;s_y;s_theta]; % states

input = [s_delta, s_v]; % inputs

% path model

%% control model
% fuel consumption model

%% simulation
    
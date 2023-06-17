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

if mode ==1
    if line == 1
        % straight line path, efficient mode
        kappa = 30;
        Q = diag([4.75,4.75,100]);
        R = diag([3,20]);
        E = 2.5*10^-10;
    elseif line == 2
        % spline path, efficient mode
        kappa = 20;
        Q = diag([10,10,2]);
        R = diag([1,1]);
        E = 2.5*10^-10;
    elseif line == 3
        % circle path, efficient mode
        kappa = 0.157;
        Q = diag([100,100,0]);
        R = diag([1,1]);
        E = 2.5*10^-10;
    else
        disp('wrong line number');
    end
elseif mode == 2
    if line == 1
        % straight line path, sport mode
        kappa = 30;
        Q = diag([4.75,4.75,100]);
        R = diag([1,1]);
        E = 0;
    elseif line == 2
        % spline path, sport mode
        kappa = 20;
        Q = diag([10,10,2]);
        R = diag([1,1]);
        E = 0;
    elseif line == 3
        % circle path, sport mode
        kappa = 0.157;
        Q = diag([100,100,0]);
        R = diag([1,1]);
        E = 0;
    else
        disp('wrong line number');
    end
else
    disp('wrong mode number');
end

% params for the simulation

%% models
import casadi.*

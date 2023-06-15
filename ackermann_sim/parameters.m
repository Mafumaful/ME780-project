%% close all
close all
clc

%% import libraries
import casadi.*

%% params
wheel_base = 2.908;
wheel_track = 1.566;

% calculate the max number of delta
% reference audi A4L
% https://car.autohome.com.cn/config/series/692.html
alpha_lim = 0.6;
inner_alpha_max = wheel_base / tan(alpha_lim);
delta_max = atan(wheel_base / (inner_alpha_max + 0.5 * wheel_track));
delta_lim = delta_max;
delta_lim_degree = rad2deg(delta_max);

%% forward Kinematic for Car-Like vehicles
s_px = SX.sym('px'); % position x
s_py = SX.sym('py'); % position y
s_delta = SX.sym('delta'); % steering angle
s_theta = SX.sym('theta'); % heading angle
s_v = SX.sym('v'); % velocity
s_w = SX.sym('w'); % steering angula speed
s_dt = SX.sym('dt'); % step time

% state variable: [x,y,theta]
s_x_dot = s_v * cos(s_theta);
s_y_dot = s_v * sin(s_theta);
s_theta_dot = s_v / wheel_base * tan(s_delta);
s_delta_dot = s_w;

rhs = [s_x_dot; s_y_dot; s_theta_dot];
% calculate the increment
calc_increment = Function('increment', {s_v, s_theta, s_w, s_delta}, {rhs});
% calculate next state
next_state = calc_increment(s_v, s_theta, s_w, s_delta) * s_dt + [s_px; s_py; s_theta];
calc_next_state = Function('next_s', {s_v, s_theta, s_w, s_delta, s_px, s_py, s_dt}, {next_state});

param.calc_increment = next_state;
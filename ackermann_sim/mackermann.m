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
s_psi = SX.sym('psi'); % steering angle
s_theta = SX.sym('theta'); % heading angle

s_v = SX.sym('v'); % velocity
s_w = SX.sym('w'); % steering angula speed
s_dt = SX.sym('dt'); % step time

% state variable: [x,y,theta]
s_x_dot = s_v * cos(s_theta);
s_y_dot = s_v * sin(s_theta);
s_theta_dot = s_v / wheel_base * tan(s_psi);
s_psi_dot = s_w;

rhs = [s_x_dot; s_y_dot; s_theta_dot; s_psi_dot];
% calculate the increment
calc_increment = Function('increment', {s_v, s_w, s_theta, s_psi}, {rhs});
% calculate next state
next_state = calc_increment(s_v, s_w, s_theta, s_psi) * s_dt + [s_px; s_py; s_theta; s_psi];
calc_next_state = Function('next_s', {s_v, s_w, s_theta, s_psi, s_px, s_py, s_dt}, {next_state});

%% simulation loop
% initial time settings
h_cont = 0.01; % step time
sim_time = 10; % 20 seconds simulation
time = 0:h_cont:sim_time; % time

% initial state settings
state = [0; 0; 0; 0.1];

% simulation loop
for i = 1:length(time)
    % calculate next state
    state = calc_next_state(10, 0, state(3), state(4), state(1), state(2), h_cont);
    % plot
    plot(full(state(1)), full(state(2)), 'r*');
    grid on;
    hold on;
    %     pause(0.01);
end

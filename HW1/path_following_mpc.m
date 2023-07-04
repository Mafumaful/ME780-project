close all
clc
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
gravity = 9.18; % m/s^2

% initialize the params
Q = zeros(3, 3);
R = zeros(2, 2);
E = 0;
% params for the controller
cmode = 2; % 1 for efficient mode, 2 for sport mode
cline = 1; % 1 for straight line path, 2 for spline path, 3 for circle path
choose_mode;

h = 0.2; % sampling time
h_cont = 0.01; % continuous time
N = 20; % prediction horizon

% params for the simulation
t_cont = 0.01; % continuous time
t_sim = 1000; % simulation time
t = 0:t_cont:t_sim; % time

%% models
import casadi.*

% lane model
s_epsilon = SX.sym('epsilon'); % a scalar path variable
s_lx = SX.sym('lx'); % x position of the lane
s_ly = SX.sym('ly'); % y position of the lane
s_theta = SX.sym('theta'); % heading angle of the lane
% we describe the lane as a straight line, a spline or a circle in cartesian coordinate
if cline == 1
    % straight line
    s_lx = s_epsilon;
    s_ly = 0;
    s_theta = 0;
elseif cline == 2
    % spline
    var = s_epsilon / 1000 * 200;
    s_lx = var;
    s_ly = 0.025 * var * var;
    s_theta = atan(0.05 * var);

elseif cline == 3
    % circle
    angle = (s_epsilon / 1000) * 3/2 * pi - pi / 2;
    s_lx = 50 * cos(angle);
    s_ly = 50 * sin(angle);
    %     aim_theta = (angle+pi/2)+((angle-pi*3/2)-(angle+pi/2))*sign(angle-3/2*pi);
    aim_theta = (angle + pi / 2);
    s_theta = aim_theta;
end

line = Function('line', {s_epsilon}, {s_lx, s_ly, s_theta});

[final_x, final_y, final_theta] = line(1000);
final_pose = [final_x; final_y; final_theta];

% system model
% states
s_dt = SX.sym('dt'); % dt
s_delta = SX.sym('delta'); % steering angle
s_frontspeed = SX.sym('frontSpeed'); % velocity
s_x = SX.sym('x'); % x position
s_y = SX.sym('y'); % y position
s_theta = SX.sym('theta'); % heading angle
s_states = [s_x; s_y; s_theta]; % states

input = [s_delta, s_frontspeed]; % inputs
s_xdot = r * s_frontspeed * (cos(s_delta) * cos(s_theta) - 0.5 * sin(s_delta) * sin(s_theta)); % state derivatives
s_ydot = r * s_frontspeed * (cos(s_delta) * sin(s_theta) + 0.5 * sin(s_delta) * cos(s_theta));
s_thetadot = r * s_frontspeed * sin(s_delta) / l;
bicycle_increment = [s_xdot; s_ydot; s_thetadot]; % state increments
calc_increment = Function('calc_increment', {s_states, input}, {bicycle_increment}); % function for calculating state increments

% fuel consumption model
s_a = SX.sym('a'); % acceleration
Torque = r / G * (m * s_a + m * gravity * Cr + 0.5 * rho * A * Cd * (s_frontspeed * r) * (s_frontspeed * r));
calc_torque = Function('calc_torque', {s_a, s_frontspeed}, {Torque}, struct('allow_free', 1)); % function for calculating fuel consumption

%% control model
% fuel consumption model
P = SX.sym('P', 6); % parameters
U = SX.sym('U', 2, N); % inputs
X = SX.sym('X', 3, N + 1); % states

obj = 0; % MPC objective
g = []; % MPC constraints

st = X(:, 1); % initial state
g = [g; st - P(1:3)]; % initial state constraint

for k = 1:N
    st = X(:, k); % current state
    input = U(:, k); % current input
    st_next = X(:, k + 1); % next state
    obj = obj + (st - P(4:6))' * Q * (st - P(4:6)); % + input' * R * input; % objective function

    % fuel consumption model
    if k > 1
        a = r * (U(2, k) - U(2, k - 1)) / h_cont;
        torque_x_wheelspeed = max(U(2, k) * r / G * (m * a + m * gravity * Cr + 0.5 * rho * A * Cd * (U(2, k) * r) * (U(2, k) * r)),0); % torque
        obj = obj + torque_x_wheelspeed' * E * torque_x_wheelspeed; % objective function

        comsumption = U(:, k) - U(:, k - 1);
        obj = obj + comsumption' * R * comsumption;
    end

    % kutta method
    k1 = calc_increment(st, input);
    k2 = calc_increment(st + h / 2 * k1, input);
    k3 = calc_increment(st + h / 2 * k2, input);
    k4 = calc_increment(st + h * k3, input);
    st_next_euler = st + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4); % euler method

    g = [g; st_next - st_next_euler]; % dynamic constraint
end

obj = obj + (st_next - P(4:6))' * diag([100, 100, 100]) * (st_next - P(4:6)); % terminal cost

% create a struct to hold the MPC control problem
OPT_variables = [reshape(X, 3 * (N + 1), 1); reshape(U, 2 * N, 1)];
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

% create solver
opts = struct;
opts.ipopt.max_iter = 200;
opts.ipopt.print_level = 0; % 0, 3
opts.print_time = 0;
opts.ipopt.acceptable_tol = 1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob, opts);

args = struct;
args.lbg(1:3 * (N + 1)) = 0; % equality constraints
args.ubg(1:3 * (N + 1)) = 0; % equality constraints

args.lbx(1:3:3 * (N + 1), 1) = -inf; % lower bound of x
args.ubx(1:3:3 * (N + 1), 1) = inf; % upper bound of x
args.lbx(2:3:3 * (N + 1), 1) = -inf; % lower bound of y
args.ubx(2:3:3 * (N + 1), 1) = inf; % upper bound of y
args.lbx(3:3:3 * (N + 1), 1) = -inf; % lower bound of theta
args.ubx(3:3:3 * (N + 1), 1) = inf; % upper bound of theta

args.lbx(3 * (N + 1) + 1:2:3 * (N + 1) + 2 * N, 1) = -0.5; % lower bound of delta
args.ubx(3 * (N + 1) + 1:2:3 * (N + 1) + 2 * N, 1) = 0.5; % upper bound of delta
args.lbx(3 * (N + 1) + 2:2:3 * (N + 1) + 2 * N, 1) = 0; % lower bound of front speed
args.ubx(3 * (N + 1) + 2:2:3 * (N + 1) + 2 * N, 1) = 100/3.6 / r; % upper bound of front speed

%% simulation
% target position
target_state = [0; 0; 0]; % target position

if cline == 1
    x0 = [0; 1; 0.1]; %initial state for line
elseif cline == 2
    x0 = [0; 0; 0]; % initial state for spline
else
    x0 = [-3; -50; 0]; % initial state for circle
end

xx = zeros(3, length(t)); % states
xx(:, 1) = x0; % initial state
u0 = zeros(N, 2); % initial control input
X0 = repmat(xx(:, 1), 1, N + 1)'; % initial state decision variables

record_target_theta = zeros(3, t_sim / h);
record_steering_angle = zeros(1, t_sim / h);
record_speed = zeros(1, t_sim / h_cont);
record_wheel_speed = zeros(1,t_sim/h_cont);
cnt = 1;
iter = 0;

% start simulation
for i = 1:length(t) - 1
    iter = iter + 1;

    if mod(i, h / h_cont) == 1
        % set parameters
        target_state = return_x_reference(x0, line, kappa);

        args.p = [x0; target_state];

        % set initial condition
        args.x0 = [reshape(X0', 3 * (N + 1), 1); reshape(u0', 2 * N, 1)];

        % solve the optimization problem
        sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx, 'lbg', args.lbg, 'ubg', args.ubg, 'p', args.p);

        % get solution
        u = reshape(full(sol.x(3 * (N + 1) + 1:end))', 2, N)'; % get controls only
        u_opt = u(1, :); % get optimal control
        record_target_theta(:, cnt) = full(target_state);
        record_steering_angle(cnt) = full(u_opt(1) * 180 / pi);

        cnt = cnt + 1;
    end

    x0 = full(x0 + h_cont * calc_increment(x0, u_opt)); % update initial state
    % calculate the next state
    % x0 = full(sol.x(4:6, :));
    xx(:, i + 1) = x0; % get solution trajectory
    record_speed(i) = full(u_opt(2) * r * 3.6);
    record_wheel_speed(i) = full(u_opt(2));

    if norm(x0 - full(final_pose)) < 1
        break;
    end

end

%% plot the result
% first plot
figure(1)
% plot according to iteration
plot(xx(1, 1:iter), xx(2, 1:iter), 'b.-', 'LineWidth', 2);
% plot(xx(1, :), xx(2, :), 'b.-', 'LineWidth', 2);
hold on;
% plot corresponding heading direction
% quiver(xx(1, :), xx(2, :), 0.5 * cos(xx(3, :)), 0.5 * sin(xx(3, :)), 'r');
% plot(full(final_pose(1)), full(final_pose(2)), 'r*');
% plot target position
plot(record_target_theta(1, 1:10:cnt - 1), record_target_theta(2, 1:10:cnt - 1), 'r-*', 'LineWidth', 1);
legend('Vehicle Path', 'Reference Path');
xlabel('x (m)', 'FontSize', 12);
ylabel('y (m)', 'FontSize', 12);
% add title of the vehicle
title('Position', 'FontSize', 12);
hold off;
% axis equal;
grid on;
% drawnow;


% second plot
figure(2)
% plot the angle onver time

plot(t(1:iter), record_speed(1:iter), 'b.-', 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Speed (km/h)', 'FontSize', 12);
title('Speed[km/h]', 'FontSize', 12);
grid on;

xlim([0,76]);

% third plot
figure(3)
t2 = h:h:t_sim;
% plot the target position
plot(t2(1:cnt - 1), record_steering_angle(1:cnt - 1), 'b.-', 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Steering Angle[\circ]', 'FontSize', 12);
title('Steering Angle[\circ]', 'FontSize', 12);
grid on;

xlim([0,76]);

% calculate energy consumption
energy = zeros(1, iter);

for i = 1:iter

    if i > 1
        a = (record_speed(i) - record_speed(i - 1)) * r / h_cont;
        torque = max(full(calc_torque(a, record_speed(i))),0);
        energy(i) = energy(i - 1) + torque * record_speed(i) / yita * h_cont;
    end

end

figure(4)
plot(t(1:iter), energy(1:iter), 'b.-', 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Energy (Joule)', 'FontSize', 12);
title('Energy Consumption[Joule]', 'FontSize', 12);
grid on;

xlim([0,76]);

% get the reference target for example position, target
function target_state = return_x_reference(x_real, target_line, kappa)
    import casadi.*
    % Symbols/expressions
    x = x_real(1);
    y = x_real(2);
    s = MX.sym('s');
    [lx, ly, ~] = target_line(s);

    f = (x - lx) ^ 2 + (y - ly) ^ 2;
    g = [];

    nlp = struct; % NLP declaration
    nlp.x = s; % decision vars
    nlp.f = f; % objective
    nlp.g = g; % constraints

    % Create solver instance, silent
    % create solver
    opts = struct;
    opts.ipopt.max_iter = 200;
    opts.ipopt.print_level = 0; % 0, 3
    opts.print_time = 0;
    opts.ipopt.acceptable_tol = 1e-8;
    opts.ipopt.acceptable_obj_change_tol = 1e-6;

    F = nlpsol('solver', 'ipopt', nlp, opts);

    % Solve the problem using a guess
    sol = F('x0', 0, 'ubg', 0, 'lbg', 0);

    psi = kappa - kappa / (1 + 0.01 * (1000 - sol.x))
    [x_ref, y_ref, theta] = target_line(sol.x + psi);
    theta = mod(full(theta), 2 * pi);
    target_state = [x_ref; y_ref; theta];
    full(target_state)
end

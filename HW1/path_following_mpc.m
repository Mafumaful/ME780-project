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
cmode = 1; % 1 for efficient mode, 2 for sport mode
cline = 3; % 1 for straight line path, 2 for spline path, 3 for circle path
choose_mode;

h = 0.2; % sampling time
h_cont = 0.01; % continuous time
N = 20; % prediction horizon

Q = zeros(3, 3);
R = zeros(2, 2);
E = 0;

% params for the simulation
t_cont = 0.01; % continuous time
t_sim = 92; % simulation time
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
    % todo
elseif cline == 3
    % circle
    angle = (s_epsilon/1000)*3/2*pi-pi/2;
    s_lx = 50 * cos(angle);
    s_ly = 50 * sin(angle);
%     aim_theta = (angle+pi/2)+((angle-pi*3/2)-(angle+pi/2))*sign(angle-3/2*pi);
    aim_theta = (angle+pi/2);
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
    obj = obj + (st - P(4:6))' * Q * (st - P(4:6)) + input' * R * input; % objective function

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
args.ubx(3 * (N + 1) + 2:2:3 * (N + 1) + 2 * N, 1) = 1000; % upper bound of front speed

%% simulation
% target position
target_state = [0; 0; 0]; % target position
if cline == 1
    x0 = [0;1;0.1]; %initial state for line
elseif cline ==2
    x0 = [0; -50; 0]; % initial state for circle
else
    x0=[-2; -50; 0];
end


xx = zeros(3, length(t)); % states
xx(:, 1) = x0; % initial state
u0 = zeros(N, 2); % initial control input
X0 = repmat(xx(:, 1), 1, N + 1)'; % initial state decision variables

% start simulation
for i = 1:length(t) - 1

    if mod(i, h / h_cont) == 1
        % set parameters
        error = norm(xx(:, i) - final_pose);
        target_state = return_x_reference(x0, line, kappa, error);
        aaa = full(target_state);
%         if aaa(1) <= -1 && aaa(2)<=-1
%             break
%         end
        args.p = [x0; target_state];

        % set initial condition
        args.x0 = [reshape(X0', 3 * (N + 1), 1); reshape(u0', 2 * N, 1)];

        % solve the optimization problem
        sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx, 'lbg', args.lbg, 'ubg', args.ubg, 'p', args.p);

        % get solution
        u = reshape(full(sol.x(3 * (N + 1) + 1:end))', 2, N)'; % get controls only
        u_opt = u(1, :); % get optimal control
    end

    xx(:, i + 1) = full(sol.x(1:3)); % get solution trajectory
    x0 = full(x0 + h_cont * calc_increment(x0, u_opt)); % update initial state
    % calculate the next state
    % x0 = full(sol.x(4:6, :));
    xx(:, i + 1) = x0; % get solution trajectory
end

%% plot the result
% first plot
figure(1)
plot(xx(1, :), xx(2, :), 'b.-', 'LineWidth', 2);
hold on;
% plot corresponding heading direction
quiver(xx(1, :), xx(2, :), 2 * cos(xx(3, :)), 2 * sin(xx(3, :)), 'r');
plot(full(final_pose(1)), full(final_pose(2)), 'r*');
hold off;
axis equal;
grid on;
drawnow;

% second plot
figure(2)
% plot the angle onver time
% cut t as the same length as xx(3, :)
x = linspace(0, t(end), length(xx(3, :)));
plot(x, xx(3, :), 'b.-', 'LineWidth', 2);


% get the reference target for example position, target
function target_state = return_x_reference(x_real, target_line, kappa, error)
    import casadi.*
    % Symbols/expressions
    x = x_real(1);
    y = x_real(2);
    s = MX.sym('s');
    [lx, ly, ~] = target_line(s);

    f = (x - lx) ^ 2 + (y - ly) ^ 2;
    g = [];

    psi = kappa - kappa / (1 + error);
    psi = 50;

    nlp = struct; % NLP declaration
    nlp.x = s; % decision vars
    nlp.f = f; % objective
    nlp.g = g; % constraints

    % Create solver instance
    F = nlpsol('F', 'ipopt', nlp);

    % Solve the problem using a guess
    sol = F('x0', 0, 'ubg', 0, 'lbg', 0);
    [x_ref, y_ref, theta] = target_line(sol.x + psi);
    theta = mod(full(theta),2*pi);
    target_state = [x_ref; y_ref; theta];
    full(target_state)
end

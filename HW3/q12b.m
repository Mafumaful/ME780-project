%% params
velo = 50/3.6;
% Vehicle parameters
m1 = mf;      % Mass of the first sprung mass (kg)
m2 = unsprung_mass;       % Mass of the second sprung mass (kg)
k1 = kf;     % Spring stiffness for the first mass (N/m)
k2 = vertcal_stiffness;      % Spring stiffness for the second mass (N/m)
c1 = cr_absorb;      % Damping coefficient for the first mass (Ns/m)
c2 = dist_absorb;       % Damping coefficient for the second mass (Ns/m)

% Half-sine road profile parameters
A = 0.1;       % Amplitude (m)
f = velo/2/wavelength;          % Frequency (Hz)

%% simulation
% Simulation parameters
duration = 5;      % Duration of simulation (seconds)
dt = 0.01;          % Time step (seconds)

% Time vector
t = 0:dt:duration;
n = length(t);

% Road profile (half-sine)
road_profile = A * sin(pi*f*t) .* (t >= 0 & t <= 1/f);

% Vehicle initial conditions
initial_displacement = [0; 0];       % Initial displacement [x1; x2] (m)
initial_velocity = [0; 0];           % Initial velocity [dx1/dt; dx2/dt] (m/s)

% Initialize arrays to store the response
displacement = zeros(2, n);
velocity = zeros(2, n);
acceleration = zeros(2, n);

% Perform numerical integration (Euler method)
displacement(:, 1) = initial_displacement;
velocity(:, 1) = initial_velocity;
for i = 2:n
    % Calculate accelerations using the 2-DOF model equations of motion
    acceleration(1, i-1) = (1/m1) * (road_profile(i) - k1*displacement(1, i-1) - c1*velocity(1, i-1) - k2*(displacement(1, i-1) - displacement(2, i-1)));
    acceleration(2, i-1) = (1/m2) * (k2*(displacement(1, i-1) - displacement(2, i-1)) - c2*velocity(2, i-1));
    
    % Update velocities and displacements using the Euler method
    velocity(:, i) = velocity(:, i-1) + acceleration(:, i-1) * dt;
    displacement(:, i) = displacement(:, i-1) + velocity(:, i) * dt;
end

% Plot the response
figure;
subplot(3, 1, 1);
plot(t, road_profile);
xlabel('Time (s)');
ylabel('Road Profile (m)');
title('Half-Sine Road Excitation');

subplot(3, 1, 2);
plot(t, displacement(1, :), 'b', t, displacement(2, :), 'r');
legend('Displacement x1', 'Displacement x2');
xlabel('Time (s)');
ylabel('Displacement (m)');
title('Vehicle Response');

subplot(3, 1, 3);
plot(t, acceleration(1, :), 'b', t, acceleration(2, :), 'r');
legend('Acceleration x1', 'Acceleration x2');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Vehicle Acceleration');

% Calculate and display the maximum displacements and accelerations
max_displacement_x1 = max(abs(displacement(1, :)));
max_displacement_x2 = max(abs(displacement(2, :)));
max_acceleration_x1 = max(abs(acceleration(1, :)));
max_acceleration_x2 = max(abs(acceleration(2, :)));

disp(['Maximum displacement x1: ' num2str(max_displacement_x1) ' m']);
disp(['Maximum displacement x2: ' num2str(max_displacement_x2) ' m']);
disp(['Maximum acceleration x1: ' num2str(max_acceleration_x1) ' m/s^2']);
disp(['Maximum acceleration x2: ' num2str(max_acceleration_x2) ' m/s^2']);

%% params
velo = 5/3.6;

% Vehicle parameters
m = mf;       % Mass of the vehicle (kg)
k = kkf;      % Spring stiffness (N/m)
c = cf_absorb;       % Damping coefficient (Ns/m)

% Half-sine road profile parameters
A = 0.1;       % Amplitude (m)
f = velo/2/wavelength;          % Frequency (Hz)

% Simulation parameters
duration = 5;      % Duration of simulation (seconds)
dt = 0.01;          % Time step (seconds)


%% simulation
% Time vector
t = 0:dt:duration;
n = length(t);

% Road profile (half-sine)
road_profile = A * sin(pi*f*t) .* (t >= 0 & t <= 1/f);

% Vehicle initial conditions
initial_displacement = 0;       % Initial displacement (m)
initial_velocity = 0;           % Initial velocity (m/s)

% Initialize arrays to store the response
displacement = zeros(1, n);
velocity = zeros(1, n);
acceleration = zeros(1, n);

% Perform numerical integration (Euler method)
displacement(1) = initial_displacement;
velocity(1) = initial_velocity;
for i = 2:n
    acceleration(i-1) = (k/m) * (road_profile(i-1) - displacement(i-1)) - (c/m) * velocity(i-1);
    velocity(i) = velocity(i-1) + acceleration(i-1) * dt;
    displacement(i) = displacement(i-1) + velocity(i) * dt;
end

% Plot the response
figure;
subplot(3, 1, 1);
plot(t, road_profile);
xlabel('Time (s)');
ylabel('Road Profile (m)');
grid on;
title('Wheel Travel through a Half-Sine Road Excitation');

subplot(3, 1, 2);
plot(t, displacement);
xlabel('Time (s)');
ylabel('Displacement (m)');
grid on;
title('Vehicle Displacement');

subplot(3, 1, 3);
plot(t, acceleration);
xlabel('Time (s)');
grid on;
ylabel('Acceleration (m/s^2)');
title('Vehicle Acceleration');

% Calculate and display the maximum displacement and acceleration
max_displacement = max(abs(displacement));
max_acceleration = max(abs(acceleration));
disp(['Maximum displacement: ' num2str(max_displacement) ' m']);
disp(['Maximum acceleration: ' num2str(max_acceleration) ' m/s^2']);

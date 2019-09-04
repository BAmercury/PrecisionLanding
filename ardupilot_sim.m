%% Ardupilot Precision Landing EKF Simulation


% Simulate Vehicle Acceleration, Velocity, and Position Profiles

amplitude = 15;
max_velocity = 2.0;
frequency_w = 0.1333;
desired_vel = (-amplitude * frequency_w) * sin(frequency_w*t);
desired_vel = 0.2;


% Acceleration

params = accelparams

% Generate N Samples at a sampling rate Fs, with sinusodial frequency
N = 1000;
Fs = 100;
Fc = 0.25;

t = (0:(1/Fs):((N-1)/Fs)).';
accel = zeros(N,3);
angVel = zeros(N,3);

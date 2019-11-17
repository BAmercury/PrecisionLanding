% Yande Liu's System ID Model for 3DR Iris
% Source: https://etda.libraries.psu.edu/catalog/13071yxl5197
% Model of Vehicle Dynamics and Ardupilot ATT controller using system ID

% System ID was performed using Pilot RC Input data (Desired Attitude encoded as a PWM signal)
% and Vehicle State (Velocity (m/s) and Attitude (rad)

%% Full State Version 

% Px, Py, Vx, Vy, Theta (Pitch, Phi (Roll), Virtual states for ACAH PID Attitude
% Controller

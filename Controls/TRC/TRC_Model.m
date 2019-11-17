% Yande Liu's System ID Model for 3DR Iris and TRC Controller
% Source: https://etda.libraries.psu.edu/catalog/13071yxl5197

%% Setup State Space Model
% Lateral Dynamics
% States are: [lateral velocity, roll angle, integral of moment of rate,
% and integrator compensation for attitude]
Alat = [-0.1180 32.033 0 0;
       0 -6.411 1 0;
       0 -20.639 0 1;
       0 -1.235 0 0];
Blat = [0; 
        0.0039974;
        0.016275;
        0.0040072];
Clat = [0.3048 0 0 0];
Dlat = 0;

sys_lat_vel = ss(Alat, Blat, Clat, Dlat);

%% Lateral Transfer Function
% include the time delay generated by CIFER
lat_time_delay = 0.156; % (in seconds)

% U(s) = desired roll (Radians), Y(s) = Vehicle Velocity
[NUM, DEN] = ss2tf(Alat, Blat, Clat, Dlat);
sys_lat_tf_vel = tf(NUM, DEN, 'InputDelay', lat_time_delay);
g = tf(NUM, DEN);

%%
% U(s) = desired roll (Radians), Y(s) = Vehicle Attitude
C = [0 1 0 0];
[NUMr, DENr] = ss2tf(Alat, Blat, C, Dlat);
sys_lat_tf_roll = tf(NUMr, DENr, 'InputDelay', lat_time_delay);



%% TRC Simulation (Pitch)
Kp = 1.351;
Ki = 0.0095;
g = 32.033;
fl = 1/g;
PWM_deadband = 30;
PWM_max_pitch = 2015;
PWM_min_pitch = 999;
PWM_bias_pitch = 1503;
pitch_max = 30; % degrees
gain_pitch_max = 1/pitch_max;
deg2rad = 57.2958;
Kpwm = 16;
% Command Filter
s = tf('s');
RC = 0.7;
cf_sys = s/(RC*s + 1);
[cf_num, cf_den] = tfdata(cf_sys);


% Yande Liu's System ID Model for 3DR Iris
% Source: https://etda.libraries.psu.edu/catalog/13071yxl5197
% Model of Vehicle Dynamics and Ardupilot ATT controller using system ID

% System ID was performed using Pilot RC Input data (Desired Attitude encoded as a PWM signal)
% and Vehicle State (Velocity (m/s) and Attitude (rad)
%% Setup State Space Model

%{

States that describe drone are: 
    Velocity (X velocity or Y velocity)
    Attitude (Pitch or Roll)
States that are needed to model the onboard attitude (2nd order dynamics)
controller:
    Integration of the moment for rate and rate for attitude
    Integrator Compensation on the attitude



%}




% Lateral Dynamics
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


% Longitudinal Dynamics
Along = [-0.1081 -32.513 0 0;
        0 -6.88 1 0;
        0 -10.931 0 1;
        0 -1.235 0 0];
Blong = [0;
        0.0053854;
        0.0069518;
        0.001443];
Clong = [0.3048 0 0 0];
Dlong = 0;

sys_long_vel = ss(Along, Blong, Clong, Dlong);

% include the time delay generated by CIFER
lat_time_delay = 0.156; % (in seconds)
long_time_delay = 0.175;

%% Lateral Velocity Transfer Function for Velocity

% U(s) = desired roll (Radians), Y(s) = Vehicle Velocity (m/s)
[NUM, DEN] = ss2tf(Alat, Blat, Clat, Dlat);
sys_lat_tf_vel = tf(NUM, DEN, 'InputDelay', lat_time_delay);

%% Longitudinal Velocity Transfer Function for Velocity

% U(s) = desired roll (Radians), Y(s) = Vehicle Velocity (m/s)
[NUML, DENL] = ss2tf(Along, Blong, Clong, Dlong);
sys_long_tf_vel = tf(NUML, DENL, 'InputDelay', long_time_delay);


%%
% Augment model with virtual state for position
% States are: Position, Velocity, Roll, ACAH Virtual State, ACAH Virtual
% State 2
Along_pl = [0 1 0 0 0;
    0 -0.1081 -32.513 0 0;
    0 0 -6.88 1 0;
    0 0 -10.931 0 1;
    0 0 -1.235 0 0];
Blong_pl = [0;
        0;
        0.0053854;
        0.0069518;
        0.001443];
Clong_pl = [0.3048 0 0 0 0]; % Convert to meters
Dlong = 0;
sys_long_pl = ss(Along_pl, Blong_pl, Clong_pl, Dlong);


Q = Clong_pl'*Clong_pl;
R = 1;
K = lqr(Along_pl, Blong_pl, Q, R);
sys_long_pl_c = ss(Along_pl-Blong_pl*K, Blong_pl, Clong_pl, Dlong);
step(sys_long_pl_c)
% Feedforward precompensation
fig1 = figure;
s = size(Along_pl,1);
Z = [zeros([1,s]) 1];
N = inv([Along_pl,Blong_pl;Clong_pl,Dlong])*Z';
Nx = N(1:s);
Nu = N(1+s);
Nbar=Nu + K*Nx;
sys_long_pl_c = ss(Along_pl-Blong_pl*K, Blong_pl*Nbar, Clong_pl, Dlong);
hold on;
step(sys_long_pl_c)


% Lsim
fig2 = figure;
t = 0:0.01:200;
r = 1*ones(size(t));
r = sin(0.001*t);
[y, t, x] = lsim(sys_long_pl_c, r, t);
plot(t, r);
hold on;
plot(t, y);

%%
% Observer
P = [-40 -41 -42 -43 -44];
L = place(Along_pl', Clong_pl', P)';

Ace = [(Along_pl-Blong_pl*K) (Blong_pl*K);
       zeros(size(Along_pl)) (Along_pl-L*Clong_pl)];
Bce = [Blong_pl*Nbar;
       zeros(size(Blong_pl))];
Cce = [Clong_pl zeros(size(Clong_pl))];
Dce = 0;
sys_est_cl = ss(Ace,Bce,Cce,Dce);
step(sys_est_cl)






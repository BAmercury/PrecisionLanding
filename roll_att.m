%% Ardupilot Attitude Controller Models
% Script to develop low fidelty model that will be used to comapre with
% system identification results

s = tf('s');
Ixx = 0.008;

%%
% Linearized about the hover state
% States: [Roll, Roll Rate]
% Generate SISO Transfer function for output Roll Velocity
A = [0 1; 0 0];
B = [0; 1/Ixx];
C = [0 1];
D = 0;
sys = ss(A, B, C, D);
[NUM, DEN] = ss2tf(A,B,C,D);
tf_sys_rate_open_loop = tf(NUM, DEN);
ctrb_result = rank(ctrb(A,B))
% Rank is 2, system is fully controllable
% Open Loop Bode Plot
figure(1);
pzmap(tf_sys_rate_open_loop)
figure(2);
margin(tf_sys_rate_open_loop)
figure(3);
step(tf_sys_rate_open_loop)

% Transfer function for Y(S) = Angle now
C = [1 0];
[NUM_ang, DEN_ang] = ss2tf(A, B, C, D);
tf_sys_angle_open_loop = tf(NUM_ang, DEN_ang);
figure(10);
pzmap(tf_sys_angle_open_loop)
figure(11);
margin(tf_sys_angle_open_loop);
figure(12);
step(tf_sys_angle_open_loop);
%% 
% Roll Velocity PID Gains from Ardupilot
P_RollV = 0.135;
I_RollV = 0.090;
D_RollV = 0.0036;
IMAX_RollV = 0.5;
FILT_RollV = 20;
% P controller Roll from Ardupilot
P_Roll = 4.5;

% P controller for XY
P_XY = 1;
% PID Controller for Linear Velocity
P_Vel = 2;
I_Vel = 1;
D_Vel = 0.5;
IMAX_Vel = 100;

%% Getting transfer functions from Simulink

[A_RollV, B_RollV, C_RollV, D_RollV] = linmod('att_rate_controller');
att_rate_tf = tf(ss(A_RollV, B_RollV, C_RollV, D_RollV));
figure(4);
pzmap(att_rate_tf);
figure(5);
margin(att_rate_tf);
figure(6);
step(att_rate_tf);
bandwidth_roll_rate = bandwidth(att_rate_tf)

%% Now we should add in outer loop 

[A_Roll, B_Roll, C_Roll, D_Roll] = linmod('att_angle_controller');
att_angle_tf = tf(ss(A_Roll, B_Roll, C_Roll, D_Roll));
figure(7);
pzmap(att_angle_tf);
figure(8);
margin(att_angle_tf);
figure(9);
step(att_angle_tf);
bandwidth_angle = bandwidth(att_angle_tf)
%% Now we should add in the translational dynamics and generate new models

% States will now be: [Roll RollV Pos Vel Accel]
A_full = [0 1 0 0; 0 0 0 0; 0 0 0 1; -9.81 0 0 0];
B_full = [0; 1/Ixx; 0; 0];
C_full = [0 1 0 0 ];
D_full = 0;

sys_full = ss(A_full, B_full, C_full, D_full);

%% Get transfer functions from Simulink Model

% Get Position Transfer function
[Af_full, Bf_full, Cf_full, Df_full] = linmod('pos_full_controller');
pos_cntrl_tf = tf(ss(Af_full, Bf_full, Cf_full, Df_full));

% Force some pole zero or near zero cancellations
pos_cntrl_tf_sys = minreal(pos_cntrl_tf, 0.03);

num_poles = length(pole(pos_cntrl_tf_sys))

num_zero = length(zero(pos_cntrl_tf_sys))

% Number of Poles: 6, Number of Zeros: 2
margin(pos_cntrl_tf_sys)
full_system_bandwidth = bandwidth(pos_cntrl_tf_sys)


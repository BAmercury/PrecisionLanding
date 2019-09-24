% Validation Data:
%{
Hover
1 Hz
0.25 hz
4 Hz
Steps (3m, 10s hold)

%}
clear; clc;
%% Variables
Ts_PSC_Y = 10; % Sampling rate for PSC
Ts_PL_Y = 25; % Sampling rate for PL
T_PSC_Y = 1/Ts_PSC_Y; % Sampling Period for PSC
T_PL_Y = 1/Ts_PL_Y; % Sampling Period for PL
% Use these variables to downsample both data sets for 5 Hz sampling rate
cut_PSC = 2; 
cut_PL = 5;
%% Grab Data
[file, path] = uigetfile
load(strcat(path, file));
%%
% Grab POS data to get time stamps to trim where vehicle was at constant
% alt
POS_TIME = POS(:,2);
POS_ALT = POS(:,5);
plot(POS_TIME, POS_ALT);

%%
% Get start and end times (In microseconds)
%start_time_us = 70595839;
%end_time_us = 155096001; 

% Now PSC Y and PL Y data and trim them
PSC_Y_time = PSC(:,2);
PSC_Y = PSC(:,6);
%index1 = find(PSC_Y_time == start_time_us);
%index2 = find(PSC_Y_time == end_time_us);
%PSC_Y_trim = PSC_Y(index1:index2);

PL_u_time = PL(:,2);
PL_u = PL(:,6);
%index1 = find(PL_u_time == start_time_us);
%index2 = find(PL_u_time == 155115998); % Time stamps may not match up
%PL_u_trim = PL_u(index1:index2);

% Downsample data
y = downsample(PSC_Y, cut_PSC);
u = downsample(PL_u, cut_PL);
plot(y)
hold on;
plot(u)
% Form ID Data object
data = iddata(y, u, 0.2, 'OutputName', 'PSC', 'InputName', 'PL', 'OutputUnit', 'm', 'InputUnit', 'm');
% Detrend the data
data_d = detrend(data);
plot(data_d)

%% FFT analysis
% Perform FFT and verify frequencies of interest
fft_u = fft(u);
fft_y = fft(y);
Fs = 5; % 5 Hz Sampling Rate
T = 1/Fs; % Sampling Period
L_u = length(fft_u); % length of signal
L_y = length(fft_y);

% Comptue Single Sided spectrum
P2 = abs(fft_u/L_u);
P1 = P2(1:L_u/2+1);
P1(2:end-1) = 2*P1(2:end-1);

% Define frequency domain f, plot singled sided spectrum P1
f = Fs*(0:(L_u/2))/L_u;
figure(5);
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of Input Data')
xlabel('f (Hz)')
ylabel('|P1(f)|')

% Comptue Single Sided spectrum
P2 = abs(fft_y/L_y);
P1 = P2(1:L_y/2+1);
P1(2:end-1) = 2*P1(2:end-1);

% Define frequency domain f, plot singled sided spectrum P1
f = Fs*(0:(L_y/2))/L_y;
figure(6);
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of Output Data')
xlabel('f (Hz)')
ylabel('|P1(f)|')
%%
% Open system identification tool to finish forming
systemIdentification
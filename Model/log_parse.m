%% Ardupilot Log Data Collection
clear; clc;
[file, path] = uigetfile
load(strcat(path, file));

%% Extract the Position Control (PSC), Precision Landing (PL) Data, as well as Attitude (ATT) Data

% Extract altitude and look at the time stamps to snip the data
% Want to ensure data is coming from aircraft being at level height
% and in the loiter mode (not land or takeoff)
% use the POS struct:

%{
POS Structure:
{'LineNo';
'TimeUS';
'Lat';
'Lng';
'Alt';
'RelHomeAlt';
'RelOriginAlt'}
%}

POS_TIME = POS(:,2);
POS_ALT = POS(:,5);
plot(POS_TIME, POS_ALT)

% Get the indices of the timestamps to trim the data at using data cursor
% tool on the plot
index1 = 198;
index2 = 936;
start_time = 70453778; % Microseconds
end_time = 144253714; % microseconds
%%
% X: 173 to 972 should be good

%{
PSC Log item details

{'LineNo';
'TimeUS';
'TPX';
'TPY';
'PX';
'PY';
'TVX';
'TVY';
'VX';
'VY';
'TAX';
'TAY';
'AX';
'AY'}

%}

out_td_PSC_Y_time = PSC(:,2); % Timestamp data
% Get index values to trim the data based on the desired start and stop
% time stamps
index1 = find(out_td_PSC_Y_time == start_time);
index2 = find(out_td_PSC_Y_time == end_time);
out_td_PSC_Y = PSC(:,6);
% Trim the data
out_td_PSC_Y_trim = out_td_PSC_Y(index1:index2);
out_td_PSC_Y_time_trim = out_td_PSC_Y_time(index1:index2); % Not sure if it'll like this
Ts_PSC_Y = 10; % Sampling rate in Hz

%%
%{
PL Structure
{'LineNo';
'TimeUS';
'Heal';
'TAcq';
'pX';
'pY';
'vX';
'vY';
'bX';
'bY';
'bVX';
'bVY';
'ax';
'ay';
'mx';
'my';
'mz'}
%}

in_td_PL_Y_time = PL(:,2); % Timestamps
% Get index values to trim the data based on the desired start and stop
% time stamps
index1 = find(in_td_PL_Y_time == start_time);
index2 = find(in_td_PL_Y_time == end_time);
in_td_PL_Y = PL(:,6);
in_td_PL_Y_trim = in_td_PL_Y(index1:index2);
in_td_PL_Y_time_trim = in_td_PL_Y_time(index1:index2);
Ts_PL_Y = 25; % Sampling rate, 1/S (Hz)
%% Create a SamplingInstants object of the time data
% 1 By N cell array where N is the number of experiments

timedata = {out_td_PSC_Y_time_trim, in_td_PL_Y_time_trim};

%%
% Form ID data object
cut_PSC = 5;
cut_PL = 2;
% Downsample both data sets to get same sampling rate (5 Hz)
y = downsample(out_td_PSC_Y, 2);
u = downsample(in_td_PL_Y, 5);
plot(y)
hold on;
plot(u)

data = iddata(y, u, 0.2, 'OutputName', 'PSC', 'InputName', 'PL', 'OutputUnit', 'm', 'InputUnit', 'm');
% Detrend the data
data_d = detrend(data)
% now keep data where drone was at constant hover altitude
% Convert out start time and end time to seconds
start_time_s = start_time / 1000000;
end_time_s = end_time / 1000000;

%% Grab Validation data (0.25 Hz, in phase)
[file, path] = uigetfile
load(strcat(path, file));
out_td_PSC_Y_time_valid = PSC(:,2); % Timestamp data
out_td_PSC_Y_valid = PSC(:,6);
in_td_PL_Y_time_valid = PL(:,2); % Timestamps
in_td_PL_Y_valid = PL(:,6);
y = downsample(out_td_PSC_Y_valid, 2);
u = downsample(in_td_PL_Y_valid, 5);
data_valid = iddata(y, u, 0.2, 'OutputName', 'PSC', 'InputName', 'PL', 'OutputUnit', 'm', 'InputUnit', 'm');
% Remove trends
data_valid_d = detrend(data_valid);
%% Now open the SystemID tool
systemIdentification
% 5 poles, 4 zeros maybe?


% Ardupilot Kalman Filter Target Tracking Implementation Study
clc; clear;
[file, path] = uigetfile
data = load(strcat(path, file));

% Extract the NTUN and Precision Landing Data


%{
    NTUN: Navigation Tuning (meters)
    Desired: Global target data in NED
    Other Data: global Vehicle data in NED
    Struct:
    time (microseconds)
    unknown
    DPosX
    DPosY
    PosX
    PosY
    DVelX
    DVelY
    VelX
    VelY
    DAccX
    DAccY
%}
NTUN = data.NTUN;

%{
    PL: Precision Landing
    Position/Vel relative to vehicle (between target and vehicle)
    Struct:
    TimeUS,Heal,TAcq,pX,pY,vX,vY

    time (microseconds)
    healthy
    target acquired
    PosX (cm)
    PosY
    VelX
    VelY
%}
PL = data.PL;


POS = data.POS;






% HW TEST Code
% 2024-08-12 / Controla Project
% Made by NDH & LJC

% Initialize Matlab
clc;
clear all;
close all;

% Add Library directory
addpath(genpath('C:\Users\DDHD\Desktop\HW_work2'));

% Main
deviceAddress = '192.168.0.101';
userName = 'jetson1';
password = 'rkd2233';

% Jetson Orin Nano
hwobj = jetson( deviceAddress, userName, password );

% Socket setting
commandPort = 12346; % Jetson1 Port
host = '192.168.0.101';

% Relay pin -  initialize
relayPins = [ 37, 35, 31, 29, 23, 21, 19, 13 ];

% % Warm up Part
client = tcpclient( host, commandPort );

while true
    
    disp(" MOVE (Body Frame) : ")
    disp(" CW ")
    disp(" R ")
    pause(1)
    sendRelayCommand(client, [ 1, 0, 1, 0, 1, 0, 1, 0 ])
    pause(1)
    sendRelayCommand(client, [ 1, 1, 1, 1, 1, 1, 1, 1 ])
    pause(10)
    % 
    % disp(" MOVE (Body Frame) : ")
    % disp(" CCW ")
    % disp(" -R ")
    % sendRelayCommand(client, [ 0, 1, 0, 1, 0, 1, 0, 1 ])
    % pause(1)

end
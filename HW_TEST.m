% HW TEST Code
% 2024-08-12 / Controla Project
% Made by NDH & LJC

% Initialize Matlab
clc;
clear all;
close all;

% Add Library directory
addpath(genpath('C:\Users\DDHD\Desktop\shj\main'));

% Main
deviceAddress = '192.168.0.101';
userName = 'jetson1';
password = 'rkd2233';

% Jetson Orin Nano
hwobj = jetson( deviceAddress, userName, password );

% Socket setting
commandPort = 54322; % Jetson1 Port
host = '192.168.0.101';

% Relay pin -  initialize
% relayPins = [ 37, 35, 31, 29, 23, 21, 19, 13 ];
relayPins = [ 13, 19, 21, 23, 29, 31, 35, 37 ];


% % Warm up Part
client = tcpclient( host, commandPort );
dlt = 1;
dlt2 = 1;

while true

    pause(dlt2)
    disp(" Thruster : ")
    disp(" No.1")
    sendRelayCommand(client, [ 0, 1, 1, 1, 1, 1, 1, 1 ])
    pause(dlt)

    pause(dlt2)
    disp(" Thruster : ")
    disp(" No.2 ")
    sendRelayCommand(client, [ 1, 0, 1, 1, 1, 1, 1, 1 ])
    pause(dlt)

    pause(dlt2)
    disp(" Thruster : ")
    disp(" No.3 ")
    sendRelayCommand(client, [ 1, 1, 0, 1, 1, 1, 1, 1 ])
    pause(dlt)

    pause(dlt2)
    disp(" Thruster : ")
    disp(" No.4 ")
    sendRelayCommand(client, [ 1, 1, 1, 0, 1, 1, 1, 1 ])
    pause(dlt)

    pause(dlt2)
    disp(" Thruster : ")
    disp(" No.5 ")
    sendRelayCommand(client, [ 1, 1, 1, 1, 0, 1, 1, 1 ])
    pause(dlt)

    pause(dlt2)
    disp(" Thruster : ")
    disp(" No.6 ")
    sendRelayCommand(client, [ 1, 1, 1, 1, 1, 0, 1, 1 ])
    pause(dlt)

    pause(dlt2)
    disp(" Thruster : ")
    disp(" No.7 ")
    sendRelayCommand(client, [ 1, 1, 1, 1, 1, 1, 0, 1 ])
    pause(dlt)

    pause(dlt2)
    disp(" Thruster : ")
    disp(" No.8 ")
    sendRelayCommand(client, [ 1, 1, 1, 1, 1, 1, 1, 0 ])
    pause(dlt)
    % sendRelayCommand(client, [ 1, 1, 1, 1, 1, 1, 1, 1 ])
    % pause(dlt2)
    % disp(" Thruster : ")
    % disp(" No.1")
    % sendRelayCommand(client, [ 0, 1, 1, 1, 1, 1, 1, 1 ])
    % pause(dlt)
    % 
    % pause(dlt2)
    % disp(" Thruster : ")
    % disp(" No.2 ")
    % sendRelayCommand(client, [ 1, 0, 1, 1, 1, 1, 1, 1 ])
    % pause(dlt)
    % 
    % pause(dlt2)
    % disp(" Thruster : ")
    % disp(" No.3 ")
    % sendRelayCommand(client, [ 1, 1, 0, 1, 1, 1, 1, 1 ])
    % pause(dlt)
    % 
    % pause(dlt2)
    % disp(" Thruster : ")
    % disp(" No.4 ")
    % sendRelayCommand(client, [ 1, 1, 1, 0, 1, 1, 1, 1 ])
    % pause(dlt)
    % 
    % pause(dlt2)
    % disp(" Thruster : ")
    % disp(" No.5 ")
    % sendRelayCommand(client, [ 1, 1, 1, 1, 0, 1, 1, 1 ])
    % pause(dlt)
    % 
    % pause(dlt2)
    % disp(" Thruster : ")
    % disp(" No.6 ")
    % sendRelayCommand(client, [ 1, 1, 1, 1, 1, 0, 1, 1 ])
    % pause(dlt)
    % 
    % pause(dlt2)
    % disp(" Thruster : ")
    % disp(" No.7 ")
    % sendRelayCommand(client, [ 1, 1, 1, 1, 1, 1, 0, 1 ])
    % pause(dlt)
    % 
    % pause(dlt2)
    % disp(" Thruster : ")
    % disp(" No.8 ")
    % sendRelayCommand(client, [ 1, 1, 1, 1, 1, 1, 1, 0 ])
    % pause(dlt)
    sendRelayCommand(client, [ 1, 1, 1, 1, 1, 1, 1, 1 ])
    pause(dlt2)
    disp("3")
    pause(dlt2)
    disp("2")
    pause(dlt2)
    disp("1")
    pause(dlt2)
    disp(" MOVE (Body Frame) : ")
    disp(" Forward ")
    disp(" Y ")
    sendRelayCommand(client, [ 1, 1, 1, 0, 0, 1, 1, 1 ])
    pause(dlt)
    disp(" 10sec ")
    pause(dlt)
    disp(" Stop ")
    sendRelayCommand(client, [ 1, 1, 1, 1, 1, 1, 1, 1 ])

    % pause(dlt2)
    % disp(" MOVE (Body Frame) : ")
    % disp(" Backward ")
    % disp(" -Y ")
    % sendRelayCommand(client, [ 0, 1, 1, 1, 1, 1, 1, 0 ])
    % pause(dlt)
    % 
    % pause(dlt2)
    % disp(" MOVE (Body Frame) : ")
    % disp(" Right ")
    % disp(" X ")
    % sendRelayCommand(client, [ 1, 1, 1, 1, 1, 0, 0, 1 ])
    % pause(dlt)
    % 
    % pause(dlt2)
    % disp(" MOVE (Body Frame) : ")
    % disp(" Left ")
    % disp(" -X ")
    % sendRelayCommand(client, [ 1, 0, 0, 1, 1, 1, 1, 1 ])
    % pause(dlt)
    % 
    % pause(dlt2)
    % disp(" MOVE (Body Frame) : ")
    % disp(" CW ")
    % disp(" R ")
    % sendRelayCommand(client, [ 1, 0, 1, 0, 1, 0, 1, 0 ])
    % pause(dlt)
    % 
    % pause(dlt2)
    % disp(" MOVE (Body Frame) : ")
    % disp(" CCW ")
    % disp(" -R ")
    % sendRelayCommand(client, [ 0, 1, 0, 1, 0, 1, 0, 1 ])
    % pause(dlt)

end
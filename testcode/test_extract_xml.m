clear all
close all
clc

addpath(genpath('C:\Users\USER\Desktop\hanul_GNC'));

xmlFilePath = 'rocket.xml';
hsr_1 = extractRocketParams(xmlFilePath);
param2rocket_3dplot(hsr_1);

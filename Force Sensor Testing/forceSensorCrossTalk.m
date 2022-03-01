%% Force Sensor Crosstalk

%% Clear
clear; clc;
close all;
instrreset;

%% Import data
fxTestRaw = readmatrix('PFH0A150_SENSOR_1_13_16_52.txt');
fyTestRaw = readmatrix('PFH0A150_SENSOR_1_13_21_41.txt');
fzTestRaw = readmatrix('PFH0A150_SENSOR_1_12_22_07.txt');

%% Variables
totalLoad = 76.881; %N

%% Create matrix
fTestCtsM = [mean(fxTestRaw(:,3)), mean(fxTestRaw(:,4)), mean(fxTestRaw(:,5));...
             mean(fyTestRaw(:,3)), mean(fyTestRaw(:,4)), mean(fyTestRaw(:,5));...
             mean(fzTestRaw(:,3)), mean(fzTestRaw(:,4)), mean(fzTestRaw(:,5))];
         
fTestPerform = [(fTestCtsM(1,:)/fTestCtsM(1,1))*100;...
                (fTestCtsM(2,:)/fTestCtsM(2,2))*100;...
                (fTestCtsM(3,:)/fTestCtsM(3,3))*100];

fTestCtsPLM = fTestCtsM/totalLoad;

fTestCtsPLM_I = inv(fTestCtsPLM);

%% 
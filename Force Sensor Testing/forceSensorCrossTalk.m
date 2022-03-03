%% Force Sensor Crosstalk
% https://hitec.humaneticsgroup.com/perspectives/cross-talk-compensation-using-matrix-methods

%% Clear
clear; clc;
close all;

%% Import data
negXRaw = readmatrix('negX922kg_SENSOR_1_16_01_22.txt');
negYRaw = readmatrix('negY922kg_SENSOR_1_15_58_54.txt');
posXRaw = readmatrix('PosX922kg_SENSOR_1_15_55_47.txt');
posYRaw = readmatrix('posY922kg_SENSOR_1_15_52_17.txt');
tensionRaw = readmatrix('tension922kg_SENSOR_1_15_46_02.txt');

%% Variables
totalLoad = 9.22*9.81; % 90.4482 N

%% +X,+Y,Z Matrix
pXpYCntMeans = [mean(posXRaw(:,3)), mean(posXRaw(:,4)), mean(posXRaw(:,5));...
                mean(posYRaw(:,3)), mean(posYRaw(:,4)), mean(posYRaw(:,5));...
                mean(tensionRaw(:,3)), mean(tensionRaw(:,4)), mean(tensionRaw(:,5))];
         
pXpYPerformanceMatrix = [(pXpYCntMeans(1,:)/pXpYCntMeans(1,1))*100;...
                         (pXpYCntMeans(2,:)/pXpYCntMeans(2,2))*100;...
                         (pXpYCntMeans(3,:)/pXpYCntMeans(3,3))*100];

pXpYKmatrx = pXpYCntMeans/totalLoad;

pXpYKmatrxI = inv(pXpYKmatrx);

%% +X,-Y,Z Matrix
% pXnYCntMeans = [mean(posXRaw(:,3)), mean(posXRaw(:,4)), mean(posXRaw(:,5));...
%                 mean(negYRaw(:,3)), mean(negYRaw(:,4)), mean(negYRaw(:,5));...
%                 mean(tensionRaw(:,3)), mean(tensionRaw(:,4)), mean(tensionRaw(:,5))];
%          
% pXnYPerformanceMatrix = [(pXnYCntMeans(1,:)/pXnYCntMeans(1,1))*100;...
%                          (pXnYCntMeans(2,:)/pXnYCntMeans(2,2))*100;...
%                          (pXnYCntMeans(3,:)/pXnYCntMeans(3,3))*100];
% 
% pXnYKmatrx = pXnYCntMeans/totalLoad;
% 
% pXnYKmatrxI = inv(pXnYKmatrx);

%% -X,-Y,Z Matrix
nXnYCntMeans = [mean(negXRaw(:,3)), mean(negXRaw(:,4)), mean(negXRaw(:,5));...
                mean(negYRaw(:,3)), mean(negYRaw(:,4)), mean(negYRaw(:,5));...
                mean(tensionRaw(:,3)), mean(tensionRaw(:,4)), mean(tensionRaw(:,5))];
         
nXnYPerformanceMatrix = [(nXnYCntMeans(1,:)/nXnYCntMeans(1,1))*100;...
                         (nXnYCntMeans(2,:)/nXnYCntMeans(2,2))*100;...
                         (nXnYCntMeans(3,:)/nXnYCntMeans(3,3))*100];

nXnYKmatrx = nXnYCntMeans/-totalLoad;

nXnYKmatrxI = inv(nXnYKmatrx);

%% -X,-Y,Z Matrix
% nXpYCntMeans = [mean(negXRaw(:,3)), mean(negXRaw(:,4)), mean(negXRaw(:,5));...
%                 mean(posYRaw(:,3)), mean(posYRaw(:,4)), mean(posYRaw(:,5));...
%                 mean(tensionRaw(:,3)), mean(tensionRaw(:,4)), mean(tensionRaw(:,5))];
%          
% nXpYPerformanceMatrix = [(nXpYCntMeans(1,:)/nXpYCntMeans(1,1))*100;...
%                          (nXpYCntMeans(2,:)/nXpYCntMeans(2,2))*100;...
%                          (nXpYCntMeans(3,:)/nXpYCntMeans(3,3))*100];
% 
% nXpYKmatrx = nXpYCntMeans/totalLoad;
% 
% nXpYKmatrxI = inv(nXpYKmatrx);

%% Inverse Matrix Found
% Factory:
%                       [   0.0496,  0.0000,  0.0000]
% [SM] = SenseMatrix  = [   0.0000,  0.0494,  0.0000] (N/cts)
%                       [   0.0000,  0.0000,  0.6231]
%
% +X and +Y Calculated:
%                       [  0.0474, -0.0042, -0.0581]
% [SM] = SensorMatrix = [ -0.0018,  0.0464, -0.0530] (N/cts) 
%                       [  0.0168,  0.0152,  0.5850]
% 
% -X and -Y Calculated:
%                       [   0.0490, -0.0002, -0.0179]
% [SM] = SensorMatrix = [ -0.00002,  0.0508, -0.0137] (N/cts) 
%                       [   0.0181,  0.0183, -0.6371]
%
% Example to code:
% if cts = sensor count readings:
% Fx = SM(1,1)*ctsX + SM(1,2)*ctsY + SM(1,3)*ctsZ
% Fy = SM(2,1)*ctsX + SM(2,2)*ctsY + SM(2,3)*ctsZ
% Fz = SM(3,1)*ctsX + SM(3,2)*ctsY + SM(3,3)*ctsZ




%% Force Sensor Crosstalk
% https://hitec.humaneticsgroup.com/perspectives/cross-talk-compensation-using-matrix-methods

%% Clear
clear; clc;
close all;

%% Import data
meanXYZ.x1 = (mean(readmatrix('posX1_SENSOR_1_16_02_47.txt')));
meanXYZ.x2 = (mean(readmatrix('posX2_SENSOR_1_16_02_57.txt')));
meanXYZ.x3 = -(mean(readmatrix('negX1_SENSOR_1_16_08_24.txt')));
meanXYZ.x4 = -(mean(readmatrix('negX2_SENSOR_1_16_08_33.txt')));
meanXYZ.y1 = (mean(readmatrix('posY1_SENSOR_1_16_05_41.txt')));
meanXYZ.y2 = (mean(readmatrix('posY2_SENSOR_1_16_05_51.txt')));
meanXYZ.y3 = -(mean(readmatrix('negY1_SENSOR_1_15_59_28.txt')));
meanXYZ.y4 = -(mean(readmatrix('negY2_SENSOR_1_15_59_48.txt')));
meanXYZ.z1 = (mean(readmatrix('posZ1_SENSOR_1_16_14_01.txt')));
meanXYZ.z2 = (mean(readmatrix('posZ2_SENSOR_1_16_14_14.txt')));
meanXYZ.z3 = -(mean(readmatrix('negZ1_SENSOR_1_16_18_46.txt')));
meanXYZ.z4 = -(mean(readmatrix('negZ2_SENSOR_1_16_18_56.txt')));
meanXYZ.z5 = -(mean(readmatrix('negZ3_SENSOR_1_16_21_13.txt')));
meanXYZ.z6 = -(mean(readmatrix('negZ4_SENSOR_1_16_22_21.txt')));

%% Variables
totalLoad = 9.22*9.81; % 90.4482 N
totalM = 4 * 4 * 6;
num = 0;
invM = zeros(3,3,totalM);

%% Find sensitivity matrx
for z = 1:6
   for x = 1:4
      for y = 1:4
          num = num + 1;
          temp = [meanXYZ.(['x',num2str(x)]); meanXYZ.(['y',num2str(y)]); meanXYZ.(['z',num2str(z)])]/totalLoad;
          invM(:,:,num) = inv(temp);
      end
   end
end
meanInvM = [mean(invM(1,1,:)), mean(invM(1,2,:)), mean(invM(1,3,:));
            mean(invM(2,1,:)), mean(invM(2,2,:)), mean(invM(2,3,:));
            mean(invM(3,1,:)), mean(invM(3,2,:)), mean(invM(3,3,:))];
stdInvM = [std(invM(1,1,:)), std(invM(1,2,:)), std(invM(1,3,:));
           std(invM(2,1,:)), std(invM(2,2,:)), std(invM(2,3,:));
           std(invM(3,1,:)), std(invM(3,2,:)), std(invM(3,3,:))];

%% Inverse Matrix Found
% Factory [M]:
%                       [   0.0496,  0.0000,  0.0000]
% [SM] = SenseMatrix  = [   0.0000,  0.0494,  0.0000] (N/cts)
%                       [   0.0000,  0.0000,  0.6231]
%
% Calculated [M]:
%                       [  0.0526, -0.0011, -0.0287]
% [SM] = SensorMatrix = [  0.0002,  0.0531, -0.0286] (N/cts) 
%                       [ -0.0264, -0.0103,  0.7597]
%
% Example to code:
% if cts = sensor count readings:
% Fx = SM(1,1)*ctsX + SM(1,2)*ctsY + SM(1,3)*ctsZ
% Fy = SM(2,1)*ctsX + SM(2,2)*ctsY + SM(2,3)*ctsZ
% Fz = SM(3,1)*ctsX + SM(3,2)*ctsY + SM(3,3)*ctsZ




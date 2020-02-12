%% Serial to CSV script
% This code takes in the serial data being sent from the dynamixel board to
% logs it to a csv file. 
% 
% Script by erick nunez

%% Clean workspace
clear; clc; delete(instrfindall);
%% CSV file
fileTime = datestr(now,'mmddyyHHMM');
csvFile = ['Logs/armSupportLog',fileTime,'.csv'];
%% Sets up duration of test
secs = input('Collection time duration in Secs? ');
%% Sets up serial object
s1 = serial('COM24');
s1.BaudRate = 115200;
%% Waits for data
disp('........opening port...........');
fopen(s1);
s1.ReadAsyncMode = 'continuous';
readasync(s1);
while(s1.BytesAvailable < 0)
   disp('Waiting for data...............');
end
%% Starts reading data
% Adjust the data varaible to match what is being sent from the controller.
dt = 0.008;
length = secs/dt; % seconds*(1frame/secs)=frames
data = zeros(length,24);
for i=1:length
    serialData = fscanf(s1);
    temp = strsplit(serialData,'\t');
    time = 0.001*str2double(temp(1));
    force = str2double(temp([2, 3, 4]));       % Forces
    presQ = str2double(temp([5, 6, 7]));       % init Angle
    presQdot = str2double(temp([8, 9, 10]));   % init Ang Vel
    goalM = str2double(temp([11, 12, 13]));    % goal Pos
    goalMdot = str2double(temp([14, 15, 16])); % goal Vel
    goalQ = str2double(temp([17, 18, 19]));    % goal Ang
    goalQdot = str2double(temp([20, 21, 22])); % goal Ang Vel
    loopTime = str2double(temp(24));    % Loop time
    mWrite = str2double(temp(23));      % Write
    data(i,:)=[time,force,presQ,presQdot,goalM,goalMdot,goalQ,goalQdot,mWrite,loopTime];
end
%% post cleanup
fclose(s1);delete(s1);
disp('.........Serial Connection close...............')
%% write data to file
disp('Writing file............................')
writematrix(data,csvFile)
%% plot data
close all
fig1 = figure;
set(fig1, 'Units', 'Normalized','OuterPosition', [0 ,0, 1, 1]);
subplot(2,2,1)
plot(data(:,1),data(:,[5 17])); grid on;
legend('Pres','Goal');title('Shoulder Position');xlabel('Time (sec)');ylabel('Angle (rad)');
subplot(2,2,3)
plot(data(:,1),data(:,[7 19])); grid on;
legend('Pres','Goal');title('Elbow Position');xlabel('Time (sec)');ylabel('Angle (rad)');
subplot(2,2,2)
plot(data(:,1),data(:,[8 20])); grid on;
legend('Pres','Goal');title('Shoulder Velocity');xlabel('Time (sec)');ylabel('Angular Velocity (rad/sec)');
subplot(2,2,4)
plot(data(:,1),data(:,[10 22])); grid on;
legend('Pres','Goal');title('Elbow Velocity');xlabel('Time (sec)');ylabel('Angular Velocity (rad/sec)');

fig2 = figure;
set(fig2, 'Units', 'Normalized','OuterPosition', [1, 0, 1, 1]);
subplot(2,2,1)
plot(data(:,1),data(:,24)); grid on; ylim([0,10])
title('Loop Time'); xlabel('Time (sec)'); ylabel('Looptime (sec)');
subplot(2,2,2)
plot(data(:,1),data(:,[2 3])); grid on;
legend('X','Y');title('Global Forces'); xlabel('Time (sec)'); ylabel('Force (N)');
subplot(2,2,3)
quiver(data(:,11),data(:,12),data(:,14),data(:,15)); grid on; xlim([-1.2,1.2]);ylim([-1.2, 1.2])
title('Mass of Model'); xlabel('X (m)'); ylabel('Y (m)');
subplot(2,2,4)
plot(data(:,1),data(:,[11 12])); grid on;
legend('X','Y');title('Task Space X-Y'); xlabel('Time (sec)'); ylabel('Position (m)');

%% saves plots
set(fig1, 'PaperOrientation', 'landscape', 'PaperUnits', 'normalized', 'PaperPosition',[0,0,1,1]);
saveas(fig1, ['./Logs/armSupport',fileTime,'_1of2.pdf']);
set(fig2, 'PaperOrientation', 'landscape', 'PaperUnits', 'normalized', 'PaperPosition',[0,0,1,1]);
saveas(fig2, ['./Logs/armSupport',fileTime,'_2of2.pdf']);
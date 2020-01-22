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
length = secs*100; % seconds*100=frames
data = zeros(length,17);
for i=1:length
    serialData = fscanf(s1);
    temp = strsplit(serialData,'\t');
    time = 0.001*str2double(temp(1));
    forceX = str2double(temp(2));       forceY = str2double(temp(3));       % Forces
    presQS = str2double(temp(4));       presQE = str2double(temp(5));       % init Angle
    presQdotS = str2double(temp(6));    presQdotE = str2double(temp(7));    % init Ang Vel
    goalX = str2double(temp(8));        goalY = str2double(temp(9));        % goal Pos
    goalXdot = str2double(temp(10));    goalYdot = str2double(temp(11));    % goal Vel
    goalQS = str2double(temp(12));      goalQE = str2double(temp(13));      % goal Ang
    goalQdotS = str2double(temp(14));   goalQdotE = str2double(temp(15));   % goal Ang Vel
    loopTime = str2double(temp(17));    % Loop time
    mWrite = str2double(temp(16));      % Write
    data(i,:)=[time,forceX,forceY,presQS,presQE,presQdotS,presQdotE,goalX,goalY,goalXdot,goalYdot,goalQS,goalQE,goalQdotS,goalQdotE,mWrite,loopTime];
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
set(fig1, 'Units', 'Normalized','OuterPosition', [1,0, 0.5, 1]);
subplot(2,1,1)
plot(data(:,1),data(:,[4 12])); grid on;
legend('Pres','Goal');title('Shoulder Position');xlabel('Time (sec)');ylabel('Angle (rad)');
subplot(2,1,2)
plot(data(:,1),data(:,[5 13])); grid on;
legend('Pres','Goal');title('Elbow Position');xlabel('Time (sec)');ylabel('Angle (rad)');
fig2 = figure;
set(fig2, 'Units', 'Normalized','OuterPosition', [1.5,0, 0.5, 1]);
subplot(2,1,1)
plot(data(:,1),data(:,[6 14])); grid on;
legend('Pres','Goal');title('Shoulder Velocity');xlabel('Time (sec)');ylabel('Angular Velocity (rad/sec)');
subplot(2,1,2)
plot(data(:,1),data(:,[7 15])); grid on;
legend('Pres','Goal');title('Elbow Velocity');xlabel('Time (sec)');ylabel('Angular Velocity (rad/sec)');
fig3 = figure;
set(fig3,'Units','Normalized', 'OuterPosition',[0.5,0.5,0.5,0.5]);
plot(data(:,1),data(:,17)); grid on; ylim([0,10])
title('Loop Time'); xlabel('Time (sec)'); ylabel('Looptime (sec)');
fig4 = figure;
set(fig4,'Units','Normalized', 'OuterPosition',[0.5,0,0.5,0.5]);
plot(data(:,1),data(:,[2 3])); grid on;
legend('X','Y');title('Global Forces'); xlabel('Time (sec)'); ylabel('Force (N)');
fig5 = figure;
set(fig5,'Units','Normalized', 'OuterPosition',[0,0.5,0.5,0.5]);
quiver(data(:,8),data(:,9),data(:,10),data(:,11)); grid on; xlim([-1.2,1.2]);ylim([-1.2, 1.2])
title('Mass of Model'); xlabel('X (m)'); ylabel('Y (m)');

%% saves plots
set(fig1, 'PaperOrientation', 'landscape', 'PaperUnits', 'normalized', 'PaperPosition',[0,0,1,1]);
saveas(fig1, ['./Logs/armSupport',fileTime,'Pos.pdf']);
set(fig2, 'PaperOrientation', 'landscape', 'PaperUnits', 'normalized', 'PaperPosition',[0,0,1,1]);
saveas(fig2, ['./Logs/armSupport',fileTime,'Vel.pdf']);
%% Serial to CSV script
% This code takes in the serial data being sent from the dynamixel board to
% logs it to a csv file.
%
% Script by erick nunez

%% Clean workspace
clear; clc; close all; delete(instrfindall);
%% CSV file
timeInSecs = 30;
logFileID = '_EN_TEST_Ex10';
fileTime = datestr(now,'mmddyyHHMM');
csvFile = ['./Logs/armSupportLog',logFileID,fileTime,'.csv'];
pause(2)

%% Modify mass and damping setup
Mxy = 1.0; % kg
Mz = 1.0; % kg
Bxy = 5.0; % N*(sec/m)
Bz = 1.0; % N*(sec/m)
scalingFactor = 0.10;
eFx = 0.0; % N
eFy = 0.0; % N
eFz = 0.0; % N
bytesMxy = typecast(int32(Mxy*10000),'uint8');
bytesMz = typecast(int32(Mz*10000),'uint8');
bytesBxy1 = typecast(int32(Bxy*10000),'uint8');
bytesBz = typecast(int32(Bz*10000),'uint8');
bytesFactor = typecast(int32(scalingFactor*10000),'uint8');
bytesFx = typecast(int32(eFx*10000),'uint8');
bytesFy = typecast(int32(eFy*10000),'uint8');
bytesFz = typecast(int32(eFz*10000),'uint8');
header = uint8([150, 10, 10, 96]);
modByte = uint8(16);
writePacket = [header,modByte,bytesMxy,bytesMz,bytesBxy1,bytesBz,bytesFactor,bytesFx,bytesFy,bytesFz];
checkSum = sum(writePacket);
csHi = uint8(floor(checkSum/256));
csLo = uint8(mod(checkSum,256));
writePacket = [writePacket,csHi,csLo];
writePacketLen = length(writePacket);
packetSent = false;

%% Sets up and open serial object
BaudRate = 115200;
packetLen = 98;
dt = 0.008;
numFrames = timeInSecs/dt; % seconds*(1frame/secs)=frames
rawData = nan(numFrames,packetLen);
data = nan(numFrames,23);

%% Open Serial Port
disp('........opening port...........');
s1 = serialport('COM24',BaudRate);

%% start main collection loop
totalTime = 0; i = 1;
while(totalTime < timeInSecs )
    if ~packetSent && i > 1
        packetSent = true;
        write(s1,writePacket,'uint8');
        disp('.....sending config')
    end
    if s1.NumBytesAvailable >= packetLen
        rawData(i,:) = read(s1,packetLen,'uint8');
        % Total Time
        data(i,1) = typecast(uint8(rawData(i,5:8)),'uint32');
        % XYZ Global Forces
        data(i,2) = double(typecast(uint8(rawData(i,9:12)),'int32'))/10000;
        data(i,3) = double(typecast(uint8(rawData(i,13:16)),'int32'))/10000;
        data(i,4) = double(typecast(uint8(rawData(i,17:20)),'int32'))/10000;
        % XYZ Bot Goal
        data(i,5) = double(typecast(uint8(rawData(i,21:24)),'int32'))/10000;
        data(i,6) = double(typecast(uint8(rawData(i,25:28)),'int32'))/10000;
        data(i,7) = double(typecast(uint8(rawData(i,29:32)),'int32'))/10000;
        % XYZ Dot Bot Goal
        data(i,8) = double(typecast(uint8(rawData(i,33:36)),'int32'))/10000;
        data(i,9) = double(typecast(uint8(rawData(i,37:40)),'int32'))/10000;
        data(i,10) = double(typecast(uint8(rawData(i,41:44)),'int32'))/10000;
        % Pres Q
        data(i,11) = double(typecast(uint8(rawData(i,45:48)),'int32'))/10000;
        data(i,12) = double(typecast(uint8(rawData(i,49:52)),'int32'))/10000;
        data(i,13) = double(typecast(uint8(rawData(i,53:56)),'int32'))/10000;
        % Goal Q
        data(i,14) = double(typecast(uint8(rawData(i,57:60)),'int32'))/10000;
        data(i,15) = double(typecast(uint8(rawData(i,61:64)),'int32'))/10000;
        data(i,16) = double(typecast(uint8(rawData(i,65:68)),'int32'))/10000;
        % Spring Force and Damping
        data(i,17) = double(typecast(uint8(rawData(i,69:72)),'int32'))/10000;
        data(i,18) = double(typecast(uint8(rawData(i,73:76)),'int32'))/10000;
        data(i,19) = double(typecast(uint8(rawData(i,77:80)),'int32'))/10000;
        data(i,20) = double(typecast(uint8(rawData(i,81:84)),'int32'))/10000;
        % Other Data
        data(i,21) = double(typecast(uint8(rawData(i,85:88)),'int32'))/10000;
        data(i,22) = double(typecast(uint8(rawData(i,89:92)),'int32'))/10000;
        % Loop time
        data(i,23) = typecast(uint8(rawData(i,93:96)),'uint32');
        % Loop Data
        i = i + 1;
        totalTime = totalTime + dt;
    end
end
data(:,1) = data(:,1)*0.001;

%% post cleanup
delete(s1);
disp('.........Serial Connection close...............')

%% plot data
fig1 = figure;
set(fig1, 'Units', 'Normalized','OuterPosition', [0 ,0, 1, 1]);

subplot(4,6,1)
q1PosPres = plot(data(:,1),data(:,11),'LineWidth',1.5,'Color',[0.00,0.45,0.74]);
hold on; grid on; xlim([0,timeInSecs]);
q1PosGoal = plot(data(:,1),data(:,14),'LineWidth',1.5,'Color',[0.85,0.33,0.10]);
legend({'Pres','Goal'},'Location','best');title('Shoulder Position');
legend('boxoff'); xlabel('Time (sec)');ylabel('Angle (rad)');

subplot(4,6,2)
q2PosPres = plot(data(:,1),data(:,12),'LineWidth',1.5,'Color',[0.00,0.45,0.74]);
hold on; grid on; xlim([0,timeInSecs]);
q2PosGoal = plot(data(:,1),data(:,15),'LineWidth',1.5,'Color',[0.85,0.33,0.10]);
legend({'Pres','Goal'},'Location','best');title('Elevation Position');
legend('boxoff'); xlabel('Time (sec)');ylabel('Angle (rad)');

subplot(4,6,3)
q3PosPres = plot(data(:,1),data(:,13),'LineWidth',1.5,'Color',[0.00,0.45,0.74]);
hold on; grid on; xlim([0,timeInSecs]);
q3PosGoal = plot(data(:,1),data(:,16),'LineWidth',1.5,'Color',[0.85,0.33,0.10]);
legend({'Pres','Goal'},'Location','best');title('Elbow Position');
legend('boxoff'); xlabel('Time (sec)');ylabel('Angle (rad)');

% subplot(4,6,[19,21])
% yyaxis left
% dampingPlot = plot(data(:,1),data(:,20),'LineWidth',1.5,'Color',[0.00,0.45,0.74]);
% hold on; grid on; xlim([0,timeInSecs]); ylim([Bxy-5,Bxy2+5]); ylabel('Damping XY');
% yyaxis right
% dampingPlotY = plot(data(:,1),data(:,5),'LineWidth',1.5,'Color',[0.85,0.33,0.10]);
% legend('Damping','X');title('Damping Regions');
% xlabel('Time (sec)'); ylabel('X (m)')

subplot(4,6,[4,12])
fPlotX = plot(data(:,1),data(:,2),'LineWidth',1.5,'Color',[0.00,0.45,0.74]);
hold on; grid on; xlim([0,timeInSecs]);
fPlotY = plot(data(:,1),data(:,3),'LineWidth',1.5,'Color',[0.85,0.33,0.10]);
fPlotZ = plot(data(:,1),data(:,4),'LineWidth',1.5,'Color',[0.93,0.69,0.13]);
legend('X','Y','Z');title('Global Forces');
xlabel('Time (sec)'); ylabel('Force (N)');

subplot(4,6,[7,15])
yyaxis left
zPlotU = plot(data(:,1),data(:,4),'LineWidth',1.5,'Color',[0.00,0.45,0.74]);
hold on; grid on; xlim([0,timeInSecs]);
zPlotS = plot(data(:,1),data(:,21),'LineWidth',1.5,'Color',[0.85,0.33,0.10]);
ylabel('Force (N)');
yyaxis right
q2PosPres2 = plot(data(:,1),data(:,12),'LineWidth',1.5,'Color',[0.93,0.69,0.13]);
ylabel('Elevation angle (Rad)');
legend('Fz','Spring Fz','Q2');title('Vertical Forces');
xlabel('Time (sec)');

subplot(4,6,[16,23])
mPlot = quiver3(data(:,5),data(:,6),data(:,7),data(:,8),data(:,9),data(:,10));
grid on; xlim([-1.2,1.2]); ylim([-1.2, 1.2]); zlim([-0.5, 0.5]);
view(3); title('Mass of Model');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)')

subplot(4,6,18)
tPlotX = plot(data(:,1),data(:,5),'LineWidth',1.5,'Color',[0.00,0.45,0.74]);
hold on; grid on; xlim([0,timeInSecs]);
tPlotY = plot(data(:,1),data(:,6),'LineWidth',1.5,'Color',[0.85,0.33,0.10]);
tPlotZ = plot(data(:,1),data(:,7),'LineWidth',1.5,'Color',[0.93,0.69,0.13]);
legend('X','Y','Z');title('Task Space X-Y');
xlabel('Time (sec)'); ylabel('Position (m)');

subplot(4,6,24)
lTime = plot(data(:,1),data(:,23),'LineWidth',1.5,'Color',[0.00,0.45,0.74]);
grid on; xlim([0,timeInSecs]); ylim([0,10]);
title('Loop Time'); xlabel('Time (sec)'); ylabel('Looptime (mSec)');

%% write data to file
disp('Writing file............................')
writematrix(data,csvFile)

%% saves plots
set(fig1, 'PaperOrientation', 'landscape', 'PaperUnits', 'normalized', 'PaperPosition',[0,0,1,1]);
saveas(fig1, ['./Logs/armSupport',logFileID,fileTime,'_1of2.pdf']);
saveas(fig1, ['./Logs/armSupport',logFileID,fileTime,'_1of2.fig']);

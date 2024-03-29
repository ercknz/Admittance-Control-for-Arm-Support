%% Serial to CSV script
% This code takes in the serial data being sent from the dynamixel board to
% logs it to a csv file.
%
% Script by erick nunez

%% Clean workspace
clear; clc; close all; delete(instrfindall);
%% CSV file
timeInSecs = 30;
logFileID = '_testNewCode_';
fileTime = datestr(now,'mmddyyHHMM');
csvFile = ['./Logs/armSupportLog',logFileID,fileTime,'.csv'];
pause(2)

%% Modify mass and damping setup
Mxy = 9.9; % kg
Mz = 10.1; % kg
Bxy = 75.5; % N*(sec/m)
Bz = 25.5; % N*(sec/m)
scalingFactor = 0.25;
filterWeight = 0.33;
eFx = 0.0; % N
eFy = 0.0; % N
eFz = 0.0; % N
bytesMxy = typecast(int32(Mxy*1000),'uint8');
bytesMz = typecast(int32(Mz*1000),'uint8');
bytesBxy = typecast(int32(Bxy*1000),'uint8');
bytesBz = typecast(int32(Bz*1000),'uint8');
byteFactor = uint8(scalingFactor*100);
byteFilter = uint8(filterWeight*100);
bytesFx = typecast(int32(eFx*1000),'uint8');
bytesFy = typecast(int32(eFy*1000),'uint8');
bytesFz = typecast(int32(eFz*1000),'uint8');
configHeader = uint8([150, 10, 10, 96]);
rxHeader = uint8([170, 8, 69, 0]);
modByte1 = uint8(15);
modByte2 = uint8(3);
blankByte = uint8(0);
writePacket = [configHeader,...
               modByte1,modByte2,blankByte,...
               bytesMxy,bytesMz,bytesBxy,bytesBz,bytesFx,bytesFy,bytesFz...
               byteFactor,byteFilter];
checkSum = sum(writePacket);
csHi = uint8(floor(checkSum/256));
csLo = uint8(mod(checkSum,256));
writePacket = [writePacket,csHi,csLo];
writePacketLen = length(writePacket);
packetSent = false;

%% Sets up and open serial object
BaudRate = 115200;
packetLen = 146;
dt = 0.008;
numFrames = timeInSecs/dt; % seconds*(1frame/secs)=frames
rawData = nan(numFrames,packetLen);
data = nan(numFrames,35);

%% Open Serial Port
disp('........opening port...........');
s1 = serialport('COM16',BaudRate);

%% start main collection loop
totalTime = 0; i = 1;
while(totalTime < timeInSecs )
    if ~packetSent && i > 5
        packetSent = true;
        write(s1,writePacket,'uint8');
        disp('.....sending config')
    end
    while s1.NumBytesAvailable < packetLen
        % Waiting for bytes
    end
    readBuffer = read(s1,packetLen,'uint8');
    tempHeader = readBuffer(1:4);
    inCS = readBuffer(end-1)* 256 + readBuffer(end);
    cCS = sum(readBuffer(1:end-2));
    if (sum(tempHeader == rxHeader)==4) && (inCS == cCS)
        rawData(i,:) = readBuffer;
    end
    % Loop Data
    i = i + 1;
    totalTime = totalTime + dt; 
end

for i = 1:numFrames
    data(i,1) = typecast(uint8(rawData(i,5:8)),'uint32');
    for j = 2:34
        data(i,j) = double(typecast(uint8(rawData(i,4*j+1:4*j+4)),'int32'));
    end
    data(i,35) = typecast(uint8(rawData(i,141:144)),'uint32');
end
data(:,1:34) = data(:,1:34) ./ 1000;

%% post cleanup
delete(s1);
disp('.........Serial Connection close...............')

%% plot data
% fig1 = figure;
% set(fig1, 'Units', 'Normalized','OuterPosition', [0 ,0, 1, 1]);
% 
% subplot(4,6,1)
% q1PosPres = plot(data(:,1),data(:,11),'LineWidth',1.5,'Color',[0.00,0.45,0.74]);
% hold on; grid on; xlim([0,timeInSecs]);
% q1PosGoal = plot(data(:,1),data(:,14),'LineWidth',1.5,'Color',[0.85,0.33,0.10]);
% legend({'Pres','Goal'},'Location','best');title('Shoulder Position');
% legend('boxoff'); xlabel('Time (sec)');ylabel('Angle (rad)');
% 
% subplot(4,6,2)
% q2PosPres = plot(data(:,1),data(:,12),'LineWidth',1.5,'Color',[0.00,0.45,0.74]);
% hold on; grid on; xlim([0,timeInSecs]);
% q2PosGoal = plot(data(:,1),data(:,15),'LineWidth',1.5,'Color',[0.85,0.33,0.10]);
% legend({'Pres','Goal'},'Location','best');title('Elevation Position');
% legend('boxoff'); xlabel('Time (sec)');ylabel('Angle (rad)');
% 
% subplot(4,6,3)
% q3PosPres = plot(data(:,1),data(:,13),'LineWidth',1.5,'Color',[0.00,0.45,0.74]);
% hold on; grid on; xlim([0,timeInSecs]);
% q3PosGoal = plot(data(:,1),data(:,16),'LineWidth',1.5,'Color',[0.85,0.33,0.10]);
% legend({'Pres','Goal'},'Location','best');title('Elbow Position');
% legend('boxoff'); xlabel('Time (sec)');ylabel('Angle (rad)');
% 
% % subplot(4,6,[19,21])
% % yyaxis left
% % dampingPlot = plot(data(:,1),data(:,20),'LineWidth',1.5,'Color',[0.00,0.45,0.74]);
% % hold on; grid on; xlim([0,timeInSecs]); ylim([Bxy-5,Bxy2+5]); ylabel('Damping XY');
% % yyaxis right
% % dampingPlotY = plot(data(:,1),data(:,5),'LineWidth',1.5,'Color',[0.85,0.33,0.10]);
% % legend('Damping','X');title('Damping Regions');
% % xlabel('Time (sec)'); ylabel('X (m)')
% 
% subplot(4,6,[4,12])
% fPlotX = plot(data(:,1),data(:,5),'LineWidth',1.5,'Color',[0.00,0.45,0.74]);
% hold on; grid on; xlim([0,timeInSecs]);
% fPlotY = plot(data(:,1),data(:,6),'LineWidth',1.5,'Color',[0.85,0.33,0.10]);
% fPlotZ = plot(data(:,1),data(:,7),'LineWidth',1.5,'Color',[0.93,0.69,0.13]);
% legend('X','Y','Z');title('Global Forces');
% xlabel('Time (sec)'); ylabel('Force (N)');
% 
% subplot(4,6,[7,15])
% rfPlotX = plot(data(:,1),data(:,2),'LineWidth',1.5,'Color',[0.00,0.45,0.74]);
% hold on; grid on; xlim([0,timeInSecs]);
% rfPlotY = plot(data(:,1),data(:,3),'LineWidth',1.5,'Color',[0.85,0.33,0.10]);
% rfPlotZ = plot(data(:,1),data(:,4),'LineWidth',1.5,'Color',[0.93,0.69,0.13]);
% legend('X','Y','Z');title('Raw Forces');
% xlabel('Time (sec)'); ylabel('Force (N)');
% 
% % subplot(4,6,[7,15])
% % yyaxis left
% % zPlotU = plot(data(:,1),data(:,4),'LineWidth',1.5,'Color',[0.00,0.45,0.74]);
% % hold on; grid on; xlim([0,timeInSecs]);
% % zPlotS = plot(data(:,1),data(:,19),'LineWidth',1.5,'Color',[0.85,0.33,0.10]);
% % ylabel('Force (N)');
% % yyaxis right
% % q2PosPres2 = plot(data(:,1),data(:,12),'LineWidth',1.5,'Color',[0.93,0.69,0.13]);
% % ylabel('Elevation angle (Rad)');
% % legend('Fz','Spring Fz','Q2');title('Vertical Forces');
% % xlabel('Time (sec)');
% 
% % subplot(4,6,[16,23])
% % mPlot = quiver3(data(:,5),data(:,6),data(:,7),data(:,8),data(:,9),data(:,10));
% % grid on; xlim([-1.2,1.2]); ylim([-1.2, 1.2]); zlim([-0.5, 0.5]);
% % view(3); title('Mass of Model');
% % xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)')
% 
% subplot(4,6,[16,23])
% rPlot = plot3(data(:,5),data(:,6),data(:,7));
% grid on; xlim([-1.2,1.2]); ylim([-1.2, 1.2]); zlim([-0.5, 0.5]);
% view(3); title('End of Robot');
% xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)')
% 
% % subplot(4,6,[8,15])
% % mPlot = plot3(data(:,5),data(:,6),data(:,7));
% % grid on; xlim([-1.2,1.2]); ylim([-1.2, 1.2]); zlim([-0.5, 0.5]);
% % view(3); title('End of Robot');
% % xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)')
% 
% subplot(4,6,18)
% tPlotX = plot(data(:,1),data(:,5),'LineWidth',1.5,'Color',[0.00,0.45,0.74]);
% hold on; grid on; xlim([0,timeInSecs]);
% tPlotY = plot(data(:,1),data(:,6),'LineWidth',1.5,'Color',[0.85,0.33,0.10]);
% tPlotZ = plot(data(:,1),data(:,7),'LineWidth',1.5,'Color',[0.93,0.69,0.13]);
% legend('X','Y','Z');title('Task Space X-Y');
% xlabel('Time (sec)'); ylabel('Position (m)');
% 
% subplot(4,6,24)
% lTime = plot(data(:,1),data(:,23),'LineWidth',1.5,'Color',[0.00,0.45,0.74]);
% grid on; xlim([0,timeInSecs]); ylim([0,10]);
% title('Loop Time'); xlabel('Time (sec)'); ylabel('Looptime (mSec)');
% 
% %% write data to file
% disp('Writing file............................')
% writematrix(data,csvFile)
% 
% %% saves plots
% set(fig1, 'PaperOrientation', 'landscape', 'PaperUnits', 'normalized', 'PaperPosition',[0,0,1,1]);
% saveas(fig1, ['./Logs/armSupport',logFileID,fileTime,'_1of2.pdf']);
% saveas(fig1, ['./Logs/armSupport',logFileID,fileTime,'_1of2.fig']);

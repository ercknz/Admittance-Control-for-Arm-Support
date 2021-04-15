function serialToCSV()
%% Serial to CSV script
% This code takes in the serial data being sent from the dynamixel board to
% logs it to a csv file. 
% 
% Script by erick nunez

%% Clean workspace
close all; delete(instrfindall);
%% CSV file
% keyWords = input('String of keywords for this trial? ');
keyWords = 'test';
fileTime = datestr(now,'mmddyyHHMM');
csvFile = ['Logs/armSupportLog',keyWords,fileTime,'.csv'];
%% Sets up duration of test
% secs = input('Collection time duration in Secs? ');
secs = 30;
pause(2)

%% Modify mass and damping setup
Mxy = 2.25; % kg
Mz = 5.5; % kg
Bxy = 10.75; % 
Bz = 7.99; %
newMxy = typecast(int32(Mxy*10000),'uint8');
newMz = typecast(int32(Mz*10000),'uint8');
newBxy = typecast(int32(Bxy*10000),'uint8');
newBz = typecast(int32(Bz*10000),'uint8');
header = uint8([150, 10, 10, 96]);
modByte = uint8(15);
writePacket = [header,modByte,newMxy,newMz,newBxy,newBz];
checkSum = sum(writePacket);
csHi = uint8(floor(checkSum/256));
csLo = uint8(mod(checkSum,256));
writePacket = [writePacket,csHi,csLo];
writePacketLen = length(writePacket);
dataSent = false;

%% Sets up and open serial object
BaudRate = 115200;
packetLen = 98;
dt = 0.008;
numFrames = secs/dt; % seconds*(1frame/secs)=frames
rawData = nan(numFrames,packetLen);
data = nan(numFrames,23);

%% plot data
fig1 = figure;
set(fig1, 'Units', 'Normalized','OuterPosition', [0 ,0, 1, 1]);

subplot(2,6,1)
q1Pos = plot(data(:,1),data(:,[11 17])); grid on; xlim([0,secs]);
legend('Pres','Goal');title('Shoulder Position');xlabel('Time (sec)');ylabel('Angle (rad)');

subplot(2,6,2)
q2Pos = plot(data(:,1),data(:,[12 18])); grid on; xlim([0,secs]);
legend('Pres','Goal');title('Elevation Position');xlabel('Time (sec)');ylabel('Angle (rad)');

subplot(2,6,3)
q3Pos = plot(data(:,1),data(:,[13 19])); grid on; xlim([0,secs]);
legend('Pres','Goal');title('Elbow Position');xlabel('Time (sec)');ylabel('Angle (rad)');

subplot(2,6,7:9)
lTime = plot(data(:,1),data(:,23)); grid on; xlim([0,secs]); ylim([0,10]);
title('Loop Time'); xlabel('Time (sec)'); ylabel('Looptime (sec)');

subplot(2,6,4:6)
fPlot = plot(data(:,1),data(:,[2 3 4])); grid on; xlim([0,secs]);
legend('X','Y','Z');title('Global Forces'); xlabel('Time (sec)'); ylabel('Force (N)');

subplot(2,6,10:11)
mPlot = quiver3(data(:,5),data(:,6),data(:,7),data(:,8),data(:,9),data(:,10)); grid on; 
xlim([-1.2,1.2]); ylim([-1.2, 1.2]); zlim([-0.5, 0.5]); view(3);
title('Mass of Model'); xlabel('X (m)'); ylabel('Y (m)');

subplot(2,6,12)
tPlot = plot(data(:,1),data(:,[5 6 7])); grid on; xlim([0,secs]);
legend('X','Y','Z');title('Task Space X-Y'); xlabel('Time (sec)'); ylabel('Position (m)');
shg

%% Open Serial Port
disp('........opening port...........');
s1 = serialport('COM28',BaudRate);

%% start main collection loop
totalTime = 0; i = 1;
while(totalTime < secs )
    if i == 1000 && ~dataSent
        write(s1,writePacket,'uint8');
        dataSent = true;
        disp('sending......')
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
        % Pres Q Dot
        data(i,14) = double(typecast(uint8(rawData(i,57:60)),'int32'))/10000;
        data(i,15) = double(typecast(uint8(rawData(i,61:64)),'int32'))/10000;
        data(i,16) = double(typecast(uint8(rawData(i,65:68)),'int32'))/10000;
        % Goal Q
        data(i,17) = double(typecast(uint8(rawData(i,69:72)),'int32'))/10000;
        data(i,18) = double(typecast(uint8(rawData(i,73:76)),'int32'))/10000;
        data(i,19) = double(typecast(uint8(rawData(i,77:80)),'int32'))/10000;
        % Goal Q Dot
        data(i,20) = double(typecast(uint8(rawData(i,81:84)),'int32'))/10000;
        data(i,21) = double(typecast(uint8(rawData(i,85:88)),'int32'))/10000;
        data(i,22) = double(typecast(uint8(rawData(i,89:92)),'int32'))/10000;
        % Loop time
        data(i,23) = typecast(uint8(rawData(i,93:96)),'uint32');
        % Update plot
%         set(fPlot,'XData',data(1:i,1),'YData',data(1:i,[2 3 4]));
        set(lTime,'XData', data(1:i,1),'YData',data(1:i,23));
        set(mPlot,'XData',data(1:i,5),'YData',data(1:i,6),'ZData',data(1:i,7),'UData',data(1:i,8),'VData',data(1:i,9),'WData',data(1:i,10));
        drawnow
        % Loop Data
        i = i + 1;
        totalTime = totalTime + dt;
    end
end
data(:,1) = data(:,1)*0.001;

%% post cleanup
delete(s1);
disp('.........Serial Connection close...............')



%% write data to file
disp('Writing file............................')
%writematrix(data,csvFile)

%% saves plots
% set(fig1, 'PaperOrientation', 'landscape', 'PaperUnits', 'normalized', 'PaperPosition',[0,0,1,1]);
% saveas(fig1, ['./Logs/armSupport',keyWords,fileTime,'_1of2.pdf']);
% saveas(fig1, ['./Logs/armSupport',keyWords,fileTime,'_1of2.fig']);
% set(fig2, 'PaperOrientation', 'landscape', 'PaperUnits', 'normalized', 'PaperPosition',[0,0,1,1]);
% saveas(fig2, ['./Logs/armSupport',keyWords,fileTime,'_2of2.pdf']);
% saveas(fig2, ['./Logs/armSupport',keyWords,fileTime,'_2of2.fig']);

end
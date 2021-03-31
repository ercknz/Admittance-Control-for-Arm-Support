%% Serial to CSV script
% This code takes in the serial data being sent from the dynamixel board to
% logs it to a csv file. 
% 
% Script by erick nunez

%% Clean workspace
clear; clc; delete(instrfindall);
%% CSV file
% keyWords = input('String of keywords for this trial? ');
fileTime = datestr(now,'mmddyyHHMM');
csvFile = ['Logs/armSupportLog',fileTime,'.csv'];
%% Sets up duration of test
secs = input('Collection time duration in Secs? ');
pause(2)
%% Sets up and open serial object
BaudRate = 115200;
packetLen = 98;
dt = 0.008;
length = secs/dt; % seconds*(1frame/secs)=frames
rawData = nan(length,packetLen); times = nan(length,1);
data = nan(length,23);
disp('........opening port...........');
s1 = serialport('COM28',BaudRate);
%% start main collection loop
totalTime = 0; i = 1;
while(totalTime < secs )
    loopTime = tic;
    if s1.NumBytesAvailable >= packetLen
        rawData(i,:) = read(s1,packetLen,'uint8');
        % Total Time
        data(i,1) = swapbytes(typecast(uint8(rawData(i,5:8)),'uint32'));
        % XYZ Global Forces
        data(i,2) = typecast(uint8(rawData(i,9:12)),'single');
        data(i,3) = typecast(uint8(rawData(i,13:16)),'single');
        data(i,4) = typecast(uint8(rawData(i,17:20)),'single');
        
        data(i,5) = typecast(uint8(rawData(i,21:24)),'single');
        data(i,6) = typecast(uint8(rawData(i,25:28)),'single');
        data(i,7) = typecast(uint8(rawData(i,29:32)),'single');
        
        data(i,8) = typecast(uint8(rawData(i,33:36)),'single');
        data(i,9) = typecast(uint8(rawData(i,37:40)),'single');
        data(i,10) = typecast(uint8(rawData(i,41:44)),'single');
        
        data(i,11) = typecast(uint8(rawData(i,45:48)),'single');
        data(i,12) = typecast(uint8(rawData(i,49:52)),'single');
        data(i,13) = typecast(uint8(rawData(i,53:56)),'single');
        
        data(i,14) = typecast(uint8(rawData(i,57:60)),'single');
        data(i,15) = typecast(uint8(rawData(i,61:64)),'single');
        data(i,16) = typecast(uint8(rawData(i,65:68)),'single');
        
        data(i,17) = typecast(uint8(rawData(i,69:72)),'single');
        data(i,18) = typecast(uint8(rawData(i,73:76)),'single');
        data(i,19) = typecast(uint8(rawData(i,77:80)),'single');
        
        data(i,20) = typecast(uint8(rawData(i,81:84)),'single');
        data(i,21) = typecast(uint8(rawData(i,85:88)),'single');
        data(i,22) = typecast(uint8(rawData(i,89:92)),'single');
        % Loop time
        data(i,23) = swapbytes(typecast(uint8(rawData(i,93:96)),'uint32'));
        times(i) = totalTime;
        i = i + 1;
    end
    totalTime = totalTime + toc(loopTime);
end
%% Starts reading data
% Adjust the data varaible to match what is being sent from the controller.

% for i=1:length
%     serialData = fscanf(s1);
%     temp = strsplit(serialData,'\t');
%     time = 0.001*str2double(temp(1));
%     forceRaw = str2double(temp([2, 3, 4]));       % Raw Forces
%     forceGlob = str2double(temp([5, 6, 7]));       % Forces
%     presQ = str2double(temp([8, 9, 10]));       % init Angle
% %     presQdot = str2double(temp([8, 9, 10]));   % init Ang Vel 
%     goalM = str2double(temp([11, 12, 13]));    % goal Pos
%     goalMdot = str2double(temp([14, 15, 16])); % goal Vel
%     goalQ = str2double(temp([17, 18, 19]));    % goal Ang
% %     goalQdot = str2double(temp([20, 21, 22])); % goal Ang Vel
%     loopTime = str2double(temp(20));    % Loop time
%     data(i,:)=[time,forceRaw,forceGlob,presQ,goalM,goalMdot,goalQ,loopTime];
% end
%% post cleanup
delete(s1);
disp('.........Serial Connection close...............')
%% write data to file
disp('Writing file............................')
%writematrix(data,csvFile)
%% plot data
% close all
% fig1 = figure;
% set(fig1, 'Units', 'Normalized','OuterPosition', [0 ,0, 1, 1]);
% subplot(2,3,1)
% plot(data(:,1),data(:,[8 17])); grid on;
% legend('Pres','Goal');title('Shoulder Position');xlabel('Time (sec)');ylabel('Angle (rad)');
% subplot(2,3,2)
% plot(data(:,1),data(:,[9 18])); grid on;
% legend('Pres','Goal');title('Elevation Position');xlabel('Time (sec)');ylabel('Angle (rad)');
% subplot(2,3,3)
% plot(data(:,1),data(:,[10 19])); grid on;
% legend('Pres','Goal');title('Elbow Position');xlabel('Time (sec)');ylabel('Angle (rad)');
% % subplot(2,3,4)
% % plot(data(:,1),data(:,[8 20])); grid on;
% % legend('Pres','Goal');title('Shoulder Velocity');xlabel('Time (sec)');ylabel('Angular Velocity (rad/sec)');
% % subplot(2,3,5)
% % plot(data(:,1),data(:,[9 21])); grid on;
% % legend('Pres','Goal');title('Elevation Velocity');xlabel('Time (sec)');ylabel('Angular Velocity (rad/sec)');
% % subplot(2,3,6)
% % plot(data(:,1),data(:,[10 22])); grid on;
% % legend('Pres','Goal');title('Elbow Velocity');xlabel('Time (sec)');ylabel('Angular Velocity (rad/sec)');
% subplot(2,3,4:6)
% plot(data(:,1),data(:,20)); grid on; ylim([0,10])
% title('Loop Time'); xlabel('Time (sec)'); ylabel('Looptime (sec)');
% 
% fig2 = figure;
% set(fig2, 'Units', 'Normalized','OuterPosition', [1, 0, 1, 1]);
% subplot(2,2,1)
% plot(data(:,1),data(:,[2 3 4])); grid on;
% legend('X','Y','Z');title('Raw Forces'); xlabel('Time (sec)'); ylabel('Force (N)');
% subplot(2,2,2)
% plot(data(:,1),data(:,[5 6 7])); grid on;
% legend('X','Y','Z');title('Global Forces'); xlabel('Time (sec)'); ylabel('Force (N)');
% subplot(2,2,3)
% quiver3(data(:,11),data(:,12),data(:,13),data(:,14),data(:,15),data(:,16)); grid on; 
% xlim([-1.2,1.2]); ylim([-1.2, 1.2]); zlim([-0.5, 0.5]); view(3);
% title('Mass of Model'); xlabel('X (m)'); ylabel('Y (m)');
% subplot(2,2,4)
% plot(data(:,1),data(:,[11 12 13])); grid on;
% legend('X','Y','Z');title('Task Space X-Y'); xlabel('Time (sec)'); ylabel('Position (m)');

%% saves plots
% set(fig1, 'PaperOrientation', 'landscape', 'PaperUnits', 'normalized', 'PaperPosition',[0,0,1,1]);
% saveas(fig1, ['./Logs/armSupport',keyWords,fileTime,'_1of2.pdf']);
% saveas(fig1, ['./Logs/armSupport',keyWords,fileTime,'_1of2.fig']);
% set(fig2, 'PaperOrientation', 'landscape', 'PaperUnits', 'normalized', 'PaperPosition',[0,0,1,1]);
% saveas(fig2, ['./Logs/armSupport',keyWords,fileTime,'_2of2.pdf']);
% saveas(fig2, ['./Logs/armSupport',keyWords,fileTime,'_2of2.fig']);
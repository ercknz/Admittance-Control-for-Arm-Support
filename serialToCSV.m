%% Serial to CSV script
% This code takes in the serial data being sent from the dynamixel board to
% logs it to a csv file. 
% 
% Script by erick nunez

%% Clean workspace
clear; clc; delete(instrfindall);
%% CSV file
csvFile = 'Logs/armSupportLog.csv';
%% Sets up serial object
s1 = serial('COM24');
s1.BaudRate = 115200;
%% Waits for data
fopen(s1);
s1.ReadAsyncMode = 'continuous';
readasync(s1);
while(s1.BytesAvailable < 0)
   disp('Waiting for data...............');
end
%% Starts reading data
% Adjust the data varaible to match what is being sent from the controller.
length = 60*100; % seconds*100=frames
data = zeros(length,1);
for i=1:length
    serialData = fscanf(s1);
    temp = strsplit(serialData,'\t');
    data(i,1) = 0.001*str2double(temp(1));
    data(i,2:3) = str2double([temp(2) temp(3)]);
    data(i,4:5) = str2double([temp(4) temp(5)]);
    data(i,6:7) = str2double([temp(6) temp(7)]);
    data(i,8:9) = str2double([temp(8) temp(9)]);
end
%% post cleanup
fclose(s1);delete(s1);
disp('.........Serial Connection close...............')
%% write data to file
disp('Writing file............................')
writematrix(data,csvFile)
    
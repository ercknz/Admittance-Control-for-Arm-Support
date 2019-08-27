%% Serial to CSV script
% This code takes in the serial data being sent from the dynamixel board to
% logs it to a csv file. 
% 
% Script by erick nunez

%% Clean workspace
clear; clc; delete(instrfindall);
%% CSV file
csvFile = 'armSupportLog.csv';
%% Sets up serial object
s1 = serial('COM24');
s1.BaudRate = 115200;
%% Waits for data
fopen(s1);
s1.ReadAsyncMode = 'continuous';
readasync(s1);
while(s1.BytesAvailable < 0)
   disp('Waiting for data') 
end
%% Starts reading data
% Adjust the data varaible to match what is being sent from the controller.
length = 30*100; % seconds*100=frames
data = zeros(length,1);
for i=1:length
%while(s1.BytesAvailable > 0)
    serialData = fscanf(s1);
    temp = strsplit(serialData,'\t');
    data(i,1) = 0.001*str2double(temp(1));
    data(i,2:4) = str2double([temp(2) temp(3) temp(4)]);
    data(i,5:7) = str2double([temp(5) temp(6) temp(7)]);
end
%% post cleanup
fclose(s1);delete(s1);
%% write data to file
writematrix(data,csvFile)
    
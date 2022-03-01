%% clear data
clear; cla; close all;
%% import file
data = readmatrix('armSupportLog.csv');
%% plot data
fig1 = figure(randi(1000));
set(fig1, 'Units', 'Normalized', 'OuterPosition', [0,0, 1, 1]);
subplot(3,1,1)
plot(data(:,1),data(:,2),'b'); 
hold on; grid on; 
plot(data(:,1),data(:,3),'r');
title('Force vs Time'); xlabel('Time (secs)'); ylabel('Force (N)');
legend('X','Y');
subplot(3,1,2)
plot(data(:,1),data(:,4),'b'); 
hold on; grid on; 
plot(data(:,1),data(:,16),'r');
legend('Previous','Goal');
title('Elbow Position vs Time');
xlabel('Time (secs)'); ylabel('Motor Counts');
subplot(3,1,3)
plot(data(:,1),data(:,5),'b'); 
hold on; grid on; 
plot(data(:,1),data(:,17),'r');
legend('Previous','Goal');
title('Shoulder Position vs Time');
xlabel('Time (secs)'); ylabel('Motor Counts');
%% display
disp(mean(diff(data(:,1))))
%% Save plots
% set(fig1, 'PaperOrientation', 'portrait', 'PaperUnits', 'normalized', 'PaperPosition',[0,0,1,1])
% saveas(fig1, 'armSupportLogPlot.pdf');
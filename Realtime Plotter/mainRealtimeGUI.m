%% Main Realtime GUI
% This script creates the main GUI used to plot the realtime data being
% sent from the serial port.
% 
% Script by erick nunez

%% clean up
close all; clear; clc;
%% Set up figure
fig1 = figure;
set(fig1, 'Units', 'Normalized', 'OuterPosition', [0,0, 1, 1]);
figAx = axes; 
set(figAx, 'Units', 'normalized', 'OuterPosition', [0.03,0.05,0.7,0.95]);
grid on; xlim([-1.2,1.2]); ylim([-1.2,1.2]);

%% UI
serialPanel = uipanel(fig1,'Units','normalized','Position',[0.75,0.8,0.1,0.15]);
serialCommButton = uicontrol('Style','togglebutton');
serialCommButton.String = 'Serial Communication';
serialCommButton.Units = 'normalized';
serialCommButton.Position = [0.76,0.825,0.08,0.1];
%% Graphing spring equations
% by erick nunez

%% clear
clear; clc; close all;

%% Known Variables
Ks = 8231; % N/m
xo = 0.127; % m (5")
xi = 0.1651; % m (6.5")
numOfSprings = 3;
Xo = xo * numOfSprings;
Xi = xi * numOfSprings;
q2 = -45:0.1:45; % range of q2 in degs
sideA = 0.066; % m
sideB = 0.0365; % m
scalingFactor = 0.01;

%% Calculated Variables
alpha = 90 - q2;
L = sqrt(sideA^2 + sideB^2 - 2 * sideA * sideB * cosd(alpha));
Li = sqrt(sideA^2 + sideB^2 + 2 * sideA * sideB * cosd(45));
beta = asind((sideA./L) .* sind(alpha));
betaI = asind((sideA./Li) .* sind(135));
Fzi = Ks *  (Xi-Xo) * sind(betaI + 45);
Fz = scalingFactor * ((Ks * (L-Li + Xi-Xo) .* sind(beta - q2)) - Fzi); 

%% plot
subplot(1,2,1)
plot(q2,L-Li); grid on;
xlabel('Q2 Angle');
ylabel('length (m)');

subplot(1,2,2)
plot(q2,Fz); grid on;
xlabel('Q2 Angle');
ylabel('Spring Force (N)');
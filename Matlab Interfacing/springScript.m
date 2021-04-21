%% Graphing spring equations
% by erick nunez

%% clear
clear; clc; close all;

%% Known Variables
Ks = 8231; % N/m
Xo = 0.127; % m (5")
Xi = 0.16002; % m (6.3")
q2 = -45:0.1:45; % range of q2 in degs
sideA = 0.066; % m
sideB = 0.0365; % m
measuredFsi = 15;

%% Calculated Variables
alpha = 90 - q2;
L = sqrt(sideA^2 + sideB^2 - 2 * sideA * sideB * cosd(alpha));
Li = sqrt(sideA^2 + sideB^2);
beta = asind((sideA./L) .* sind(alpha));
betaI = asind((sideA./Li) * sind(90));
Fzi = Ks *  (Xi-Xo) * sind(betaI);
scalingFactor = measuredFsi/Fzi;
Fz = 0.07 * ((Ks * (L-Li + Xi-Xo) .* sind(beta - q2)) - Fzi); 

%% plot
subplot(1,2,1)
plot(q2,L-Li); grid on;
xlabel('Q2 Angle');
ylabel('length (m)');

subplot(1,2,2)
plot(q2,Fz); grid on;
xlabel('Q2 Angle');
ylabel('Spring Force (N)');
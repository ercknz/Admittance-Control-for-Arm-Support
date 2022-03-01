%% clear
clear; clc;

%% syms set up
syms A1 A2 L1 L2 OS q1 q2 q4;

%% jacobian matrix
J11 = - (A1 + A2) * sin(q1) - L1 * sin(q1) * cos(q2) + OS * cos(q1 + q4) - L2 * sin(q1 + q4);
J12 = - L1 * cos(q1) * sin(q2);
J13 = OS * cos(q1 + q4) - L2 * sin(q1 + q4);
J21 = (A1 + A2) * cos(q1) + L1 * cos(q1) * cos(q2) + OS * sin(q1 + q4) + L2 * cos(q1 + q4);
J22 = - L1 * sin(q1) * sin(q2);
J23 = OS * sin(q1 + q4) + L2 * cos(q1 + q4);
J32 = L1 * cos(q2); 

%% Construct
JM = [J11, J12, J13; J21, J22, J23; 0, J32, 0];

invJ = inv(JM);

syms u v w
Q1st = invJ*[u; v; w];
qDot1 = simplify(Q1st(1))
qDot2 = simplify(Q1st(2))
qDot4 = simplify(Q1st(3))

Q2nd = JM\[u; v; w];
qDot1 = simplify(Q2nd(1))
qDot2 = simplify(Q2nd(2))
qDot4 = simplify(Q2nd(3))
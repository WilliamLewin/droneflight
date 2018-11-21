% FUNCTIONS

get_pulsewidth_value = @(w) 10^(-6) * 2*pi / w;


% SYMBOLS

% Thrust, roll, pitch and yaw control symbols
syms F_T tau_phi tau_theta tau_psi
% Angular velocities: omega1, omega2, omega3, omega4
syms w1 w2 w3 w4
% Angular velocity squared vector: omega^2
ww = [w1^2; w2^2; w3^2; w4^2];


% CONSTANTS

% Thrust factor [N/omega^2]
b = 1.63 * 10^(-5);
% Power factor
d = 5.03 * 10^(-7);
% Quadcopter axis length [m]
l = 0.3;


% MOVEMENT CONTROLS

% Thrust control [N]: F_T
U1 = @(b, ww) b * (ww(1) + ww(2) + ww(3) + ww(4));
% Roll control [Nm]: U2 = tau_phi
U2 = @(b, l, ww) b*l * (ww(1) - ww(2) + ww(3) - ww(4));
% Pitch control [Nm]: U3 = tau_theta
U3 = @(b, l, ww) b*l * (-ww(1) - ww(2) + w(3) + ww(4));
% Yaw control [Nm]: U4 = tau_psi
U4 = @(d, ww) d * (ww(1) - ww(2) - ww(3) + ww(4));


% MOVEMENT MATRIX

E_b = @(b, d, l) [zeros(2,4); [b, b, b, b]; [b*l, -b*l, b*l, -b*l]; [-b*l, -b*l, b*l, b*l]; [d, -d, -d, d]];


% MOVEMENT VECTOR

u_b = E_b(b, d, l) * ww;


% Angular velocity vector: omega
temp = solve([0; 0; F_T; tau_phi; tau_theta; tau_psi] == u_b, [w1 w2 w3 w4]);
w = [temp.w1(1); temp.w2(1); temp.w3(1); temp.w4(1)];
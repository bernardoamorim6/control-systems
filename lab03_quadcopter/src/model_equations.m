close all

load('materials/references_06.mat');
%Initialize values
m=0.5;
L=0.25;
k=3*10^-6;
b=10^-7;
g=9.81;
kd=0.25;
Ixx=5*10^-3;
Iyy=5*10^-3;
Izz=10^-2;
cm=10^4;


%Equilibrium
% Initialize variables
xdot_e = 0;
ydot_eit = 0;
zdot_e = 0;
vxdot_e = 0;
vydot_e = 0;
vzdot_e = 0;
phidot_e = 0;
thetadot_e = 0;
psidot_e = 0;
omegaxdot_e = 0;
omegaydot_e = 0;
omegazdot_e = 0;

u_e = g*m/(4*k*cm);
u_e_sum = 4*u_e;

% Define A matrix
A = [...
    0 0 0 1 0 0 0 0 0 0 0 0;                       %  1
    0 0 0 0 1 0 0 0 0 0 0 0;                       %  2
    0 0 0 0 0 1 0 0 0 0 0 0;                       %  3
    0 0 0 -kd/m 0 0 0 k*cm/m*u_e_sum 0 0 0 0;      %  4
    0 0 0 0 -kd/m 0 -k*cm/m*u_e_sum 0 0 0 0 0;     %  5
    0 0 0 0 0 -kd/m 0 0 0 0 0 0;                   %  6
    0 0 0 0 0 0 0 0 0 1 0 0;                       %  7
    0 0 0 0 0 0 0 0 0 0 1 0;                       %  8
    0 0 0 0 0 0 0 0 0 0 0 1;                       %  9
    0 0 0 0 0 0 0 0 0 0 0 0;                       %  10
    0 0 0 0 0 0 0 0 0 0 0 0;                       %  11
    0 0 0 0 0 0 0 0 0 0 0 0;                       %  12
];

% Define B matrix
B = [
    0 0 0 0;                                      %  1
    0 0 0 0;                                      %  2
    0 0 0 0;                                      %  3
    0 0 0 0;                                      %  4
    0 0 0 0;                                      %  5
    k*cm/m k*cm/m k*cm/m k*cm/m;                  %  6
    0 0 0 0;                                      %  7
    0 0 0 0;                                      %  8
    0 0 0 0;                                      %  9
    L*k*cm/Ixx 0 -L*k*cm/Ixx 0;                 %  10
    0 L*k*cm/Iyy 0 -L*k*cm/Iyy;                 %  11
    b*cm/Izz -b*cm/Izz b*cm/Izz -b*cm/Izz; %  12
    ]

%Define C matrix
C = [...
    1 0 0 0 0 0 0 0 0 0 0 0;  % x
    0 1 0 0 0 0 0 0 0 0 0 0;  % y
    0 0 1 0 0 0 0 0 0 0 0 0;  % z
    0 0 0 0 0 0 1 0 0 0 0 0;  % roll (phi)
    0 0 0 0 0 0 0 1 0 0 0 0;  % pitch (theta)
    0 0 0 0 0 0 0 0 1 0 0 0;  % yaw (psi)
];

% Define the input
t = 0:0.01:5;               % Time vector from 0 to 5 seconds with a step size of 0.01 seconds
u = zeros(length(t), 4);    % Initialize input matrix with the correct dimensions

u(t >= 1 & t < 5, :) = 5;   % Set input to [5 5 5 5] for 1s <= t < 50
% Create the state-space model
sys = ss(A, B, C, []);

% Save system
save('lin_sys.mat', 'A', 'B', 'C', 'sys');

% Simulate the system
y = lsim(sys, u, t);
% Plot the output
plot(t, y);
grid on;
xlabel('Time');
ylabel('Output');
title('System Response');
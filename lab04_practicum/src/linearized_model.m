% mechanical part parameters
m = 0.210; % [kg] mass of the rod
r = 0.145; % [m] radius of the arm
L = 0.305; % [m] length of the rod
B_eq = 0.004; % [N m s / rad] viscous damping coefficient
J_eq = 0.0044; % [kg m^2] inertia of the arm at the rotation axis
J_cm = 1/3 * m * L^2; % [kg m^2] inertia of the rod

% electrical part parameters
eta_m = 0.69; % motor efficiency
eta_g = 0.9; % gearbox efficiency
K_t = 0.00767; % [N m / A] motor torque constant
K_m = 0.00767; % [V s / rad] motor back EMF constant
K_g = 70; % gearbox ratio
R_m = 2.6; % [Ohm] motor armature resistance

% constants
g = 9.81; % [m / s^2] gravitational acceleration

% linearized model description

a = J_eq + m * r^2;
b = m * L * r;
c = 4/3 * m * L^2;
d = m * g * L;

E = a*c - b^2;
F = eta_m * eta_g * K_t * K_g / R_m;
G = F * K_m * K_g + B_eq; 

A = [
    0,     0,      1, 0; % \theta
    0,     0,      0, 1; % \alpha
    0, b*d/E, -c*G/E, 0; % \dot{\theta}
    0, a*d/E, -b*G/E, 0  % \dot{\alpha}
]

B = [0; 0; c*F/E; b*F/E]

C = [
    1, 0, 0, 0;
    0, 1, 0, 0
]

D = [0; 0]

% nonlinear model description
% function states = nonlinearModelStep(x_0, u, time)
    
% end

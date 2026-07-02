%Discretize the model
Ts = 0.05;                   % Sampling time (in seconds)
sys_dt = c2d(sys, Ts, 'zoh');  % Discrete-time state-space model
Ad=sys_dt.A; Bd= sys_dt.B; Cd = C; Dd= zeros(6,4);
poless = eig(Ad) 
trans_zeros = tzero(sys_dt)
ctrb_mat = ctrb(Ad, Bd);
rank(ctrb_mat) 

obsv_mat = obsv(Ad, Cd);
rank(obsv_mat)

%Control canonical form
[eigenvectors,eigenvalues] = eig(Ad);
T = ctrb_mat;

% Create the transfer function object
z = tf('z'); 
% G = Cd * inv(z*eye(size(Ad)) - Ad) * Bd;
% numerator = G.Numerator;
% denominator = G.Denominator;



G = [1.2, 3.4, 0.7, 4.9, 2.3, 1.5, 0.8, 3.2, 2.9, 4.7, 1.1, 0.3;
     2.6, 0.9, 3.8, 0.1, 4.3, 1.7, 2.4, 0.5, 4.2, 1.6, 3.1, 0.6;
     4.8, 1.4, 3.6, 0.4, 2.1, 4.5, 1.9, 3.7, 0.2, 1.8, 2.7, 4.6;
     3.5, 0.5, 1.3, 4.0, 2.8, 3.9, 1.6, 0.9, 4.3, 1.2, 2.5, 0.7];
%G=eye(4,12);
BG = Bd*G;

% Create the desired pole matrix
V = 0.5*eye(12,12);

% Solve the Sylvester equation
X = sylvester(Ad, -V, BG);
K=G*inv(X);
display(K);
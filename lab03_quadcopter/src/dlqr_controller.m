load('lin_sys.mat', 'sys');

Ts = 0.05;                   % Sampling time (in seconds)
sys_dt = c2d(sys, Ts, 'zoh');  % Discrete-time state-space model

Q = diag([2, 2, 20, 1, 1, 1, 2, 2, 0.1, 0.1, 0.1, 0.1])
R = eye(4)

[K, S, e] = dlqr(sys_dt.A, sys_dt.B, Q, R, zeros(12, 4))
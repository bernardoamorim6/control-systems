load('lin_sys.mat', 'C', 'sys');

Ts = 0.05;                   % Sampling time (in seconds)
sys_dt = c2d(sys, Ts, 'zoh');  % Discrete-time state-space model

Ad = sys_dt.A
Bd = sys_dt.B

% A matrix of augmented system
% the eye(3, 3) is tracking only the state of (x, y, z)
NA = [
    eye(3, 3), eye(3, 12);
    zeros(12, 3), Ad
]
NB = [zeros(3, 4); sys_dt.B]


CM = ctrb(NA, NB);
assert(rank(CM) == 15)

Q = diag([0.2, 0.2, 0.5, 2, 2, 20, 1, 1, 1, 2, 2, 0.1, 0.1, 0.1, 0.1])
R = eye(4)

[K, S, e] = dlqr(NA, NB, Q, R, zeros(15, 4))

Ki = K(:,1:3)
Ks = K(:,4:end) 


pos_var = 2.5e-5
ang_var = 7.57e-5

qk_diag = repmat(1e-5, [1, 12])
qk_diag(6) = 2e-4
Qk = diag(qk_diag)

Rk = diag([repmat(pos_var, [1, 3]), repmat(ang_var, [1, 3])])

% Kalman filter
B1 = eye(12, 12)
[M, P] = dlqe(Ad, B1, C, Qk, Rk)
L = Ad * M

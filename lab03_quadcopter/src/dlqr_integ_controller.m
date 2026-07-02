load('lin_sys.mat', 'sys');

Ts = 0.05;                   % Sampling time (in seconds)
sys_dt = c2d(sys, Ts, 'zoh');  % Discrete-time state-space model

% A matrix of augmented system
% the eye(3, 3) is tracking only the state of (x, y, z)
NA = [
    eye(3, 3), eye(3, 12);
    zeros(12, 3), sys_dt.A
]
NB = [zeros(3, 4); sys_dt.B]


CM = ctrb(NA, NB);
assert(rank(CM) == 15)

Q = diag([0.2, 0.2, 0.5, 2, 2, 20, 1, 1, 1, 2, 2, 0.1, 0.1, 0.1, 0.1])
R = eye(4)

[K, S, e] = dlqr(NA, NB, Q, R, zeros(15, 4))

Ki = K(:,1:3)
Ks = K(:,4:end) 
  
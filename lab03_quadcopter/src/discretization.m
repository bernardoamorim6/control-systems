load('lin_sys.mat', 'C', 'sys');

Ts = 0.05;                   % Sampling time (in seconds)
sys_dt = c2d(sys, Ts, 'zoh');  % Discrete-time state-space model

Ad = sys_dt.A; 
Bd = sys_dt.B; 
Cd = C;

poles = eig(Ad) 
trans_zeros = tzero(sys_dt)
ctrb_mat = ctrb(Ad, Bd); 
rank(ctrb_mat) % controlable

obsv_mat = obsv(Ad, Cd);
rank(obsv_mat) % observable
ss_system = ss(A,B,C,D)
Q = [         
    10, 0, 0, 0;
    0, 20, 0, 0;
    0, 0, 4, 0;
    0, 0, 0, 2
]
R = [2]

[K,S,P] = lqr(ss_system, Q, R)
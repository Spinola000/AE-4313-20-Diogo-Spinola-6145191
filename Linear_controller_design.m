%% Script for design of the linear controler
% Euler
% First define linearized system

J11 = 124.531;
J22 = 124.586;
J33 = 1.704;

k1 = (J22 - J33)/J11
k2 = (J33 - J11)/J22
k3 = (J22 - J11)/J33
k1*k3
1 + 3*k1 + k1*k3
4*(k1*k3)^0.5

A_e = [zeros(3,3),eye(3,3);
    diag([-k3*n^2,3*k2*n^2,-4*k1*n^2]), [0,0,n*(k3-1);0,0,0;(1-k1)*n,0,0]]
B1 = zeros(3,3)
B2 = [0,0,1/J33; 0,1/J22,0;1/J11,0,0]
B_e = [B1;B2]
C = eye(6,6)
D =zeros(6,3)
Linearized_model_e = ss(A_e,B_e,C,D)
p = pole(Linearized_model_e)
Q1 = eye(3,3)./(5*deg)^2
Q = [Q1,zeros(3,3);zeros(3,3), Q1*(10)^2]
R = eye(3,3)./(0.1)^2

% Use LQR function do obtain optimal controller
K_e = lqr(Linearized_model_e,Q,R)

% Quaternions
A_q = [zeros(3,3),eye(3,3);
zeros(3,6)   
]
B_q = 0.5*[zeros(3,3);inv(J)]
Linearized_model_q = ss(A_q,B_q,C,D)
p = pole(Linearized_model_q)
Q1 = eye(3,3)./(0.05)^2
Q = [Q1,zeros(3,3);zeros(3,3), Q1*(15)^2]
R = eye(3,3)./(0.1)^2

% Use LQR function do obtain optimal controller
K_q = lqr(Linearized_model_q,Q,R)

%% Gains for NDI
K_e_ndi = B_e(4:6,:)*K_e

K_q_ndi = B_q(4:6,:)*K_q

%% Gains for NDI -TSS
% Inner loop design
A = zeros(3,3)
B = eye(3,3)
Q = eye(3,3)/(0.5*deg)^2
R = eye(3,3)/(0.1)^2
K_o = lqr(A,B,Q,R)

% Euler angles
R = Q
Q = eye(3,3)*1/(7.5*deg)^2
K_e_ndi_tss = lqr(A,B,Q,R)

% With quaternions

Q = eye(3,3)*1/(0.1)^2
K_q_ndi_tss = lqr(A,B,Q,R)
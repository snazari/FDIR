%% Fault Detection and Isolation using UIO
%  This example is taken from Chen's Robust Model Based FDIR book.  It is
%  example 3.3.3 presented on page 82.
%  Author: Sam Nazari
%  Date:  30-June-2015
clear,clc

%%
%----------State Space Formulation---------%
A   = [-3.6         0       0   0;
        0       -3.6702     0   0.0702;
        0           0 -36.2588  0.2588;
        0           0.6344 0.7781  -1.4125]
    
B   = [ 1 0 0;
        0 1 0;
        0 0 1;
        0 0 0]
    
C   = [1    0   0   0;
       0    1   0   0;
       0    0   1   0;
       0    0   0   0]
   
D   = 0
E   = [1;20.758;0;0]

%%
%---------LQR design----------%
Q=[.25 0 0 0;0 4 0 0;0 0 1 0;0 0 0 4];
R=eye(3)
[K l s]=lqr(A,B,Q,R);

%%
%----------rank(CE) = rank(E) ?= 1---------%
rank(C*E)
rank(E)

%%

%-------UIO Design------------%
H = E*inv((C*E)'*(C*E))*(C*E)'
T = eye(4)-H*C
A1 = T*A

rank(obsv(A1,C)) % Check observability of the pair (C,A1)

k1 = place(A',C',[-2,-10,-5,-3])'


F = A1-k1*C
k = k1 + F*H

%%
%----UIO-1---%
E1 = [E B(:,1)]
H1 = E1*inv((C*E1)'*(C*E1))*(C*E1)'
T1 = eye(4) - H1*C

K1 = place((T1*A),C',[-1,-20,-30,-40])'

% k11=k1;
% H1=zeros(4,4);
% H1(1:4,2:4)=.25;
% C1=C;
% C1(:,1)=0;
% T1=eye(4)-H1*C1;
% F1=T1*A-k11*C1;
% k12=F1*H1;
% K1=k11+k12;

%%
%----UIO-2----%
k21=k1;
H2=zeros(4,4);
H2(1:4,[1,3,4])=.25;
C2=C;
C2(:,2)=0;
T2=eye(4)-H2*C2;
F2=T2*A-k21*C2;
k22=F2*H2;
K2=k21+k22;

%%
%----UIO-3----%
k31=k1;
H3=zeros(4,4);
H3(1:4,[1,2,4])=.25;
C3=C;
C3(:,3)=0;
T3=eye(4)-H3*C3;
F3=T3*A-k31*C3;
k32=F3*H3;
K3=k31+k32;

%%
%---UIO-4----%
k41=k1;
H4=zeros(4,4);
H4(1:4,1:3)=.25;
C4=C;
C4(:,4)=0;
T4=eye(4)-H4*C4;
F4=T4*A-k41*C4;
k42=F4*H4;
K4=k41+k42;

%% Simulation results
%
% Set up simulation initial condiditons
x1_0       = -1;
x2_0       = -5;
x3_0       = 1;
x4_0       = 1; % theta = pi is vertically upward equilibrium

% sim time
TSIM = 30

d=5   % time that disturbance occurs

%% Sim the system
sim('massSpringMDL')
figure,
plot(tout,X(:,1)-X_hat(:,1),'r'),hold on,
plot(tout,X(:,2)-X_hat(:,2),'b'),hold on, 
plot(tout,X(:,3)-X_hat(:,3),'k'),hold on, 
plot(tout,X(:,4)-X_hat(:,4),'g'),
title('State Estimation Error for coupled mass spring system')
xlabel('time (sec)'),ylabel('m and m/s')
legend('q_1','q_2','dq_1','dq_2','Location','NorthEast')
xlim([0,10])
ylim([-1,1])
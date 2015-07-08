%% Fault Detection and Isolation using UIO
%  This example is taken from Chen's Robust Model Based FDIR book.  It is
%  example 3.3.3 presented on page 82.  We look in the residual from UIO 1 
%  in order to find faults occuring in u3 and we look in the residual
%  emenating from UIO 2 if there is a fault in u1 or u2.
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

k1 = place(A1,C,[-2,-10,-5,-3])


F = A1-k1*C
k = k1 + F*H

%%
%----UIO-1---%
E1 = [E B(:,1)]
H1 = E1*inv((C*E1)'*(C*E1))*(C*E1)'
T1 = eye(4) - H1*C

k1 = place((T1*A),C,[-10,-3,-2,-1])'
F1 = T1*A-k1*C
K1 = k1+F1*H1

C1=C;
C1(:,1)=0;

%%
%----UIO-2---%
E2 = [E B(:,2)]
H2 = E2*inv((C*E2)'*(C*E2))*(C*E2)'
T2 = eye(4) - H2*C

k2 = place((T2*A),C,[-10,-3,-2,-1])'
F2 = T2*A-k2*C
K2 = k2+F2*H2
C2 = C
C2(:,2)=0;

%----UIO-3---%
E3 = [E B(:,3)]
H3 = E3*inv((C*E3)'*(C*E3))*(C*E3)'
T3 = eye(4) - H3*C

k3 = place((T3*A),C,[-1,-3,-2,-5])'
F3 = T3*A-k3*C
K3 = k3+F3*H3
C3 = C
C3(:,3)=0;

%% Simulation Parameters
%

% Plant Inputs
u1Val = 34.632  % Input 1 step value
u2Val = 1641.6  % Input 2 step value
u3Val = 29980  % Input 3 step value

% Sensor Fault Params 
fsVal = 0  % Fault value 
fsTime= 7  % Time of fault

% Actuator Fault Params 
faVal = .2*u1Val  % Fault value 
faTime= 4  % Time of fault

% Select faulty actuator
fa_u1 = 1
fa_u2 = 0
fa_u3 = 0

% Set up simulation initial condiditons
x1_0       = 0.3412;
x2_0       = 525.7;
x3_0       = 472.2;
x4_0       = 496.2; 

% sim time
TSIM = 10


%% Sim the system
sim('chenEx3p3p3mdlV3')
figure,
plot(tout,estErrUIO(:,1),'r'),hold on,
plot(tout,estErrUIO(:,2),'b'),hold on,
plot(tout,estErrUIO(:,3),'g'),hold on,
plot(tout,estErrUIO(:,4),'k')
title('State Estimation Error for Chen Ex 3.3.3')
xlabel('time (hours)'),ylabel('Concentration of product')
legend('x_1','x_2','x_3','x_4','Location','NorthEast')
xlim([0,10])
ylim([-10,10])

figure
plot(tout,r1(:,1),'r'),hold on,
plot(tout,r1(:,2),'b'),hold on,
plot(tout,r1(:,3),'g'),hold on,
plot(tout,r1(:,4),'k')
title('Residual Signals from UIO 1 for Chen Ex 3.3.3')
xlabel('time (hours)'),ylabel('Residuals r(t)')
legend('r_1','r_2','r_3','r_4','Location','NorthEast')
xlim([0,10])
%ylim([-10,10])

figure
plot(tout,r2(:,1),'r'),hold on,
plot(tout,r2(:,2),'b'),hold on,
plot(tout,r2(:,3),'g'),hold on,
plot(tout,r2(:,4),'k')
title('Residual Signals from UIO 2 for Chen Ex 3.3.3')
xlabel('time (hours)'),ylabel('Residuals r(t)')
legend('r_1','r_2','r_3','r_4','Location','NorthEast')
xlim([0,10])

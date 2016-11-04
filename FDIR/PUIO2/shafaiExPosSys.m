%% Unknown Input Observer Dynamic System Matricies
% Sam Nazari
% April 2015
% This example is form the book: 
% Robust Model Based Fault Diagnosis for Dynamic Systems by J. Chen
% Please see the associated ShareLaTeX pdf file "UIO and Examples"
clear
clc
A = [-1 1 1;1 0 0;0 1 -1];
B = [1;0;1];
C = [1 0 0;0 1 0]; % this is the orginal system that shafai proposed, but
%I am changing it to make the pair A1,C observable
%C = [1 0 1;0 1 0];
D = 0;
E = [1;0;0];

%% Construct the state space system
% Let us construct the system as a state space object for simulation.
sys = ss(A,B,C,D);
%% Step one: Check rank
% We check to see if the rank(CE) = rank(E) = 1
rank(C*E)
rank(E)

%% Step two: Compute Observer Matricies
% Since we have no issues in the previous step, let us compute the observer
% matrices now:
H = E*inv((C*E)'*(C*E))*(C*E)'
T = eye(3)-H*C
A1 = T*A

%% Step three: check observability 
% We must check the system A1,C observability
rank(obsv(A1,C))
% Since rank(obsv(A1,C))<n, the pair (A1,C) is not observable.  Let us
% utilize the PBH test to see which eigenvalue of A1 is the culprit. 
l = eig(A1);
lam1=l(1),lam2=l(2),lam3=l(3)
plam1 = rank([lam1*eye(3)-A1;C])
plam2 = rank([lam2*eye(3)-A1;C])
plam3 = rank([lam3*eye(3)-A1;C])
% it can be seen that the eigenvalue that fails the PBH test is already in
% the left half plane (at -1).  Therefore, the pair (A1,C) is detectible
% and a UIO exists. 
%% Pole Placement 
% Pole placement is used to assign the observer poles
K1 = [3 0;-1 2;-1/2 0]
%% Finish observer design
% Finally, the F and K matricies are computed
F = A1-K1*C
K = K1 + F*H
eig(F)
%% Simulation results
%
% Set up simulation initial condiditons
x1_0 = 1;
x2_0 = 0;
x3_0 = 0;

d = 10;
TSIM = 50;

%% Sim the system
sim('shafExPosSys')

%% Plot the results
figure(1)
plot(tout,Xerr(:,1),'b'),hold on
plot(tout,Xerr(:,2),'g'),hold on
plot(tout,Xerr(:,3),'r')
xlabel('Time in seconds'),ylabel('State Estimate Error')
title('Positive Dynamic System and Positive UIO Observer State Estimation Error')
legend('x_1','x_2','x_3')


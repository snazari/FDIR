%% Unknown Input Observer Dynamic System Matricies
%  The convey-crane system is a well studied mechanical system in control
%  theory.  The control objective of this dynamic system is to move the
%  load that is suspended from the crane to the origin while minimizing the
%  oscillations occuring in the suspended mass. 
%  Author: Sam Nazari
%  Date:  April 2015
clear,clc

%% crane pole parameters
M   = 1   % kg
m   = 1   % kg
l   = 1 % m
g   = 9.8 % m/s/s

%% State Space formulation
A   = [0 1 0 0;
    0 0 (-m*g)/M 0;
    0 0 0 1;
    0 0 -(M+m)*g/(l*M) 0]
    
B   = [0;(1/M);0;(1/(l*M))]
C   = [1 0 0 0;0 0 0 0;0 0 1 0;0 0 0 1]
D   = 0
E   = [0;1;0;1]

%% Control of the crane-pole system
Q=[1e3 0 0 0;
    0 10 0 0;
    0 0 1e6 0;
    0 0 0 10]
R=1
[Klqr ll ss]=lqr(A,B,Q,R)

%% Step one: Check rank
% We check to see if the rank(CE) = rank(E) = 1
rank(C*E)
rank(E)

%% Step two: Compute Observer Matricies
% Since we have no issues in the previous step, let us compute the observer
% matrices now:
H = E*inv((C*E)'*(C*E))*(C*E)'
T = eye(4)-H*C
A1 = T*A

%% Step three: check observability 
% We must check the system A1,C observability
rank(obsv(A1,C))

%% Pole Placement 
% Pole placement is used to assign the observer poles
K1 = place(A',C',[-2,-10,-5,-3])'

%% Finish observer design
% Finally, the F and K matricies are computed
F = A1-K1*C
K = K1 + F*H

%% Simulation results
%
% Set up simulation initial condiditons
x_d_0       = 0;
x_0         = -5;
theta_d_0   = 0;
theta_0     = -pi/4; % theta = pi is vertically upward equilibrium

d=1

%% Sim the system
sim('conveyPoleMDL')
figure,
plot(tout,X(:,1))
figure,
plot(tout,X(:,3))
% %% Plot the results
% t = linspace(1,10,length(X_hat));
% figure(1),plot(t,X),hold on
% plot(t,X_hat(:,1),'xb'),hold on
% plot(t,X_hat(:,2),'xg'),hold on
% plot(t,X_hat(:,3),'xr')
% xlabel('Time in seconds'),ylabel('State trajectory and estimate')
% title('Dynamic system state and UIO estimate')
% legend('x_1','x_2','x_3','x_1 hat','x_2 hat','x_3 hat')
% 
% 

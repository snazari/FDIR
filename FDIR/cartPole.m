%% Unknown Input Observer Dynamic System Matricies
% This example is form the book: 
% Robust Model Based Fault Diagnosis for Dynamic Systems by J. Chen
% Please see the associated ShareLaTeX pdf file "UIO and Examples"
%
clear,clc

%% Cart pole parameters
M   = .5   % kg
I   = .006 % kg.m^2
m   = .2   % kg
b   = 0.1 % N/m/sec
c   = 0.1 % N/m/sec
l   = 0.3 % m
g   = 9.8 % m/s/s

%% State Space formulation - linearized about theta = pi...
D   = I*(M+m)+M*m*l^2
a22 = c/M
a23 = -m*g/M
a24 = b/(M*l)
a42 = c/(M*l)
a43 = ((M+m)*g)/(M*l)
a44 = (-(M+m)*b)/(M*m*l^2)

b2  = 1/M
b4  = -1/(M*l)

A   = [0 1 0 0;
    0 a22 a23 a24;
    0 0 0 1;
    0 a42 a43 a44]
    
B   = [0;b2;0;b4]
C   = [1 0 0 0;0 0 0 0;0 0 1 0;0 0 0 1]
D   = 0
E   = [0;1;0;1]

%% Control of the cart-pole system
Q=[10 0 0 0;
    0 1 0 0;
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
x_0         = 0;
theta_d_0   = 0.0;
theta_0     = pi; % theta = pi is vertically upward equilibrium

d=1

%% Sim the system
sim('cartPoleMDL')
x = X(:,1);
xd= X(:,2);
th= X(:,3);
thd=X(:,4);

figure
plot(tout,x)
figure
plot(tout,th*(180/pi))
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

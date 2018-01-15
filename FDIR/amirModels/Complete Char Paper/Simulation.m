%% Initialization
clear all;
close all;
clc;
% xDot = A x + B u + Ed d + Ea fa
% y = C x + Es fs
A=[-1 1 1;1 -2 0;0 1 -3];
B=[1;1;0];
C=[1 0 1;0 1 0];
x0=[1;2;3];

n = size(B,1);
m = size(B,2);
p = size(C,1);

% r = size(Ed,2);
% qa = size(Ea,2);
% qs = size(Es,2);

%% Positive Observer with Known Model Disturbance(Faultless)
% ddot = Md d, xhat=[x;d]
% Initializing
tEnd=6;
xhat0=[.1;.2;.3;.4;.5];
Md=[-1 0;0 -1];
%Md = -randn(r,r);
d0=[4;5];
Ed=[1 0;0 1;1 1];
% Finding Observer
r = size(Ed,2);
Aa = [A,zeros(n,r);zeros(r,n),Md];
Ba = [B;zeros(r,m)];
Ca = [C,zeros(p,r)];
[La,Pa,Ya] = POfinder(Aa,Ca);
% Simulating
sim('POModel');
yTS = yout.getElement(1);y = yTS.Values.Data;t = yTS.Values.Time;
xTS = yout.getElement(2);x = xTS.Values.Data;
dTS = yout.getElement(3);d = dTS.Values.Data;
xhatTS = yout.getElement(4);xhat = xhatTS.Values.Data;
% Plotting
figure()
hold on; box on;
subplot(5,1,1);plot(t,x(:,1),'r',t,xhat(:,1),'r--');
legend({'$x_1(t)$','$\hat{x}_1(t)$'},'Interpreter','latex','FontSize',14)
subplot(5,1,2);plot(t,x(:,2),'g',t,xhat(:,2),'g--');
legend({'$x_2(t)$','$\hat{x}_2(t)$'},'Interpreter','latex','FontSize',14)
subplot(5,1,3);plot(t,x(:,3),'b',t,xhat(:,3),'b--');
legend({'$x_3(t)$','$\hat{x}_3(t)$'},'Interpreter','latex','FontSize',14)
subplot(5,1,4);plot(t,d(:,1),'k',t,xhat(:,4),'k--');
legend({'$d_1(t)$','$\hat{d}_1(t)$'},'Interpreter','latex','FontSize',14)
subplot(5,1,5);plot(t,d(:,2),'m',t,xhat(:,5),'m--');
legend({'$d_2(t)$','$\hat{d}_2(t)$'},'Interpreter','latex','FontSize',14)
hold off; box off;


%% Positive Observer with Unknown Disturbance(Faultless) OR Actuator Fault(Disturbanceless)
% Initializing
tEnd=6;
z0=[0.1;0.2;0.3];
d=0.2; % Constant Disturbance
Ed=[1;0;0];  % OR Ea=[];
% Finding Observer
r = size(Ed,2);  
[F,G,H,M,N,P,Y,error,A1] = PUIOfinder(A,B,C,Ed,2); % 1 for thrm3.1 (G>0) and 2 for thrm3.2 (xhat>0) 
Ndhat = pinv(C*Ed);
% Simulating
sim('PUIOModel');
yTS = yout.getElement(1);y = yTS.Values.Data;t = yTS.Values.Time;
xTS = yout.getElement(2);x = xTS.Values.Data;
dTS = yout.getElement(3);d = dTS.Values.Data;
xhatTS = yout.getElement(4);xhat = xhatTS.Values.Data;
dhatTS = yout.getElement(5);dhat = dhatTS.Values.Data;
% Plotting
figure()
hold on; box on;
subplot(4,1,1);plot(t,x(:,1),'r',t,xhat(:,1),'r--');
legend({'$x_1(t)$','$\hat{x}_1(t)$'},'Interpreter','latex','FontSize',14)
subplot(4,1,2);plot(t,x(:,2),'g',t,xhat(:,2),'g--');
legend({'$x_2(t)$','$\hat{x}_2(t)$'},'Interpreter','latex','FontSize',14)
subplot(4,1,3);plot(t,x(:,3),'b',t,xhat(:,3),'b--');
legend({'$x_3(t)$','$\hat{x}_3(t)$'},'Interpreter','latex','FontSize',14)
subplot(4,1,4);plot(t,d,'k',t,dhat,'k--');
legend({'$d(t)$','$\hat{d}(t)$'},'Interpreter','latex','FontSize',14)
hold off; box off;


%% Positive Observer with Actuator and/or Sensor Fault(Distless)
% Initializing
tEnd=6;
v0=[0.1;0.2]; % this is used to build x0aug=[x0;v0]
z0=[0.1;0.2;0.3;0.4;0.5];
fa=2; % Constant Actuator Fault
fs=3; % Constant Sensor Fault
Ea=[1;0;1];
Es=[1;1];
Maug = diag([1,2]);
% Finding Observer
qa = size(Ea,2);
qs = size(Es,2);
Aaug=[A, zeros(n,p);Maug*C, -Maug];
Baug=[B;zeros(p,m)];
Caug=[C,eye(p,p)];
faug=[fa;fs];
Eaug=[Ea,zeros(n,qs);zeros(p,qa),Maug*Es];
x0aug=[x0;v0];
r = size(Eaug,2);
[F,G,H,M,N,P,Y,error] = PUIOfinder(Aaug,Baug,Caug,Eaug,2); % 1 for thrm3.1 (G>0) and 2 for thrm3.2 (xhat>0) 
Ndhat = pinv(Caug*Eaug);
% Simulating
sim('PUIOaugModel');
yaugTS = yout.getElement(1);yaug = yaugTS.Values.Data;t = yaugTS.Values.Time;
xaugTS = yout.getElement(2);xaug = xaugTS.Values.Data;
faugTS = yout.getElement(3);faug = faugTS.Values.Data;
xhatTS = yout.getElement(4);xhat = xhatTS.Values.Data;
fhataugTS = yout.getElement(5);fhataug = fhataugTS.Values.Data;
% Plotting
figure()
hold on; box on;
subplot(5,1,1);plot(t,xaug(:,1),'r',t,xhat(:,1),'r--');
legend({'$x_1(t)$','$\hat{x}_1(t)$'},'Interpreter','latex','FontSize',14)
subplot(5,1,2);plot(t,xaug(:,2),'g',t,xhat(:,2),'g--');
legend({'$x_2(t)$','$\hat{x}_2(t)$'},'Interpreter','latex','FontSize',14)
subplot(5,1,3);plot(t,xaug(:,3),'b',t,xhat(:,3),'b--');
legend({'$x_3(t)$','$\hat{x}_3(t)$'},'Interpreter','latex','FontSize',14)
subplot(5,1,4);plot(t,faug(:,1),'k',t,fhataug(:,1),'k--');
legend({'$f_a(t)$','$\hat{f}_a(t)$'},'Interpreter','latex','FontSize',14)
subplot(5,1,5);plot(t,faug(:,2),'m',t,fhataug(:,2),'m--');
legend({'$f_s(t)$','$\hat{f}_s(t)$'},'Interpreter','latex','FontSize',14)
hold off; box off;

%% Robust Positive Observer with Faults and Disturbance
% Initializing
tEnd=6;
%v0=[0.1;0.2]; % this is used to build x0aug=[x0;v0]
%z0=[0.1;0.2;0.3;0.4;0.5];
fa=3; % Constant Actuator Fault
Ea=[1;1;0];

% Finding Observer
qa = size(Ea,2);
Aaug=[A, zeros(n,p);Maug*C, -Maug];
Baug=[B;zeros(p,m)];
Caug=[C,eye(p,p)];
faug=[fa;fs];
Eaug=[Ea,zeros(n,qs);zeros(p,qa),Maug*Es];
x0aug=[x0;v0];
r = size(Eaug,2);
[F,G,H,M,N,P,Y,error] = PUIOfinder(Aaug,Baug,Caug,Eaug,2); % 1 for thrm3.1 (G>0) and 2 for thrm3.2 (xhat>0) 
Ndhat = pinv(Caug*Eaug);



clear all;
close all;
clc;
% xDot = A x + B u + Ed d + Ea fa
% y = C x + Es fs
% A=[-1 1 1;1 -2 0;0 1 -3];
% B=[1;1;0];
% C=[1 0 1;0 1 0];
A=[-2 0 1;0 -1 1;1 2 -3];
B=[1;0;1];
C=[1 2 3;0 1 2];

x0=[1;2;3];

n = size(B,1);
m = size(B,2);
p = size(C,1);
tEnd=2.5;
%v0=[0.1;0.2]; % this is used to build x0aug=[x0;v0]
%z0=[0.1;0.2;0.3;0.4;0.5];
d=1;
Ed=[1;1;0];
fa=3; % Constant Actuator Fault
Ea=[1;0;1];
xhat0=[2;1;1];
fahat0=[5];
R=0;

z0=xhat0;
w0=[1;1];



% Finding Observer
qa = size(Ea,2);
N = Ed* pinv(C*Ed);
T = eye(n)-N*C;
H = T*B;
J = T*Ea;
A1 = A - N*C*A;
A1hat=[A1, J;zeros(qa,n+qa)];
Chat=[C, zeros(p,qa)];
observerPoles = [-1,-2,-3,-4];
Lhat = (place(A1hat',Chat',observerPoles))';
G1 = Lhat(1:n,:);
L = Lhat(n+1:end,:);
F = A1-G1*C; 
G2 = F*N;
M1 = -L*C;
M2 = L*(eye(p)-C*N);
G = G1+G2;

%% Simulating
sim('PIUIOModelZW');
yTS = yout.getElement(1);y = yTS.Values.Data;t = yTS.Values.Time;
xTS = yout.getElement(2);x = xTS.Values.Data;
dTS = yout.getElement(3);d = dTS.Values.Data;
faTS = yout.getElement(4);fa = faTS.Values.Data;
xhatTS = yout.getElement(5);xhat = xhatTS.Values.Data;
fahatTS = yout.getElement(6);fahat = fahatTS.Values.Data;
zTS = yout.getElement(7);z = zTS.Values.Data;
wTS = yout.getElement(8);w = wTS.Values.Data;

%% Plotting
figure()
hold on; box on;
subplot(4,1,1);plot(t,x(:,1),'r',t,xhat(:,1),'r--');
legend({'$x_1(t)$','$\hat{x}_1(t)$'},'Interpreter','latex','FontSize',14)
subplot(4,1,2);plot(t,x(:,2),'g',t,xhat(:,2),'g--');
legend({'$x_2(t)$','$\hat{x}_2(t)$'},'Interpreter','latex','FontSize',14)
subplot(4,1,3);plot(t,x(:,3),'b',t,xhat(:,3),'b--');
legend({'$x_3(t)$','$\hat{x}_3(t)$'},'Interpreter','latex','FontSize',14)
subplot(4,1,4);plot(t,fa(:,1),'k',t,fahat(:,1),'k--');
legend({'$f_a(t)$','$\hat{f}_a(t)$'},'Interpreter','latex','FontSize',14)
% subplot(5,1,5);plot(t,d(:,1),'m',t,w(:,2),'m--');
% legend({'$d(t)$','$\hat{d}(t)$'},'Interpreter','latex','FontSize',14)
hold off;

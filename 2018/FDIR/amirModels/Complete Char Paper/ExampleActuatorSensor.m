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
tEnd=4;
v0=[0.1;0.2]; % this is used to build x0aug=[x0;v0]
z0=[0.1;.2;.3;0.4;0.5];
%z0=[5;5;5;.4;.5]; %test
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


N = Eaug* pinv(Caug*Eaug);
T = eye(size(Aaug))-N*Caug;
A1 = Aaug - N*Caug*Aaug;

% if( rank(Caug*Eaug) ~= rank(Eaug) )
%     disp('rank(CE) is different from rank(E)!')
% elseif( sum(sum(N<0)) > 0 )
%     disp('N is not positive!')
% elseif( sum(sum(T<0)) > 0) 
%     disp('T=I-NC is not positive!')
% elseif( rank(obsv(A1,Caug)) < rank(A1) )
%     disp('{A1,Caug} is not observable!')
% else
%     disp('ALL OK')
% end

%%
tol=1e-9; %1e-5
%tol=-5;
n1 = size(A1,1);
p1 = size(Caug,1);
warning off;
cvx_begin sdp
variable P(n1,n1) diagonal
variable Y(n1,p1)
minimize( 1 )
subject to
P >= 0;
C1 = A1'*P + P*A1 - Caug'*Y' - Y*Caug;
C1 <= 0 ;

C2 = A1'*P - Caug'*Y' + eye(n1);
C2(:) >= tol.*ones(n1*n1,1)

%C3 =  A1'*P - Caug'*Y' + eye(n1) ;
%C3(:) <= tol.*ones(n1*n1,1);

% C3 = P*N*Caug*Aaug + Y*Caug ;
% C3(:) >= tol.*ones(n1*n1,1);
cvx_solver sdpt3 %sedumi 
cvx_end

G1 = P\Y;
F = A1 - G1*Caug;
G = G1 + A1*N - G1*Caug*N;
H = T*Baug;
M = eye(size(Aaug));


%%
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
hold off;






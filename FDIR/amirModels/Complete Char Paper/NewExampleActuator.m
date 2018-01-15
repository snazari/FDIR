clear all;
close all;
clc;
% xDot = A x + B u + Ed d + Ea fa
% y = C x 
A=[-1 1 1;1 -2 0;0 1 -3];
B=[1;1;0];
C=[1 0 1;0 1 0];
x0=[1;2;3];

n = size(B,1);
m = size(B,2);
p = size(C,1);
tEnd=1.5;
z0=[0.1,.2,0.3];%[0.1;0.2;0.3];
d=2; % Constant Disturbance
Ed=[1;0;0];  % OR Ea=[];
r = size(Ed,2);

N = Ed* pinv(C*Ed);
T = eye(size(A))-N*C;
A1 = A - N*C*A;

if( rank(C*Ed) ~= rank(Ed) )
    disp('rank(CE) is different from rank(E)!')
elseif( sum(sum(N<0)) > 0 )
    disp('N is not positive!')
% elseif( sum(sum(T<0)) > 0) 
%     disp('T=I-NC is not positive!')
elseif( rank(obsv(A1,C)) < rank(A1) )
    disp('{A1,Caug} is not observable!')
else
    disp('ALL OK')
end

tol=1e-9;
n1 = size(A1,1);
p1 = size(C,1);
warning off;
cvx_begin sdp
variable P(n1,n1) diagonal
variable Y(n1,p1)
minimize( 1 )
subject to
P >= 0;
C1 = A1'*P + P*A1 - C'*Y' - Y*C;
C1 <= 0 ;
C2 = A1'*P - C'*Y' + eye(n1);
C2(:) <= tol.*ones(n1*n1,1);
%C3 = P*N*C*A + Y*C ;
%C3(:) >= tol.*ones(n1*n1,1);
cvx_solver sdpt3 %sedumi 
cvx_end
G1 = P\Y;

F = A1 - G1*C;
G = G1 + A1*N - G1*C*N;
H = T*B;
M = eye(size(A));
Ndhat = pinv(C*Ed);

% Simulating
sim('PUIOModel');
xTSp = yout.getElement(2);xp = xTSp.Values.Data; tp=xTSp.Values.Time;
dTSp = yout.getElement(3);dp = dTSp.Values.Data;
xhatTSp = yout.getElement(4);xhatp = xhatTSp.Values.Data;
dhatTSp = yout.getElement(5);dhatp = dhatTSp.Values.Data;

%%
A=[-1 1 1;1 -2 0;0 1 -3];
B=[1;1;0];
C=[1 0 1;0 1 0];
x0=[1;2;3];

n = size(B,1);
m = size(B,2);
p = size(C,1);
tEnd=1.5;
z0=[0.1,.2,0.3];%[0.1;0.2;0.3];
d=2; % Constant Disturbance
Ed=[1;0;0];  % OR Ea=[];
r = size(Ed,2);

N = Ed* pinv(C*Ed);
T = eye(size(A))-N*C;
A1 = A - N*C*A;


observerPoles = [-10,-15,-20];  
G1 = (place(A1',C',observerPoles))';

F = A1 - G1*C;
G = G1 + A1*N - G1*C*N;
H = T*B;
M = eye(size(A));
Ndhat = pinv(C*Ed);

% Simulating
sim('PUIOModel');
xTSr = yout.getElement(2);xr = xTSr.Values.Data; tr=xTSr.Values.Time;
dTSr = yout.getElement(3);dr = dTSr.Values.Data;
xhatTSr = yout.getElement(4);xhatr = xhatTSr.Values.Data;
dhatTSr = yout.getElement(5);dhatr = dhatTSr.Values.Data;


%% Plotting
figure()
hold on; box on;
%subplot(4,1,1);plot(tr,xr(:,1),'k',tr,xhatr(:,1),'k--',tp,xhatp(:,1),'k-.');
%legend({'$x_1(t)$','UIO Estimate','PUIO Estimate'},'Interpreter','latex','FontSize',14)
%subplot(4,1,2);plot(tr,xr(:,2),'k',tr,xhatr(:,2),'k--',tp,xhatp(:,2),'k-.');
%legend({'$x_2(t)$','UIO Estimate','PUIO Estimate'},'Interpreter','latex','FontSize',14)
subplot(2,1,1);plot(tr,xr(:,3),'k',tr,xhatr(:,3),'k--',tp,xhatp(:,3),'k-.');
legend({'State','UIO Estimate','PUIO Estimate'},'Interpreter','latex','FontSize',14)
subplot(2,1,2);plot(tr,dr,'k',tr,dhatr,'k--',tp,dhatp,'k-.');
legend({'Disturbance','UIO Estimate','PUIO Estimate'},'Interpreter','latex','FontSize',14)



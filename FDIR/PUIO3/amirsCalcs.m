clear all;
close all;
clc;

%% Graph Structure
Ag = [0 1 1 1 0 0 0;
     1 0 1 1 1 0 0;
     1 1 0 0 0 1 0;
     1 1 0 0 0 0 1;
     0 1 0 0 0 0 1;
     0 0 1 0 0 0 1;
     0 0 0 1 1 1 0];
 
Deg = eye(7);
for i = 1:7
    Deg(i,i) = sum(Ag(i,:));
end

C = eye(7)

L = Deg-Ag

x0 = [0 1 1 0 0 0 0]

A = -L

% Condition: Distinct Eigenvalues
eig(A)

Ed = [0 1 0 0 0 0 0]'
%%
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

%%
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
C2(:) >= tol.*ones(n1*n1,1);
C3 = P*N*C*A + Y*C ;
C3(:) >= tol.*ones(n1*n1,1);
cvx_solver sdpt3 %sedumi 
cvx_end

G1 = P\Y;
F = A1 - G1*C;
G = G1 + A1*N - G1*C*N;
H = T*B;
M = eye(size(A));
%%
Ndhat = pinv(C*Ed);
% Simulating
% Simulating
sim('PUIOModel');
yTS = yout.getElement(1);y = yTS.Values.Data;t = yTS.Values.Time;
xTS = yout.getElement(2);x = xTS.Values.Data;
dTS = yout.getElement(3);d = dTS.Values.Data;
xhatTS = yout.getElement(4);xhat = xhatTS.Values.Data;
dhatTS = yout.getElement(5);dhat = dhatTS.Values.Data;
%% Plotting
figure()
hold on; box on;
subplot(4,1,1);plot(t,x(:,1),'r',t,xhat(:,1),'r--');
legend({'$x_1(t)$','$\hat{x}_1(t)$'},'Interpreter','latex','FontSize',14)
subplot(4,1,2);plot(t,x(:,2),'g',t,xhat(:,2),'g--');
legend({'$x_2(t)$','$\hat{x}_2(t)$'},'Interpreter','latex','FontSize',14)
subplot(4,1,3);plot(t,x(:,3),'b',t,xhat(:,3),'b--');
legend({'$x_3(t)$','$\hat{x}_3(t)$'},'Interpreter','latex','FontSize',14)
subplot(4,1,4);plot(t,d,'k',t,dhat,'k--');
legend({'$f_a(t)$','$\hat{f}_a(t)$'},'Interpreter','latex','FontSize',14)
hold off;

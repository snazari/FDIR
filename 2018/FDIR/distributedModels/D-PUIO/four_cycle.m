%% Positive Unknown Input Observer Dynamic System Matricies
% Sam Nazari
% Jan 2018
clear
clc

%% Graph Structure
Ag = [0 1 0 1;1 0 1 0;0 1 0 1;1 0 1 0];
 
Deg = eye(4);
for i = 1:4
    Deg(i,i) = sum(Ag(i,:));
end

C = eye(4)

L = Deg-Ag

x0 = [0 1 1 0]

A = -L

% Condition: Distinct Eigenvalues
eig(A)

%% Construct Fault Vectors
f1 = [1 0 0 0]'  % Vertex one is the intruder
f2 = [0 1 0 0]'  % Vertex two is the intruder
f3 = [0 0 1 0]'  % Vertex three is the intruder
f4 = [0 0 0 1]'  % Vertex four is the intruder

E = f2

% Choose the agent to be attacked
flt1  = 0;
flt2  = 1;
flt3  = 0;
flt4  = 0;

% Choose a magnitude for the attack
f1Val = 10;
f2Val = 10;
f3Val = 10;
f4Val = 10;

% Chose the attack time
tf1   = 2;
tf2   = 2;
tf3   = 2;
tf4   = 2;

%% PUIO 1

% This UIO is insensitive to faults in agent 3, but can detect faults in agents 2 and 4:
bf1 = [0 0 0 1]' 

% Observer matrices
c1 = [0 1 0 0;
      0 0 0 1]

%% Step one: Check rank
rank(c1*bf1)
rank(bf1)

%% Step two: Check the condition of Lemma (3.2). If a nonnegative left inverse 
%  of CEa exists, then compute N=Ea(C*Ea)^g >= 0 and T = I-N*C >= 0
CE = c1*E
CEg= pinv(CE)
N = E*CEg
NC = N*c1
T  = eye(size(A)) - NC

%% Step three: Define A1 = A - N*C*A so that {A1, C} is observable
NCA = NC*A
A1  = A-NCA
rank(obsv(A1,c1))

if( rank(c1*E) ~= rank(E) )
    disp('rank(CE) is different from rank(E)!')
elseif( sum(sum(N<0)) > 0 )
    disp('N is not positive!')
% elseif( sum(sum(T<0)) > 0) 
%     disp('T=I-NC is not positive!')
elseif( rank(obsv(A1,c1)) < rank(A1) )
    disp('{A1,Caug} is not observable!')
else
    disp('ALL OK')
end
%% Step four: Place the poles of F = A1-G1*C
%p = [-1,-10,-20,-3,-5,-7,-9]
%G1=place(A1',c1',p)'
%F = A1-G1*c1
%eig(F)
tol=1e-9;
n1 = size(A1,1);
p1 = size(c1,1);
warning off;
cvx_begin sdp
variable P(n1,n1) diagonal
variable Y(n1,p1)
minimize( 1 )
subject to
P >= 0;
C1 = A1'*P + P*A1 - c1'*Y' - Y*c1;
C1 <= 0 ;
C2 = A1'*P - c1'*Y' + eye(n1);
C2(:) >= tol.*ones(n1*n1,1);
C3 = P*N*c1*A + Y*c1 ;
C3(:) >= tol.*ones(n1*n1,1);
cvx_solver sdpt3 %sedumi 
cvx_end

G1 = P\Y;
F = A1 - G1*c1;
%G = G1 + A1*N - G1*c1*N;
%H = T*B;
M = eye(size(A));
%% Step five: finish the PUIO
%G = G1 + A1*N - G1*c1*N
% H = T*B
H = 0

%% 
G2 = F*N
%% Simulation results
%
% Set up simulation initial condiditons
x1_0 = 1;
x2_0 = 0.5;
x3_0 = 0.25;

x0=[10,1,1,1];

d = 10;
TSIM = 50;
TH = 5 % detection Threshold
%% Sim the system
sim('PUIO_four_cycle')

%% Plot the results
figure(1)
subplot(211),plot(fn2,'k','lineWidth',1),xlabel('Time in seconds'),
legend('Residual Sig.'),grid on,ylim([0,TH]),xlim([0,5])
ylabel('Agent 2')
title('Distributed PUIO with Fault Occurance at t=2 in Agent 2')
% subplot(312),plot(fn3,'b','lineWidth',1),xlabel('Time in seconds'),
% legend('Residual Sig.'),grid on,ylim([0,TH]),xlim([0,5])
% ylabel('Agent 3'),
subplot(212),plot(fn4,'r','lineWidth',1),grid on,ylim([0,TH]),xlim([0,5])
xlabel('Time in seconds'),
legend('Residual Sig.')
ylabel('Agent 4'),title(''),



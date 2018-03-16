%% Positive Unknown Input Observer Dynamic System Matricies
% Sam Nazari
% Jan 2018
clear
clc

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

%% Construct Fault Vectors
f1 = [1 0 0 0 0 0 0]'  % Vertex one is the intruder
f2 = [0 1 0 0 0 0 0]'  % Vertex two is the intruder
f3 = [0 0 1 0 0 0 0]'  % Vertex three is the intruder
f4 = [0 0 0 1 0 0 0]'  % Vertex four is the intruder
f5 = [0 0 0 0 1 0 0]'  % Vertex five is the intruder
f6 = [0 0 0 0 0 1 0]'  % Vertex six is the intruder
f7 = [0 0 0 0 0 0 1]'  % Vertex seven is the intruder

%E = [f1 f2 f3 f4 f5 f6 f7]
%E = [f2 f3 f4 f5 f6 f7]
E = f2

% Choose the agent to be attacked
flt1  = 0;
flt2  = 0;
flt3  = 0;
flt4  = 1;
flt5  = 0;
flt6  = 0;
flt7  = 0;

% Choose a magnitude for the attack
f1Val = 10;
f2Val = 10;
f3Val = 10;
f4Val = 10;
f5Val = 10;
f6Val = 10;
f7Val = 10;

% Chose the attack time
tf1   = 2;
tf2   = 2;
tf3   = 2;
tf4   = 2;
tf5   = 2;
tf6   = 5;
tf7   = 7;

%% PUIO 1: vertex 1 is endowed with detection filter.

% This UIO is insensitive to faults in agent 3, but can detect faults in agents 2 and 4:
bf1 = [0 0 1 0 0 0 0]' 

% Observer matrices
c1 = [0 1 0 0 0 0 0;
      0 0 1 0 0 0 0;
      0 0 0 1 0 0 0]

% Step one: Check rank
rank(c1*bf1)
rank(bf1)

% Step two: Check the condition of Lemma (3.2). If a nonnegative left inverse 
%  of CEa exists, then compute N=Ea(C*Ea)^g >= 0 and T = I-N*C >= 0
CE = c1*E
CEg= pinv(CE)
N = E*CEg
NC = N*c1
T  = eye(size(A)) - NC

% Step three: Define A1 = A - N*C*A so that {A1, C} is observable
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
% Step four: Place the poles of F = A1-G1*C
%p = [-1,-10,-20,-3,-5,-7,-9]
%G1=place(A1',c1',p)'
%F = A1-G1*c1
%eig(F)
tol=1e-9;
n1 = size(A1,1);
p1 = size(c1,1);
warning off;
cvx_begin sdp
variable P(n1,n1) 
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
G = G1 + A1*N - G1*c1*N;
%H = T*B;
M = eye(size(A));
% Step five: finish the PUIO
%G = G1 + A1*N - G1*c1*N
% H = T*B
H = 0

% Simulation results
%
% Set up simulation initial condiditons
x1_0 = 1;
x2_0 = 0.5;
x3_0 = 0.25;

d = 10;
TSIM = 50;
TH = 5 % detection Threshold
% Sim the system
sim('PUIO')

% Plot the results
figure(1)
subplot(311),plot(fn2,'k','lineWidth',1),xlabel('Time in seconds'),
legend('Residual Sig.'),grid on,ylim([0,TH]),xlim([0,5])
ylabel('Agent 2')
title('Distributed PUIO with Fault Occurance at t=2 in Agent 2')
subplot(312),plot(fn3,'b','lineWidth',1),xlabel('Time in seconds'),
legend('Residual Sig.'),grid on,ylim([0,TH]),xlim([0,5])
ylabel('Agent 3'),
subplot(313),plot(fn4,'r','lineWidth',1),grid on,ylim([0,TH]),xlim([0,5])
xlabel('Time in seconds'),
legend('Residual Sig.')
ylabel('Agent 4'),title(''),

%% Construct Fault Vectors for second fault instance
f1 = [1 0 0 0 0 0 0]'  % Vertex one is the intruder
f2 = [0 1 0 0 0 0 0]'  % Vertex two is the intruder
f3 = [0 0 1 0 0 0 0]'  % Vertex three is the intruder
f4 = [0 0 0 1 0 0 0]'  % Vertex four is the intruder
f5 = [0 0 0 0 1 0 0]'  % Vertex five is the intruder
f6 = [0 0 0 0 0 1 0]'  % Vertex six is the intruder
f7 = [0 0 0 0 0 0 1]'  % Vertex seven is the intruder

%E = [f1 f2 f3 f4 f5 f6 f7]
%E = [f2 f3 f4 f5 f6 f7]
E = f5

% Choose the agent to be attacked
flt1  = 0;
flt2  = 0;
flt3  = 0;
flt4  = 0;
flt5  = 1;
flt6  = 0;
flt7  = 0;

% Choose a magnitude for the attack
f1Val = 10;
f2Val = 10;
f3Val = 10;
f4Val = 10;
f5Val = 10;
f6Val = 10;
f7Val = 10;

% Chose the attack time
tf1   = 2;
tf2   = 2;
tf3   = 2;
tf4   = 2;
tf5   = 2;
tf6   = 5;
tf7   = 7;

%% PUIO 2: agent 7

% This UIO is insensitive to faults in agent 4, but can detect faults in agents 5 and 6:
bf2 = [0 0 0 1 0 0 0]' 

% Observer matrices
c2 = [0 0 0 1 0 0 0;
      0 0 0 0 1 0 0;
      0 0 0 0 0 1 0]

% Step one: Check rank
rank(c2*bf2)
rank(bf2)

% Step two: Check the condition of Lemma (3.2). If a nonnegative left inverse 
%  of CEa exists, then compute N=Ea(C*Ea)^g >= 0 and T = I-N*C >= 0
CE = c2*E
CEg= pinv(CE)
N = E*CEg
NC = N*c2
T  = eye(size(A)) - NC

% Step three: Define A1 = A - N*C*A so that {A1, C} is observable
NCA = NC*A
A2  = A-NCA
rank(obsv(A2,c2))


if( rank(c2*E) ~= rank(E) )
    disp('rank(CE) is different from rank(E)!')
elseif( sum(sum(N<0)) > 0 )
    disp('N is not positive!')
% elseif( sum(sum(T<0)) > 0) 
%     disp('T=I-NC is not positive!')
elseif( rank(obsv(A2,c2)) < rank(A2) )
    disp('{A1,Caug} is not observable!')
else
    disp('ALL OK')
end
% Step four: Place the poles of F = A1-G1*C
%p = [-1,-10,-20,-3,-5,-7,-9]
%G1=place(A1',c1',p)'
%F = A1-G1*c1
%eig(F)
tol=1e-9;
n2 = size(A2,1);
p2 = size(c2,1);
warning off;
cvx_begin sdp
variable P(n1,n1) 
variable Y(n1,p1)
minimize( 1 )
subject to
P >= 0;
C1 = A2'*P + P*A2 - c2'*Y' - Y*c2;
C1 <= 0 ;
C2 = A2'*P - c2'*Y' + eye(n2);
C2(:) >= tol.*ones(n2*n2,1);
C3 = P*N*c2*A + Y*c2 ;
C3(:) >= tol.*ones(n2*n2,1);
cvx_solver sdpt3 %sedumi 
cvx_end

G1 = P\Y;
F = A2 - G1*c2;
G = G1 + A2*N - G1*c2*N;
%H = T*B;
M = eye(size(A));
% Step five: finish the PUIO
%G = G1 + A1*N - G1*c1*N
% H = T*B
H = 0

% Simulation results
%
% Set up simulation initial condiditons
x1_0 = 1;
x2_0 = 0.5;
x3_0 = 0.25;

d = 10;
TSIM = 50;
TH = 5 % detection Threshold
% Sim the system
sim('PUIO_2')

% Plot the results
figure(2)
subplot(311),plot(fn2,'k','lineWidth',1),xlabel('Time in seconds'),
legend('Residual Sig.'),grid on,ylim([0,TH]),xlim([0,5])
ylabel('Agent 4')
title('Distributed PUIO with Fault Occurance at t=2 in Agent 5')
subplot(312),plot(fn3,'b','lineWidth',1),xlabel('Time in seconds'),
legend('Residual Sig.'),grid on,ylim([0,TH]),xlim([0,5])
ylabel('Agent 5'),
subplot(313),plot(fn4,'r','lineWidth',1),grid on,ylim([0,TH]),xlim([0,5])
xlabel('Time in seconds'),
legend('Residual Sig.')
ylabel('Agent 6'),title(''),

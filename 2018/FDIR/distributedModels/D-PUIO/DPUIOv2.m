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
flt2  = 1;
flt3  = 0;
flt4  = 0;
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

%% PUIO 1

% This UIO is insensitive to faults in agent 3, but can detect faults in agents 2 and 4:
bf1 = [0 0 1 0 0 0 0]' 

% Observer matrices
c1 = [0 1 0 0 0 0 0;
      0 0 1 0 0 0 0;
      0 0 0 1 0 0 0]

%% Step one: Check rank
rank(c1*bf1)
rank(bf1)

%% Step two: Check the condition of Lemma (3.2). If a nonnegative left inverse 
%  of CEa exists, then compute N=Ea(C*Ea)^g >= 0 and T = I-N*C >= 0
CE = c1*E
CEg= pinv(CE)
N = E*CEg
NC = N*c1
T  = eye(7) - NC

%% Step three: Define A1 = A - N*C*A so that {A1, C} is observable
NCA = NC*A
A1  = A-NCA
rank(obsv(A1,c1))

%% Step four: Place the poles of F = A1-G1*C
p = [-1,-10,-20,-3,-5,-7,-9]
G1=place(A1',c1',p)'
F = A1-G1*c1
eig(F)

%% Step five: finish the PUIO
G = G1 + A1*N - G1*c1*N
% H = T*B
H = 0
M = eye(size(A));
 %% Simulation results
%
% Set up simulation initial condiditons
x1_0 = 1;
x2_0 = 0;
x3_0 = 0;

d = 10;
TSIM = 50;

%% Sim the system
sim('PUIO')

%% Plot the results
figure(1)
plot(tout,fn2,'b'),hold on
plot(tout,fn4,'g'),hold on
xlabel('Time in seconds'),ylabel('State Estimate Error')
title('Positive Dynamic System and Positive UIO Observer State Estimation Error')
legend('x_2','x_4')



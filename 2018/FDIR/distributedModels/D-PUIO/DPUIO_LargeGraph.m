%% Positive Unknown Input Observer Dynamic System Matricies
% Sam Nazari
% Jan 2018
clear
clc

%% Graph Structure
Ag = [
   0   0   1   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   1   0   0   0   0   0   0;
   0   0   0   1   0   1   0   0   0   0   0   0   0   0   0   1   0   0   0   1   0   0   0   0   0;
   1   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0;
   0   1   0   0   0   0   0   0   0   0   0   1   0   0   0   0   1   1   1   0   1   0   0   1   1;
   0   0   0   0   0   0   0   1   1   0   0   1   0   0   0   1   0   0   0   1   1   0   0   0   0;
   0   1   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0;
   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   1   0   0   1   0   0   0;
   0   0   0   0   1   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   1   0   0   0;
   0   0   0   0   1   0   0   0   0   0   0   0   0   1   0   0   0   0   0   0   0   0   1   0   0;
   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   1   0   0   0   0   0   0   1   1   0;
   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   1   0;
   0   0   0   1   1   0   0   0   0   0   0   0   1   0   0   0   1   1   0   0   0   0   0   0   0;
   0   0   0   0   0   0   0   0   0   0   0   1   0   0   0   1   0   0   0   0   1   0   0   0   0;
   0   0   0   0   0   0   0   0   1   0   0   0   0   0   0   1   1   1   0   0   0   1   0   0   0;
   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   1   0   0   0   1   1   0   0;
   0   1   0   0   1   0   0   0   0   1   0   0   1   1   0   0   1   0   1   0   0   0   0   0   0;
   0   0   0   1   0   0   0   0   0   0   0   1   0   1   0   1   0   0   0   0   0   0   0   0   0;
   0   0   0   1   0   0   0   0   0   0   0   1   0   1   1   0   0   0   0   0   0   0   0   0   1;
   1   0   0   1   0   0   1   0   0   0   0   0   0   0   0   1   0   0   0   0   0   0   1   0   1;
   0   1   0   0   1   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   1;
   0   0   0   1   1   0   0   0   0   0   0   0   1   0   0   0   0   0   0   0   0   0   1   1   0;
   0   0   0   0   0   0   1   1   0   0   0   0   0   1   1   0   0   0   0   0   0   0   0   0   0;
   0   0   0   0   0   0   0   0   1   1   0   0   0   0   1   0   0   0   1   0   1   0   0   0   0;
   0   0   0   1   0   0   0   0   0   1   1   0   0   0   0   0   0   0   0   0   1   0   0   0   0;
   0   0   0   1   0   0   0   0   0   0   0   0   0   0   0   0   0   1   1   1   0   0   0   0   0
   ];

n = length(Ag)
Deg = eye(n);
for i = 1:n
    Deg(i,i) = sum(Ag(i,:));
end

C = eye(n)

L = Deg-Ag

A = -L
%% %% Construct Fault Vectors: we are endowing vertex 22 with the detection filter. Its neighbors are {1,5,24}
f1 = zeros(25,1);f1(1) = 1;  % Vertex one is the intruder
f5 = zeros(25,1);f5(5) = 1;  % Vertex five is the intruder
f24 = zeros(25,1);f24(24) = 1; % Vertex twenty-four is the intruder

E = f5

% % Choose the agent to be attacked
flt1  = 0;
flt2  = 0;
flt3  = 0;
flt4  = 0;
flt5  = 1;  % FAULT
flt6  = 0;
flt7  = 0;
flt8  = 0;
flt9  = 0;
flt10 = 0;
flt11 = 0;
flt12 = 0;
flt13 = 0;
flt14 = 0;
flt15 = 0;
flt16 = 0;
flt17 = 0;
flt18 = 0;
flt19 = 0;
flt20 = 0;
flt21 = 0;
flt22 = 0;
flt23 = 0;
flt24 = 0;
flt25 = 0;

% 
% % Choose a magnitude for the attack
f1Val = 10;
f2Val = 10;
f3Val = 10;
f4Val = 10;
f5Val = 10;
f6Val = 10;
f7Val = 10;
f8Val = 10;
f9Val = 10;
f10Val = 10;
f11Val = 10;
f12Val = 10;
f13Val = 10;
f14Val = 10;
f15Val = 10;
f16Val = 10;
f17Val = 10;
f18Val = 10;
f19Val = 10;
f20Val = 10;
f21Val = 10;
f22Val = 10;
f23Val = 10;
f24Val = 10;
f25Val = 10;

% % Chose the attack time
tf1   = 2;
tf2   = 2;
tf3   = 2;
tf4   = 2;
tf5   = 2;
tf6   = 5;
tf7   = 7;
tf8   = 2;
tf9   = 2;
tf10   = 2;
tf11   = 2;
tf12   = 2;
tf13   = 5;
tf14   = 7;
tf15   = 2;
tf16   = 2;
tf17   = 2;
tf18   = 2;
tf19   = 2;
tf20   = 5;
tf21   = 7;
tf22   = 2;
tf23   = 2;
tf24   = 2;
tf25   = 2;
%% PUIO 1

% This UIO is insensitive to faults in agent 1, but can detect faults in agents 5 and 24:
bf1 = zeros(25,1); bf1(1)=1

% Observer matrices: the observer matrix for the monitoring agent 22
c1 = zeros(3,25);
c1(1) = 1;
c1(2,5) = 1;
c1(3,24)= 1;

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
    disp('{A1,c1} is not observable!')
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
%% Step five: finish the PUIO
%G = G1 + A1*N - G1*c1*N
% H = T*B
H = 0


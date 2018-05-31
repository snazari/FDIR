%% Distributed Proportional Integral Observer 
% Sam Nazari
% March 2018
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

Ea = f2

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

%% Construct Disturbance Vectors
d1 = [1 0 0 0 0 0 0]'  % Vertex one undergoes a disturbance
d2 = [0 1 0 0 0 0 0]'  % Vertex two undergoes a disturbance
d3 = [0 0 1 0 0 0 0]'  % Vertex three undergoes a disturbance
d4 = [0 0 0 1 0 0 0]'  % Vertex four undergoes a disturbance
d5 = [0 0 0 0 1 0 0]'  % Vertex five undergoes a disturbance
d6 = [0 0 0 0 0 1 0]'  % Vertex six undergoes a disturbance
d7 = [0 0 0 0 0 0 1]'  % Vertex seven undergoes a disturbance

Ed = d2

% Choose the agent to undergo a disturbance
dist1  = 0;
dist2  = 1;
dist3  = 0;
dist4  = 0;
dist5  = 0;
dist6  = 0;
dist7  = 0;

% Choose a magnitude for the disturbance
d1Val = 10;
d2Val = 10;
d3Val = 10;
d4Val = 10;
d5Val = 10;
d6Val = 10;
d7Val = 10;

% Chose the disturbance time
dt1   = 1;
dt2   = 1;
dt3   = 1;
dt4   = 1;
dt5   = 1;
dt6   = 4;
dt7   = 6;

%% PIO 1

% This UIO is insensitive to faults in agent 3, but can detect faults in agents 2 and 4:
bf1 = [0 1 0 0 0 0 0]' 

% Observer matrices
c1 = [0 1 0 0 0 0 0;
      0 0 1 0 0 0 0;
      0 0 0 1 0 0 0]

% Make sure assumption 1 and 2 are met:
% Ass 1: rank(c1*Ed) = rank(Ed)
% Ass 2: for every re{lambda} > 0 rank(P) = dim(A) + dim(d) where P = [A-lam I Ed;C 0]

fprintf('rank(c1) = %i \n',rank(c1))
fprintf('rank(Ed) = %i \n',rank(Ed))
fprintf('rank(c1*Ed) = %i\n',rank(c1*Ed))

%% Step two: Check the condition of Lemma (3.2). If a nonnegative left inverse 
%  of CEa exists, then compute N=Ea(C*Ea)^g >= 0 and T = I-N*C >= 0
% n  = size(A,1)
% p  = size(c1,1)
% qa = size(Ea,2)
% N = Ed*pinv(c1*Ed)
% T = eye(n)-N*c1
% H = 0
% J = T*Ea
% A1 = A - N*c1*A
% A1hat=[A1, J;zeros(qa,n+qa)]
% Chat=[c1, zeros(p,qa)]
% if( rank(c1*Ea) ~= rank(Ea) )
%     disp('rank(CEa) is different from rank(Ea)!')
% elseif( rank(c1*Ed) ~= rank(Ed) )
%     disp('rank(CEd) is different from rank(Ed)!')
% elseif( sum(sum(N<0)) > 0 )
%     disp('N is not positive!')
% % elseif( sum(sum(T<0)) > 0) 
% %     disp('T=I-NC is not positive!')
% elseif( rank(obsv(A1,c1)) < rank(A1) )
%     disp('{A1,Caug} is not observable!')
% else
%     disp('ALL OK')
% end
% %% Step four: Place the poles of F = A1-G1*C
% 
% tol=1e-9;
% n1 = size(A1hat,1);
% p1 = size(Chat,1);
% warning off;
% cvx_begin sdp
% variable P(n1,n1) 
% variable Y(n1,p1)
% minimize( 1 )
% subject to
% P >= 0;
% C1 = A1hat'*P + P*A1hat - Chat'*Y' - Y*Chat;
% C1 <= 0 ;
% C2 = A1hat'*P - Chat'*Y' + eye(n1);
% C2(:) >= tol.*ones(n1*n1,1);
% % C3 = P*N*Caug*Aaug + Y*Caug ;
% % C3(:) >= tol.*ones(n1*n1,1);
% cvx_solver sedumi %sdpt3 
% cvx_end
% 
% Lhat = P\Y
%     
% G1 = Lhat(1:n,:)
% %L = Lhat(n+1:end,:)
% D  = Lhat(n+1:end,:)
% F = A1-G1*c1 
% G2 = F*N
% M1 = -D*c1
% M2 = D*(eye(p)-c1*N)
% G = G1+G2
% H = 0
% R=0
% 
%  %% Simulation results
% %
% % Set up simulation initial condiditons
% x1_0 = 1;
% x2_0 = 0.5;
% x3_0 = 0.25;
% 
% d = 10;
% TSIM = 50;
% TH = 5 % detection Threshold
% %% Sim the system
% sim('PIUIO')
% 
% %% Plot the results
% figure(1)
% subplot(311),plot(fn2,'k','lineWidth',1),xlabel('Time in seconds'),
% legend('Residual Sig.'),grid on,ylim([0,TH]),xlim([0,5])
% ylabel('Agent 2')
% title('Distributed PUIO with Fault Occurance at t=2 in Agent 2')
% subplot(312),plot(fn3,'b','lineWidth',1),xlabel('Time in seconds'),
% legend('Residual Sig.'),grid on,ylim([0,TH]),xlim([0,5])
% ylabel('Agent 3'),
% subplot(313),plot(fn4,'r','lineWidth',1),grid on,ylim([0,TH]),xlim([0,5])
% xlabel('Time in seconds'),
% legend('Residual Sig.')
% ylabel('Agent 4'),title(''),
% 
% figure(2)
% subplot(711), plot(fhat.Data(:,1),'lineWidth',1),xlabel('Time in seconds'),ylabel('Fault Signal 1'),
% title('Fault Signal'),grid on
% legend('f1'),ylim([0,2])
% subplot(712), plot(fhat.Data(:,2),'lineWidth',1),xlabel('Time in seconds'),ylabel('Fault Signal 2'),
% legend('f2'),ylim([0,2]),grid on
% subplot(713), plot(fhat.Data(:,3),'lineWidth',1),xlabel('Time in seconds'),ylabel('Fault Signal 3'),
% legend('f3'),ylim([0,2]),grid on
% subplot(714), plot(fhat.Data(:,4),'lineWidth',1),xlabel('Time in seconds'),ylabel('Fault Signal 4'),
% legend('f4'),ylim([0,2]),grid on
% subplot(715), plot(fhat.Data(:,5),'lineWidth',1),xlabel('Time in seconds'),ylabel('Fault Signal 5'),
% legend('f5'),ylim([0,2]),grid on
% subplot(716), plot(fhat.Data(:,6),'lineWidth',1),xlabel('Time in seconds'),ylabel('Fault Signal 6'),
% legend('f6'),ylim([0,2]),grid on
% subplot(717), plot(fhat.Data(:,7),'lineWidth',1),xlabel('Time in seconds'),ylabel('Fault Signal 7'),
% legend('f7'),ylim([0,2]),grid on
% 
% 

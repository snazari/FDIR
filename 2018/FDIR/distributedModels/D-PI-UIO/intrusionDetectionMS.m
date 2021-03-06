%% Sam Nazari
%  Teixeira MS Thesis Intrusion Detection Model
%  06-Nov-2016
%  04-Jan-2017
%  12-Jan-2017

clear
clc

%% Simulation parameters
TSIM = 20;

% u1Val = 34.632;
% u2Val = 1641.6;
% u3Val = 29980;
% 
% X0 = [0.3412;525.7;525.7;496.2];
% 
% z110 = 518.6174;
% z410 = -51365.5370;
% z320 = 472.2;
% z420 = -18391.8;
% 
% fa1 = 0;
% fa2 = 0;
% fa3 = 1;
% 
% f = [fa1;fa2;fa3]
%% Dynamic system 

L = [
    3   -1  -1  -1  0   0   0;
    -1  4   -1  -1  -1  0   0;
    -1  -1  3   0   0   -1  0;
    -1  -1  0   3   0   0  -1;
    0   -1  0   0   2   0  -1;
    0   0   -1  0   0   2  -1;
    0   0   0  -1  -1  -1   3
    ]

A = -L

B = eye(7)
Bf=B;
Bf(2,2)=0

C = [
    0 0 0 0 0 0 0;
    0 1 0 0 0 0 0; 
    0 0 1 0 0 0 0;
    0 0 0 1 0 0 0;
    0 0 0 0 0 0 0;
    0 0 0 0 0 0 0;
    0 0 0 0 0 0 0
    ]

E = [1.0;20.758;0.0;0.0]

x0 = [0 1 1 0 0 0 0]

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
%E = [f1 f3 f5]
Ea  = [f2 f3 f4]

% Choose the agent to be attacked
flt1  = 0
flt2  = 0
flt3  = 0
flt4  = 1
flt5  = 0
flt6  = 0
flt7  = 0

% Choose a magnitude for the attack
f1Val = 10
f2Val = 10
f3Val = 10
f4Val = 10
f5Val = 10
f6Val = 10
f7Val = 10

% Chose the attack time
tf1   = 2
tf2   = 2
tf3   = 2
tf4   = 2
tf5   = 2
tf6   = 5
tf7   = 7

%% Construct Disturbance Vectors
d1 = [1 0 0 0 0 0 0]'  % Vertex one undergoes a disturbance
d2 = [0 1 0 0 0 0 0]'  % Vertex two undergoes a disturbance
d3 = [0 0 1 0 0 0 0]'  % Vertex three undergoes a disturbance
d4 = [0 0 0 1 0 0 0]'  % Vertex four undergoes a disturbance
d5 = [0 0 0 0 1 0 0]'  % Vertex five undergoes a disturbance
d6 = [0 0 0 0 0 1 0]'  % Vertex six undergoes a disturbance
d7 = [0 0 0 0 0 0 1]'  % Vertex seven undergoes a disturbance

%E = [f1 f2 f3 f4 f5 f6 f7]
%E = [f2 f3 f4 f5 f6 f7]
%E = [f1 f3 f5]
Ed = Ea
% Choose the agent to undergo a disturbance
dist1  = 0
dist2  = 0
dist3  = 0
dist4  = 1
dist5  = 0
dist6  = 0
dist7  = 0

% Choose a magnitude for the disturbance
d1Val = 10
d2Val = 10
d3Val = 10
d4Val = 10
d5Val = 10
d6Val = 10
d7Val = 10

% Chose the disturbance time
dt1   = 1
dt2   = 1
dt3   = 1
dt4   = 1
dt5   = 1
dt6   = 4
dt7   = 6

%% UIO 1

% This UIO is insensitive to faults in agent 2, but can detect faults in agents 3 and 4:
bf1 = [0 1 0 0 0 0 0]' 

% Rank conditions
rank(C*bf1)
rank(bf1)

% Observer matrices
c1 = [0 1 0 0 0 0 0;
      0 0 1 0 0 0 0;
      0 0 0 1 0 0 0]
  
N  = Ed*pinv(C*Ed)
T1 = eye(7) - N*C
J1 = T1*Ea*100
A11= A - N*C*A
R  = [1 0 0 0 0 0 0;0 1 0 0 0 0 0;0 1 1 0 0 0 0]
A1h= [A11 J1;zeros(3,3) R]
Ch = [C zeros(7,3)]
p  = [-1,-100,-3,-4,-5,-6,-7,-8,-9,-10]
rank(obsv(A1h,Ch))
Lhat = place(A1h',Ch',p')'
Ac = A1h - Lhat*Ch
eig(Ac)
G1 = Lhat(1:7,:)
L1 = Lhat(8:end,:)
F1 = A11 - G1*C
G2 = F1*N
G  = G1 + G2
M1 = -1*L1*C
M2 = L1*(eye(7)-C*N)
% CE = C*bf1
% CEin = inv(CE'*CE)
% H1=bf1*CEin*CE'
% T1 = eye(7)-H1*C
% A11 = T1*A
% rank(obsv(A11,C))
% k11 = place(A11,C,[-1,-2,-3,-4,-5,-6,-7])
% %F = A1-k1*C
% F1 = A+H1*C*L-k11*C
% k1 = k11 + F1*H1

%% UIO 2
% This UIO is insensitive to faults in agent 3, but can detect faults in agents 2 and 4:
% bf2 = [0 0 1 0 0 0 0]' 
% 
% % Rank conditions
% rank(C*bf2)
% rank(bf2)
% 
% % Observer matrices
% CE = C*bf2
% CEin = inv(CE'*CE)
% H2=bf2*CEin*CE'
% T2 = eye(7)-H2*C
% A12 = T2*A
% rank(obsv(A12,C))
% k12 = place(A12,C,[-1,-2,-3,-4,-5,-6,-7])
% %F = A1-k1*C
% F2 = A+H2*C*L-k12*C
% k2 = k12 + F2*H2
%  
% %% Sim & Plot
% 
% sim('TeixeiraModel')
% 
% figure,
% subplot(311),
% plot(fn2,'LineWidth',2),ylabel('r12'),xlabel('Time (sec)'),title('Residual Signal for Agent 2'), grid on, ylim([-5,5])
% subplot(312),
% plot(fn3,'r','LineWidth',2),ylabel('r13'),xlabel('Time (sec)'),title('Residual Signal for Agent 3'), grid on, ylim([-5,5])
% subplot(313),
% plot(fn4,'g','LineWidth',2),ylabel('r14'),xlabel('Time (sec)'),title('Residual Signal for Agent 4'), grid on, ylim([-5,5])
% 
% figure,
% subplot(311),
% plot(tout,y1R,'k'),ylabel('y1_R'),xlabel('Time (sec)'),title('Outputs: y1_R, y2_R, y3_R')
% subplot(312),
% plot(tout,y2R,'b'),ylabel('y2_R'),xlabel('Time (sec)')
% subplot(313),
% plot(tout,y3R,'r'),ylabel('y3_R'),xlabel('Time (sec)')
% 
% figure,
% plot(tout,r11,'k'),ylabel('r_1^1'),xlabel('Time (sec)')
% title('Residual from UIO 1')
% 
% figure,
% plot(tout,r12,'k'),ylabel('r_1^2'),xlabel('Time (sec)')
% title('Residual from UIO 2')
% 
% figure
% ax1=subplot(411),plot(tout,y1R*T1(1,1),'k'),ylabel('y1R x T_1(1,1)')
% grid on
% ax2=subplot(412),plot(tout,y2R,'r'),ylabel('y2R')
% grid on
% ax3=subplot(413),plot(tout,z11,'b'),ylabel('z_1^1'),
% grid on
% ax4=subplot(414),plot(tout,r11,'g'),ylabel('r_1^1'),
% grid on
% linkaxes([ax1,ax2,ax3,ax4],'x')
% 
% figure,
% ax1=subplot(211),plot(tout,r11,'k'),ylabel('r_1^1'),
% title('Residual from UIO 1 and UIO 2'),xlabel('Time (hr)')
% grid on
% ax2=subplot(212),plot(tout,r12,'b'),ylabel('r_1^2')
% xlabel('Time (hr)'),grid on
% linkaxes([ax1,ax2],'x')
% figure
% ax1=subplot(411),plot(tout,y1R*T1(1,1),'k'),ylabel('y1R x T_1(1,1)')
% grid on
% ax2=subplot(412),plot(tout,y2R,'r'),ylabel('y2R')
% grid on
% ax3=subplot(413),plot(tout,z11,'b'),ylabel('z_1^1'),
% grid on
% ax4=subplot(414),plot(tout,r11,'g'),ylabel('r_1^1'),
% grid on
% linkaxes([ax1,ax2,ax3,ax4],'x')

% 
% figure,
% plot(tout,r2)
     
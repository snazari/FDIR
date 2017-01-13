%% Sam Nazari
%  Teixeira MS Thesis Intrusion Detection Model
%  06-Nov-2016
%  04-Jan-2017
%  12-Jan-2017

clear,
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


bf = [0 1 0 0 0 0 0]'

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
E = [f1 f3 f5]
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


%% Rank conditions
rank(C*bf)
rank(bf)
%% UIO 1

CE = C*bf
CEin = inv(CE'*CE)
H=bf*CEin*CE'
T = eye(7)-H*C
A1 = T*A

rank(obsv(A1,C))
k1 = place(A1,C,[-1,-2,-3,-4,-5,-6,-7])
%F = A1-k1*C
F = A+H*C*L-k1*C
k = k1 + F*H

%% UIO 2

%% UIO 3
 
%% Sim & Plot

% sim('ChenEx3p3p3_final')
% 
% figure,
% subplot(311),
% plot(tout,u1R,'k'),ylabel('u1_R'),xlabel('Time (sec)'),title('Inputs: u1_R, u2_R, u3_R')
% subplot(312),
% plot(tout,u2R,'b'),ylabel('u2_R'),xlabel('Time (sec)')
% subplot(313),
% plot(tout,u3R,'r'),ylabel('u3_R'),xlabel('Time (sec)')
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
     
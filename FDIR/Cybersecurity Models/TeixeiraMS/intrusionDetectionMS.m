%% Sam Nazari
%  Teixeira MS Thesis Intrusion Detection Model
%  06-Nov-2016

clear,
clc

%% Simulation parameters
TSIM = 20;

u1Val = 34.632;
u2Val = 1641.6;
u3Val = 29980;

X0 = [0.3412;525.7;525.7;496.2];

z110 = 518.6174;
z410 = -51365.5370;
z320 = 472.2;
z420 = -18391.8

fa1 = 0;
fa2 = 0;
fa3 = 1;

f = [fa1;fa2;fa3]
%% Dynamic system 

L = [
    3   -1  -1  -1  0   0   0;
    -1  4   -1  -1  -1  0   0;
    -1  -1  3   0   0   -1  0;
    -1  -1  0   3   0   0  -1;
    0   -1  0   0   2   0  -1;
    0   0   0   -1  -1  -1  3
    ]

A = -L

Bf = [
    1 0 0;
    0 1 0; 
    0 0 1
    ]

%C = [1 0 0 0;0 1 0 0; 0 0 1 0]

%E = [1.0;20.758;0.0;0.0]

%% UIO 1
% H1 = [
%       21.758        -1.0        0.0;
%       0.0           1.0         0.0;
%       0.0           0.0         1;
%       -2075.8       100.0       0.0
%       ]
% 
% T1 = [
%       -20.758   1   0   0;
%       0.0       0   0   0;
%       0         0   0   0;
%       2075.8    -100 0  1
%       ]
% 
% F1 = [
%      -10        0.0         0.0         0.0702;
%      0.0        -20         0.0         0.0;
%      0.0        0.0         -30         0.0;
%      0.0        0.0         0.0         -8.4325
%      ]
%  
%  K1 = [
%         -278.5724       13.3496     0.0;
%         0.0             0.0         0.0;
%         0.0             0.0         0.0;
%         10031.304       -475.5956   0.7781
%         ]
%     
% ICH1 =eye(3)-C*H1

%% UIO 2
% H2 = [
%         1   0   0;
%         0   1   0;
%         0   0   0;
%         0   0   40
%         ]
%     
%  T2 = [
%         0   0   0   0;
%         0   0   0   0;
%         0   0   1   0;
%         0   0   -40 0
%         ]
%     
% F2 =  [
%         -20     0       0       0;
%         0       -30     0       0;
%         0       0       -10     0.2588;
%         0       0       0       -11.7645
%         ]
%     
% K2 =    [
%             0       0       0;
%             0       0       0;
%             0       0       -15.9068;
%             0       0.6344  980.5501
%          ]
% 
%  ICH2 = eye(3)-C*H2
 
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
     
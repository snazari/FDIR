%% Plot1
clear all;
close all;
clc;
%%
% xDot = A x + B u + Ed d + Ea fa
% y = C x + Es fs
% A=[-1 1 1;1 -2 0;0 1 -3];
% B=[1;1;0];
% C=[1 0 1;0 1 0];
A=[-1 1 1;1 -2 0;0 1 -3];
B=[1;1;0];
C=[1 0 1;0 1 0];

x0=[1;2;3];

n = size(B,1);
m = size(B,2);
p = size(C,1);
%v0=[0.1;0.2]; % this is used to build x0aug=[x0;v0]
%z0=[0.1;0.2;0.3;0.4;0.5];
d=1;
Ed=[1;0;1];
fa_amp=50;fa_freq=1; % Sinosuid Fault Actuator Fault in simulink!
Ea=[1;0;0];
xhat0=[2;1;1];
fahat0=[1];

tEnd=40;
method = 2; % 1 is pole placement, 2 is LMI
eta=.15;
R=0;

z0=xhat0;
w0=[1;1];

%%
% Finding Observer
qa = size(Ea,2);
N = Ed* pinv(C*Ed);
T = eye(n)-N*C;
H = T*B;
J = T*Ea;
A1 = A - N*C*A;
A1hat=[A1, eta.*J;zeros(qa,n),R];
Chat=[C, zeros(p,qa)];
%%
if(method==1)
    % Pole Placement
    observerPoles = 2.*[-3,-3,-6,-10];
    Lhat = (place(A1hat',Chat',observerPoles))';
elseif(method==2)
    % LMI
    tol=1e-9;
    n1 = size(A1hat,1);
    p1 = size(Chat,1);
    warning off;
    cvx_begin sdp
    variable P(n1,n1) diagonal
    variable Y(n1,p1)
    minimize( 1 )
    subject to
    P >= 0;
    C1 = A1hat'*P + P*A1hat - Chat'*Y' - Y*Chat;
    C1 <= 0 ;
    C2 = A1hat'*P - Chat'*Y' + eye(n1);
    C2(:) >= tol.*ones(n1*n1,1);
    % C3 = P*N*Caug*Aaug + Y*Caug ;
    % C3(:) >= tol.*ones(n1*n1,1);
    cvx_solver sedumi %sdpt3 
    cvx_end

    Lhat = P\Y;
end
    
G1 = Lhat(1:n,:);
L = Lhat(n+1:end,:);
F = A1-G1*C; 
G2 = F*N;
M1 = -L*C;
M2 = L*(eye(p)-C*N);
G = G1+G2;
%%

% Simulating
sim('PIUIOModelZWSin');
yTS = yout.getElement(1);y = yTS.Values.Data;t = yTS.Values.Time;
faTS = yout.getElement(4);fa = faTS.Values.Data;
fahatTS = yout.getElement(6);fahat = fahatTS.Values.Data;
% Plotting
figure()
hold on; box on;
subplot(3,1,1);plot(t,fa(:,1),'k',t,fahat(:,1),'r--');
legend({'$f_a(t)$','$w(t)$'},'Interpreter','latex','FontSize',14)
hold off


%% Plot2
eta=.093;
R=0;

A1hat=[A1, eta.*J;zeros(qa,n),R];
Chat=[C, zeros(p,qa)];

if(method==1)
    % Pole Placement
    observerPoles = 2.*[-3,-3,-6,-10];
    Lhat = (place(A1hat',Chat',observerPoles))';
elseif(method==2)
    % LMI
    tol=1e-9;
    n1 = size(A1hat,1);
    p1 = size(Chat,1);
    warning off;
    cvx_begin sdp
    variable P(n1,n1) diagonal
    variable Y(n1,p1)
    minimize( 1 )
    subject to
    P >= 0;
    C1 = A1hat'*P + P*A1hat - Chat'*Y' - Y*Chat;
    C1 <= 0 ;
    C2 = A1hat'*P - Chat'*Y' + eye(n1);
    C2(:) >= tol.*ones(n1*n1,1);
    % C3 = P*N*Caug*Aaug + Y*Caug ;
    % C3(:) >= tol.*ones(n1*n1,1);
    cvx_solver sedumi %sdpt3 
    cvx_end

    Lhat = P\Y;
end
    
G1 = Lhat(1:n,:);
L = Lhat(n+1:end,:);
F = A1-G1*C; 
G2 = F*N;
M1 = -L*C;
M2 = L*(eye(p)-C*N);
G = G1+G2;


% Simulating
sim('PIUIOModelZWSin');
yTS = yout.getElement(1);y = yTS.Values.Data;t = yTS.Values.Time;
faTS = yout.getElement(4);fa = faTS.Values.Data;
fahatTS = yout.getElement(6);fahat = fahatTS.Values.Data;
% Plotting
hold on; box on;
subplot(3,1,2);plot(t,fa(:,1),'k',t,fahat(:,1),'r--');
legend({'$f_a(t)$','$w(t)$'},'Interpreter','latex','FontSize',14)

hold off;


%% Plot3
eta=0.15;
R=-3;

A1hat=[A1, eta.*J;zeros(qa,n),R];
Chat=[C, zeros(p,qa)];

if(method==1)
    % Pole Placement
    observerPoles = 2.*[-3,-3,-6,-10];
    Lhat = (place(A1hat',Chat',observerPoles))';
elseif(method==2)
    % LMI
    tol=1e-9;
    n1 = size(A1hat,1);
    p1 = size(Chat,1);
    warning off;
    cvx_begin sdp
    variable P(n1,n1) diagonal
    variable Y(n1,p1)
    minimize( 1 )
    subject to
    P >= 0;
    C1 = A1hat'*P + P*A1hat - Chat'*Y' - Y*Chat;
    C1 <= 0 ;
    C2 = A1hat'*P - Chat'*Y' + eye(n1);
    C2(:) >= tol.*ones(n1*n1,1);
    % C3 = P*N*Caug*Aaug + Y*Caug ;
    % C3(:) >= tol.*ones(n1*n1,1);
    cvx_solver sedumi %sdpt3 
    cvx_end

    Lhat = P\Y;
end
    
G1 = Lhat(1:n,:);
L = Lhat(n+1:end,:);
F = A1-G1*C; 
G2 = F*N;
M1 = -L*C;
M2 = L*(eye(p)-C*N);
G = G1+G2;


% Simulating
sim('PIUIOModelZWSin');
yTS = yout.getElement(1);y = yTS.Values.Data;t = yTS.Values.Time;
faTS = yout.getElement(4);fa = faTS.Values.Data;
fahatTS = yout.getElement(6);fahat = fahatTS.Values.Data;
% Plotting
hold on; box on;
subplot(3,1,3);plot(t,fa(:,1),'k',t,fahat(:,1),'r--');
legend({'$f_a(t)$','$w(t)$'},'Interpreter','latex','FontSize',14)

hold off;


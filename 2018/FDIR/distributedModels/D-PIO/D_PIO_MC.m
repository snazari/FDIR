%% Distributed Proportional Integral Observer 
% Sam Nazari
% March 2018
clear
clc

% Graph Structure
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

C = eye(7);

L = Deg-Ag;

x0 = [0 1 1 0 0 0 0];

A = -L;

% Condition: Distinct Eigenvalues
%eig(A)
lam = 10;
warning('off')
fprintf('Beginning MC simulation with lam = %i ',lam)
for run_num = 1:1000
    
    n = rand();

    if 0.5 < n < 0.55
        a=0;
        b=0;
        c=0;
    elseif n <= 0.5
        a=0;
        b=1;
        c=0
    elseif n>=0.75
        a=1;
        b=0;
        c=0;
    else
        a=0;
        b=0;
        c=1;
    end
    % Construct Fault Vectors
    f1 = [1 0 0 0 0 0 0]';  % Vertex one is the intruder
    f2 = [0 1 0 0 0 0 0]';  % Vertex two is the intruder
    f3 = [0 0 1 0 0 0 0]';  % Vertex three is the intruder
    f4 = [0 0 0 1 0 0 0]';  % Vertex four is the intruder
    f5 = [0 0 0 0 1 0 0]';  % Vertex five is the intruder
    f6 = [0 0 0 0 0 1 0]';  % Vertex six is the intruder
    f7 = [0 0 0 0 0 0 1]';  % Vertex seven is the intruder

    Ea = f2;

    % Choose the agent to be attacked
    flt1  = 0;
    flt2  = a;
    flt3  = b;
    flt4  = c;
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

    % Construct Disturbance Vectors
    d1 = [1 0 0 0 0 0 0]';  % Vertex one undergoes a disturbance
    d2 = [0 1 0 0 0 0 0]';  % Vertex two undergoes a disturbance
    d3 = [0 0 1 0 0 0 0]'; % Vertex three undergoes a disturbance
    d4 = [0 0 0 1 0 0 0]';  % Vertex four undergoes a disturbance
    d5 = [0 0 0 0 1 0 0]';  % Vertex five undergoes a disturbance
    d6 = [0 0 0 0 0 1 0]';  % Vertex six undergoes a disturbance
    d7 = [0 0 0 0 0 0 1]';  % Vertex seven undergoes a disturbance

    Ed = [d2,zeros(7,1) ,zeros(7,1)];


    % Choose the agent to undergo a disturbance
    dist1  = 0;
    dist2  = 0;
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

    % PIO 1

    % This PIO is designed to be embedded in agent 1 in order to monitor faults
    % in agents 2 and 3. This PIO monitors agent 2. 

    %bf1 = [0 1 0 0 0 0 0]' 

    % Observer matrices
    c1 = [0 1 0 0 0 0 0;
          0 0 1 0 0 0 0;
          0 0 0 1 0 0 0];

    % Make sure assumption 1 and 2 are met:
    % Ass 1: rank(c1*Ed) = rank(Ed)
    % Ass 2: for every re{lambda} > 0 rank(P) = dim(A) + dim(d) where P = [A-lam I Ed;C 0]

%     fprintf('rank(c1) = %i \n',rank(c1));
%     fprintf('rank(Ed) = %i \n',rank(Ed));
%     fprintf('rank(c1*Ed) = %i\n',rank(c1*Ed));

    sys1 = ss(A,zeros(7,1),c1,0);
    rank(obsv(sys1));
    LP = place(A,c1',[-1,-2,-3,-4,-5,-6,-7])';
    ALPC = A-LP*c1;
    LI = 10;
    ddot0 = 0;
    xdot0 = zeros(7,1)';
    TSIM = 10;
    sim('D_P_I_O');

    % Plot errors
%     stateEstimErr = logsout.getElement(1)
%     distEstimErr  = logsout.getElement(2)
%     yhatEstimErr  = logsout.getElement(3)
%     stateEstimErr.Name = 'State Estimation Error'
%     distEstimErr.Name = 'Fault Estimation Error'
%     yhatEstimErr.Name = 'Output Estimation Error'
%     figure
%     subplot(311)
%     title('State Estimation Error')
%     plot(stateEstimErr.Values,'LineWidth',2),grid on
%     subplot(312)
%     title('Fault Estimation Error')
%     plot(distEstimErr.Values,'LineWidth',2), grid on
%     subplot(313)
%     title('Output Estimation Error')
%     plot(yhatEstimErr.Values,'LineWidth',2), grid on

%     dhat.Name='Fault Estimate'
%     figure
%     title(dhat.Name)
%     plot(dhat, 'lineWidth',2),grid on
%     xlabel('Time (seconds)')
%     ylabel('Fault Estimate')
%     legend('Agent 2','Agent 3','Agent 4')

    agent_2_residual = dhat.Data(end,1);
    agent_3_residual = dhat.Data(end,2);
    agent_4_residual = dhat.Data(end,3);
    
    fault_2_pos = 0;
    fault_3_pos = 0;
    fault_4_pos = 0;
    
    if agent_2_residual > lam
        fault_2_pos = 1;
    end
    if agent_3_residual > lam
        fault_3_pos = 1;
    end
    if agent_4_residual > lam
        fault_4_pos = 1;
    end
    %% Save the results
    fileID = fopen('logs.txt', 'a');
    fprintf(fileID, '%i %i %i %i %f %f %f %f %i %i %i\n', [run_num, a,b,c,agent_2_residual, agent_3_residual, agent_4_residual, lam, fault_2_pos, fault_3_pos, fault_4_pos]);
    fclose(fileID);
end
fprintf('\n')
fprintf('Done! \n')
%% PIO Test
% A  = [-2 0;0 -3]
% B  = [1;-1]
% C  = eye(2)
% E  = [1;0]
% Ax = [A E;zeros(1,3)]
% Bx = [B;0]
% Cx = [C zeros(2,1)]
% sys= ss(Ax,Bx,Cx,0)
% rank(obsv(sys))
% 
% warning off;
% cvx_begin sdp
% variable P(3,3)
% variable G(3,2)
% minimize( 1 )
% subject to
% P>=0;
% C1 = Ax'*P-Cx'*G'+P*Ax-G*Cx;
% C1<=0;
% cvx_solver sedumi
% cvx_end
% 
% LI = place(A,C',[-1,-2])
% LP = 10

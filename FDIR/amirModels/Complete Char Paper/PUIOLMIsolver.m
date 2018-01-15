function [P,Y] = PUIOLMIsolver(A,C,N,method,AA)
n = size(A,1);
p = size(C,1);
tol=1e-9;

warning off;
cvx_begin sdp

% Defining Variables
variable P(n,n) diagonal
variable Y(n,p) 

% Defining Objective Function
minimize( 1 )
subject to

% Defining LMI Constraints
P >= 0; % making P PD

% Stability Term
C1 = A'*P + P*A - C'*Y' - Y*C;
C1 <= 0 ;

% Positivity Terms
% F being Metzler

C2 = A'*P - C'*Y' + eye(n);
C2(:) >= tol.*ones(n*n,1); 

%if(method==1)
    % G being positive
%     C3 = Y + P*A*N - Y*C*N ;
%     C3(:) >= zeros(n*p,1);
%     disp('method 1 used')
% elseif(method==2)
    C3 = P*N*C*AA + Y*C ;
    C3(:) >= tol.*ones(n*n,1);
%    disp('method 2 used')
%end

% Choosing Solver
cvx_solver sdpt3 %sedumi 

cvx_end

end
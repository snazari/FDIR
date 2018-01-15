function [P,Y] = OLMIsolver(A,C)
n = size(A,1);
p = size(C,1);

warning off;
cvx_begin sdp quiet

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
%C2 = A'*P - C'*Y' + eye(n);
%C2(:) >= zeros(n*n,1); 
% G being positive
%C3 = Y*C ;
%C3(:) >= zeros(n*n,1);

% Choosing Solver
cvx_solver sdpt3 %sedumi

cvx_end

end
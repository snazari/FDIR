function [F,G,H,M,N,P,Y,error,A1] = PUIOfinder(A,B,C,E,method)
% two methods; method1: theorem 3.1 with G>0 AND method2: theorem 3.2 with any G and xhat>0
error=0;
F=0;G=0;H=0;M=0;N=0;

% if( rank(C*E) ~= rank(E) )
%     disp('rank(CE) is different from rank(E)!')
%     error = 1;
%     return;
% end
N = E* pinv(C*E);
T = eye(size(A))-N*C;
% if( sum(sum(N<0)) > 0 )
%     disp('N is not positive!')
%     N
%     error = 1;
%     return;
% elseif( sum(sum(T<0)) > 0 && method==1) 
%     disp('T=I-NC is not positive!')
%     T
%     error = 1;
%     return;
% end

A1 = A - N*C*A;

% if ( rank(obsv(A1,C)) < rank(A1) )
%     disp('{A1,C} is not observable!')
%     error=1;
%     return;
% end

[P,Y] = PUIOLMIsolver(A1,C,N,method,A);
G1 = P\Y;
F = A1 - G1*C;
G = G1 + A1*N - G1*C*N;
H = T*B;
M = eye(size(A));

end

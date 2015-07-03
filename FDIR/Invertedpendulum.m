%----System Matrices----%
A=[0 1 0 0;19.6 0 0 0;0 0 0 1;-9.8 0 0 0];
B=[0;-1;0;1];
C=eye(4);
E=[1;1;1;1];
f=zeros(4,4);
v=[-4 -4 -4 -4];
F=diag(v);

%---------LQR design----------%
Q=[.25 0 0 0;0 1 0 0;0 0 4 0;0 0 0 1];
R=.0003;
[K l s]=lqr(A,B,Q,R);

%-------UIO Design------------%
H=E*(inv((C*E)'*(C*E)))*(C*E)';
T=eye(4)-H*C;
k1=inv(C)*(A-F-H*C*A);
k2=F*H;
k=k1+k2;

%------Design of bank UIOs each insensitive to One  sensor Fault-----%
%----UIO-1---%
k11=k1;
H1=zeros(4,4);
H1(1:4,2:4)=.25;
C1=C;
C1(:,1)=0;
T1=eye(4)-H1*C1;
F1=T1*A-k11*C1;
k12=F1*H1;
K1=k11+k12;

%----UIO-2----%
k21=k1;
H2=zeros(4,4);
H2(1:4,[1,3,4])=.25;
C2=C;
C2(:,2)=0;
T2=eye(4)-H2*C2;
F2=T2*A-k21*C2;
k22=F2*H2;
K2=k21+k22;

%----UIO-3----%
k31=k1;
H3=zeros(4,4);
H3(1:4,[1,2,4])=.25;
C3=C;
C3(:,3)=0;
T3=eye(4)-H3*C3;
F3=T3*A-k31*C3;
k32=F3*H3;
K3=k31+k32;

%---UIO-4----%
k41=k1;
H4=zeros(4,4);
H4(1:4,1:3)=.25;
C4=C;
C4(:,4)=0;
T4=eye(4)-H4*C4;
F4=T4*A-k41*C4;
k42=F4*H4;
K4=k41+k42;
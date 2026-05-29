clear;
close all;
clc;
pkg load control
%% Component parameters
R1=15000; %ohms
R2=15000; %ohms
C1=100e-9;%farads
C2=100e-9;%farads

%% States space
A=[-((1/(R1*C1))+(1/(R2*C1))),1/(R2*C1);
    1/(R2*C2),-1/(R2*C2)]
B=[1/(R1*C1);0]
Q=[1,0;0,5] % 2x2 matrix that weights states
R=[1.0]% Positive scalar that wheights control effort
C=[0,1]
%Solve Riccati: AˆT P + P A - P B Rˆ-1 BˆT P + Q = 0
% using control pkg
% K= Feedback control gains, P= Ricatti solution, eig= pole positions
[K, P, eig] = lqr(A,B,Q,R)

%% System response
%% Closed-loop system
Ts = 1e-6;
t = 0:Ts:0.02;
x = zeros(2,length(t));
u = zeros(1,length(t));
N = [[A,B];[C,0]]\[0;[1;1]]; %N(1:2)=Nx, N(3)=Nu
Nbar = (N(3)+K*N(1:2));
r=Nbar*1.0;
for k=2:length(t)
x(:,k)=x(:,k-1)+Ts*(A*x(:,k-1)+B*u(k-1));
u(k)=r-K*x(:,k);
if u(k)>3.3
  u(k)=3.3;
elseif u(k)<0
  u(k)=0;
endif

endfor
%% State response
figure
grid on
title('Closed-loop state response')
subplot(3,1,1)
plot(t,x(1,:))
title("X1")
subplot(3,1,2)
plot(t,x(2,:))
title("X2")
subplot(3,1,3)
plot(t,u)
title("U")


%Integral LQR
Ai=[[0,-C];[[0;0],A]];
Bi=[0;B];
Qi=[0.01,[0,0];[0;0],Q];
[Ki, Pi, eigi] = lqr(Ai,Bi,Qi,R)

%% System response
%% Closed-loop system
Ts=0.0001;
t= 0:Ts:0.02;
x = zeros(2,length(t));
xi = zeros(1,length(t));
u = zeros(1,length(t));
for k=2:length(t)
x(:,k) = x(:,k-1)+Ts*(A*x(:,k-1)+B*u(k-1));
y = C*x(:,k);
xi(k)=xi(k-1)+Ts*(y-r);
u(k)=r-Ki(2:3)*x(:,k)-Ki(1)*xi(k);
if u(k)>3.3
  u(k)=3.3;
elseif u(k)<0
  u(k)=0;
endif
endfor
%% State response
figure
grid on
title('Closed-loop state response')
subplot(4,1,1)
plot(t,x(1,:))
title("X1")
subplot(4,1,2)
plot(t,x(2,:))
title("X2")
subplot(4,1,3)
plot(t,xi)
title("Xi")
subplot(4,1,4)
plot(t,u)
title("U")

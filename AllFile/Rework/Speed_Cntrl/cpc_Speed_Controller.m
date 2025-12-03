%value
M = 4.676;       %kg% Mass of the cart           
m = 0.098;     %kg% Mass of the pendulum
d = 0.285;     %m% length of the pendulum rope
g = 9.8;     %m/s^2% gravity
c=5;       %friction constant

%state space

%System Matrix
A = [0,(-g*(M+m))/(M*d),c/M*d,0;1,0,0,0;0,(m*g)/M,-c/M,0;0,0,1,0];
%Input Matrix
B = [-1/(M*d);0;1/M;0 ]
%Output Matrix
C=eye(4);
%Feed-through Matrix
D=[0;0;0;0];
% Desired Poles
pole = -4
alpha= poly([pole pole pole pole]);
%ct = C(4,:);

% proportional controller
% Controllability matrix
s =[B,A*B,A^2*B,A^3*B];
% Rank of Controllability matrix 
r = rank(s);
% Inverse of Controllability matrix
IS=inv(s);
% Last Row of Inverse Controllability matrix
q = IS(4,1:4);
% Desired Poles
%pole = -1
alpha= poly([pole pole pole pole]);
% output matrix
ct = C(4,:);
% Ackermann?s formula
k = q*(alpha(5)*(eye(4))+ (alpha(4)*(A))+ (alpha(3)*(A^2)) + (alpha(2)*(A^3))+ +(A^4));
% Proportional gain (Preamplifer Gain)
P=1/(ct*((B*k-A)^-1)*B);

%Pi controller with state feedback
% System Matrix
A_pi = [A [0;0;0;0]; -ct 0];
% Input Matrix
B_pi = [B;0];
% Output Matrix
C_pi = eye(5);
% Feed-through Matrix
D_pi = [0;0;0;0;0];
%Controllability Matrix
S_pi =[B_pi A_pi*B_pi A_pi^2*B_pi A_pi^3*B_pi A_pi^4*B_pi ];
%Rank of Inverse Controllability Matrix
RI = rank(S_pi);
%Inverse Controllability Matrix 
S_pi_inv = inv(S_pi);
% Last Row of Inverse Controllability matrix 
Q_pi = S_pi_inv(5,1:5);
% Desired poles
alpha1= poly([pole pole pole pole pole]);
% Ackermann's formula for continous system with integrator
k_pi = Q_pi*(alpha1(6)*eye(5)+ alpha1(5)*A_pi + alpha1(4)*A_pi^2 + alpha1(3)*A_pi^3+ alpha1(2)*A_pi^4 + A_pi^5);
%Value of state-feedback
ksf =  k_pi(1:4)-(P*ct);
% Gain of Integral
PI = -k_pi(5);


% Continuous to discrete system
Ts = 10e-3
sys1=c2d(ss(A,B,C,D),Ts); 
% Accessing data of discrete system without integrator 
[AD,BD,CD,DD]=ssdata(sys1);
% Discrete Controllability matrix 
S_D = [BD AD*BD AD^2*BD AD^3*BD]; 
% Discrete Inverse of Controllability matrix
Si_D = inv(S_D); 
% Discrete Last Row of Inverse Controllability matrix 
q_D = Si_D(4, 1:4); 
% Transfer function for desired poles 
tf_p=tf([1],poly([pole pole pole pole])); 
% Converting transfer function to discrete
tfd=c2d(tf_p,Ts); 
[zeros_d,poles_d]=tfdata(tfd,'v');
%Ackermann?s formula for discrete system
K_D = q_D*(poles_d(5)*eye(4)+poles_d(4)*AD+poles_d(3)*AD^2 + poles_d(2)*AD^3+poles_d(1)*AD^4); 
% Discrete Proportional gain value
P_D = 1/((ct*(eye(4)-(AD-BD*K_D))^-1)*BD)

%%Continuous to discrete system pi
% Discrete system with integrator
sys = ss(A_pi,B_pi,C_pi,D_pi,0);
sys_d = c2d(sys,Ts);
% Accessing data of discrete system
[Adi,Bdi,Cdi,Ddi] = ssdata(sys_d);
Sd=[Bdi Adi*Bdi (Adi^2)*Bdi (Adi^3)*Bdi (Adi^4)*Bdi];
Rd=rank(Sd); 
Sd_i=inv(Sd); 
qdi=Sd_i(5, 1:5);
% Converting transfer function to discrete 
tf_pi=tf(1,poly([pole pole pole pole pole]));
tfd=c2d(tf_pi,Ts); 
[zeros_d,poles_d]=tfdata(tfd,'v');
Ktd= qdi*(poles_d(6)*eye(5) + poles_d(5)*Adi + poles_d(4)*Adi^2 + poles_d(3)*Adi^3 + poles_d(2)*Adi^4 + Adi^5); 
kt_d = Ktd(1:4)-(P_D*ct)
pi_d = -Ktd(5)


Ts=10e-3;
P_0k=57147;
P_0=1;
F_0=20;
F_0k=200;
V_0=1;
V_0k=571.47;
A_0=2*pi;
A_0k=4096;
Av_0=2*pi;
Av_0k=40.96;
ktd_s(1)=(Av_0/Av_0k)*kt_d(1)*(F_0k/F_0)
ktd_s(2)=(A_0/A_0k)*kt_d(2)*(F_0k/F_0)
ktd_s(3)=(V_0/V_0k)*kt_d(3)*(F_0k/F_0)
ktd_s(4)=(P_0/P_0k)*kt_d(4)*(F_0k/F_0)
Gp_s=(P_0/P_0k)*P_D*(F_0k/F_0)
Gi_s=(P_0/P_0k)*pi_d*Ts*(F_0k/F_0)

P_SHIFT=0;
KP_SHIFT=15;
KI_SHIFT=22;
KTD_SHIFT_0=11;
KTD_SHIFT_1=14;
KTD_SHIFT_2=14;
KTD_SHIFT_3=14;


KP_D = round(Gp_s * 2^KP_SHIFT);
KI_D = round(Gi_s * 2^KI_SHIFT);
KTD_D(1) = round(ktd_s(1) * 2^KTD_SHIFT_0);
KTD_D(2) = round(ktd_s(2) * 2^KTD_SHIFT_1);
KTD_D(3) = round(ktd_s(3) * 2^KTD_SHIFT_2);
KTD_D(4) = round(ktd_s(4) * 2^KTD_SHIFT_3);


fprintf('----- Position controller fixed point scaling ------\n');
fprintf('#define P_D  %d  --> int 32.%d \n', P_0k,P_SHIFT);
fprintf('#define KP_D  %d  --> int 16.%d \n', KP_D,KP_SHIFT);
fprintf('#define KI_D  %d  --> int 16.%d \n', KI_D,KI_SHIFT);
fprintf('#define KTD_D: (%7d, %7d, %7d, %7d)  --> int 16.%d, int 16.%d, int 16.%d, int 16.%d \n', KTD_D(1),KTD_D(2),KTD_D(3),KTD_D(4),KTD_SHIFT_0,KTD_SHIFT_1,KTD_SHIFT_2,KTD_SHIFT_3);
fprintf('\n');

Pi=P_D*(c/M);
PD_s=(V_0/V_0k)*P_D*(F_0k/F_0);
PI_s=(V_0/V_0k)*Pi*Ts*(F_0k/F_0);

PD_SHIFT=15;
PI_SHIFT=15;
V_SHIFT=5;

PD_D = round(PD_s * 2^PD_SHIFT);
PI_D = round(PI_s * 2^PI_SHIFT);
V_D = round(V_0k * 2^V_SHIFT);
fprintf('----- Speed controller fixed point scaling ------\n');
fprintf('#define V_D  %d  --> int 16.%d \n', V_D,V_SHIFT);
fprintf('#define PD_D  %d  --> int 16.%d \n', PD_D,PD_SHIFT);
fprintf('#define PI_D  %d  --> int 16.%d \n', PI_D,PI_SHIFT);
fprintf('----- cut above ------\n');
disp('That`s all.')
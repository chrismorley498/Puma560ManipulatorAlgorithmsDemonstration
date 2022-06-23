%Dynamics of puma robot
syms q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 m1 m2 m3 l1 l2 l3 lc2 lc3 g;

J=0.302;%kg*m^2
J1=lc3^2*m3;
J12=lc2^2*m2+l2^2*m3;
J13=l2*lc3*m3;

%Define task space position
X=[cos(q1)*(l2*cos(q2)+l3*sin(q2+q3));sin(q1)*(l2*cos(q2)+l3*sin(q2+q3));l1-l2*sin(q2)+l3*cos(q2+q3)];

%Define the Jacobian
JacPuma=jacobian(X,[q1 q2 q3]);

%Define the H(q) Matrix
h11=J + J1*sin(q2+q3)^2 + J12*cos(q2)^2 + 2*J13*cos(q2)*sin(q2+q3);
h22=J1+J12+2*J13*sin(q3);
h33=J1;
h23=J1+J13*sin(q3);
h32=h23;
H=[h11 0 0;0 h22 h23;0 h32 h33];

%Define the C(q,dq) Matrix
c12_1=J1*sin(2*(q1+q2))-J12-sin(2*q2)+2*J13*cos(2*q2+q3);
c13_1=J1*sin(2*(q2+q3))+2*J13*cos(q2)*cos(q2+q3);
c11_2=0.5*(-J1*sin(2*(q2+q3)))+J12*sin(2*q2)-2*J13*cos(2*q2+q3);
c22_2=2*J13*cos(q3);
c23_2=J13*cos(q3);
c22_3=-J13*cos(q3)*dq2;%IS THIS CORRECT?
c11_3=-0.5*(J1*sin(2*(q2+q3))+J13*cos(q2)*cos(q2+q3));
C=[0 c12_1*dq1 c13_1*dq3;c11_2*dq1 0 c22_2*dq2+c23_2*dq3;c11_3*dq1 c22_3*dq2 0];

%Define the G(q) Matrix
G=[0;lc3*m3*sin(q2+q3)+lc2*m2*cos(q2)+l2*m3*cos(q2);lc3*m3*sin(q2+q3)]*-g;

%Define state and state derivative vectors
q=[q1;q2;q3];
dq=[dq2; dq2; dq3];
ddq=[ddq1;ddq2;ddq3];

%Need to define the dqr vector which is used for adaptive control
syms dqr1 dqr2 dqr3;
dqr=[dqr1;dqr2;dqr3];

%% Finding the regressor matrix for adaptive control below
%Note that here eveything must be parameteric in order for the formulation
%of the regrossor matrix to take place

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Therfore must reculated H,C & G with Inertias being parametric
syms J J1 J12 J13
%Define task space position
X=[cos(q1)*(l2*cos(q2)+l3*sin(q2+q3));sin(q1)*(l2*cos(q2)+l3*sin(q2+q3));l1-l2*sin(q2)+l3*cos(q2+q3)];
%Define the Jacobian
JacPuma=jacobian(X,[q1 q2 q3]);
%Define the H(q) Matrix
h11=J + J1*sin(q2+q3)^2 + J12*cos(q2)^2 + 2*J13*cos(q2)*sin(q2+q3);
h22=J1+J12+2*J13*sin(q3);
h33=J1;
h23=J1+J13*sin(q3);
h32=h23;
H=[h11 0 0;0 h22 h23;0 h32 h33];

%Define the C(q,dq) Matrix
c12_1=J1*sin(2*(q1+q2))-J12-sin(2*q2)+2*J13*cos(2*q2+q3);
c13_1=J1*sin(2*(q2+q3))+2*J13*cos(q2)*cos(q2+q3);
c11_2=0.5*(-J1*sin(2*(q2+q3)))+J12*sin(2*q2)-2*J13*cos(2*q2+q3);
c22_2=2*J13*cos(q3);
c23_2=J13*cos(q3);
c22_3=-J13*cos(q3)*dq2;%IS THIS CORRECT?
c11_3=-0.5*(J1*sin(2*(q2+q3))+J13*cos(q2)*cos(q2+q3));
C=[0 c12_1*dq1 c13_1*dq3;c11_2*dq1 0 c22_2*dq2+c23_2*dq3;c11_3*dq1 c22_3*dq2 0];

%Define the G(q) Matrix
G=[0;lc3*m3*sin(q2+q3)+lc2*m2*cos(q2)+l2*m3*cos(q2);lc3*m3*sin(q2+q3)]*-g;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Find expression for actuator torque
syms ddqr1 ddqr2 ddqr3;
ddqr=[ddqr1;ddqr2;ddqr3];
dynamicEquations=H*ddqr+C*dqr+G;
%Create a vector with all the different variables
vars=[ddq1, ddq2, ddq3,ddqr1,ddqr2,ddqr3, dq1, dq2, dq3, dqr1, dqr2, dqr3, q1, q2, q3,sin(q1),cos(q1),sin(q2),cos(q2),sin(q3),cos(q3),cos(q2+q3),sin(q2+q3),sin(2*q2+2*q3),sin(2*q1+2*q2),cos(2*q2+q3),sin(2*q2)];

%Collect all the different combinations of variables
[a1 Y1] = coeffs(dynamicEquations(1), vars);
[a2 Y2] = coeffs(dynamicEquations(2), vars);
[a3 Y3] = coeffs(dynamicEquations(3), vars);

%Compute the regressor matrix & a vector
Y=sym(zeros(3,size(cat(2,Y1,Y2,Y3),2)));
Y1_size=size(Y1,2);
Y2_size=size(Y2,2);
Y3_size=size(Y3,2);
Y(1,1:Y1_size)=Y1;
Y(2,Y1_size+1:Y1_size+Y2_size)=Y2;
Y(3,Y1_size+Y2_size+1:Y1_size+Y2_size+Y3_size)=Y3;
a=[transpose(a1);transpose(a2);transpose(a3)];

%Print a numerical value of aHat for testing
a_numerical=subs(a,[J,J1,J12,J13],[.302,lc3^2*m3,lc2^2*m2+l2^2*m3,l2*lc3*m3]);
a_numerical=vpa(subs(a_numerical,[g, l2, lc2, lc3, m2, m3],[9.81,.432,.216,.164,15.46,9.55]),4);


%% Calculate nurmeric matrix values for certain simulink implementations
% H=subs(H,[m2 m3 l2 l3 lc2 lc3],[15.46 9.55 0.432 0.434 0.216 0.164])
% C=subs(C,[m2 m3 l2 l3 lc2 lc3],[15.46 9.55 0.432 0.434 0.216 0.164])
% G=subs(G,[m2 m3 l2 l3 lc2 lc3],[15.46 9.55 0.432 0.434 0.216 0.164])

%% Calculate derivative of Jacobian for Impedance Implementation
% syms q1(t) q2(t) q3(t)
% 
% dj11=-diff(q1(t), t)*cos(q1(t))*(l3*sin(q2(t) + q3(t)) + l2*cos(q2(t))) - sin(q1(t))*(l3*(diff(q2(t), t) + diff(q3(t), t))*cos(q2(t) + q3(t)) - l2*diff(q2(t), t)*sin(q2(t)));
% dj12=-diff(q1(t), t)*sin(q1(t))*(l3*cos(q2(t) + q3(t)) - l2*sin(q2(t))) + cos(q1(t))*(-l3*(diff(q2(t), t) + diff(q3(t), t))*sin(q2(t) + q3(t)) - l2*diff(q2(t), t)*cos(q2(t)));
% dj13 = -l3*(diff(q2(t), t) + diff(q3(t), t))*sin(q2(t) + q3(t))*cos(q1(t)) - l3*cos(q2(t) + q3(t))*diff(q1(t), t)*sin(q1(t));
% dj21 = -diff(q1(t), t)*sin(q1(t))*(l3*sin(q2(t) + q3(t)) + l2*cos(q2(t))) + cos(q1(t))*(l3*(diff(q2(t), t) + diff(q3(t), t))*cos(q2(t) + q3(t)) - l2*diff(q2(t), t)*sin(q2(t)));
% dj22 = diff(q1(t), t)*cos(q1(t))*(l3*cos(q2(t) + q3(t)) - l2*sin(q2(t))) + sin(q1(t))*(-l3*(diff(q2(t), t) + diff(q3(t), t))*sin(q2(t) + q3(t)) - l2*diff(q2(t), t)*cos(q2(t)));
% dj23 = -l3*(diff(q2(t), t) + diff(q3(t), t))*sin(q2(t) + q3(t))*sin(q1(t)) + l3*cos(q2(t) + q3(t))*diff(q1(t), t)*cos(q1(t));
% dj31 = 0;
% dj32 = -l3*(diff(q2(t), t) + diff(q3(t), t))*cos(q2(t) + q3(t)) + l2*diff(q2(t), t)*sin(q2(t));
% dj33 = -l3*(diff(q2(t), t) + diff(q3(t), t))*cos(q2(t) + q3(t));
% 
% dJac=[dj11,dj12,dj13;dj21,dj22,dj23;dj31,dj32,dj33];



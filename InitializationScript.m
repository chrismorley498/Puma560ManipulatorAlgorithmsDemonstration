%% Run This Part First
puma560ModifiedDynamics
creatingRigidBodyTree
load('VariantSettings.mat');

% Create the variant which will control which environment to make active
SpringWallRobustDemo=Simulink.Variant('environment==1');
MovingWallXDirection=Simulink.Variant('environment==2');
SinusoidalMovingWallXDirection=Simulink.Variant('environment==3');
EmptyEnvironment=Simulink.Variant('environment==4');
ImpedanceForceDemo=Simulink.Variant('environment==5');

% Create the variants for the puma itself
Puma560Robot=Simulink.Variant('robot==1');
Puma560Model=Simulink.Variant('robot==2');
Puma560Planner=Simulink.Variant('robot==3');
Puma560Simplified=Simulink.Variant('robot==4');

%========================================Different Senarios Below================================================================%

%% Inverse Dynamics (1)
IC=[0;0;0];
pathPoints=[0 3*pi/2 pi pi/2 0;0 -pi/2 -5*pi/4 0 0;0 pi pi pi 0];
timePoints=[0 2 4 6 8];
velPoints=zeros(3,5);
environment=4;
robot=2;
stopTime=8;
fEE=0;
fdControl=3;
%% Inverse Dynamics (2)
IC=[0;0;0];
pathPoints=[0 3*pi/2 pi pi/2 0;0 -pi/2 -5*pi/4 0 0;0 pi pi pi 0];
timePoints=[0 2 4 6 8];
velPoints=zeros(3,5);
environment=4;
robot=1;
stopTime=8;
fEE=0;
fdControl=3;
%% Sliding Adaptive Control
IC=[0;0;0];
pathPoints=[0 3*pi/2 pi pi/2 0;0 -pi/2 -5*pi/4 0 0;0 pi pi pi 0];
timePoints=[0 2 4 6 8];
velPoints=zeros(3,5);
environment=4;
robot=1;
stopTime=8;
fEE=0;
fdControl=3;
%% Sliding Robust Control - Force<200N
IC=[0;0;0];
pathPoints=[0 3*pi/2 pi pi/2 0;0 -pi/2 -5*pi/4 0 0;0 pi pi pi 0];
timePoints=[0 2 4 6 8];
velPoints=zeros(3,5);
environment=4;
robot=1;
fEE=100;
stopTime=8;
fdControl=3;
%% Sliding Robust Control - Force>200N
IC=[0;0;0];
pathPoints=[0 3*pi/2 pi pi/2 0;0 -pi/2 -5*pi/4 0 0;0 pi pi pi 0];
timePoints=[0 2 4 6 8];
velPoints=zeros(3,5);
environment=4;
robot=1;
fEE=300;
stopTime=8;
fdControl=3;
%% Impdeance Control - Moving Wall test
%In this senario the PUMA starts upright and has a wall moving towards it.
%The impedance reacts accordingly
% config=[0;-pi/3;pi/3+pi/2];
config=[0;-pi/2;pi];
IC=config;
pathPoints=[config,config,config,config,config,config,config];
environment=2;
timePoints=[0 2 4 6 8 10 12];
velPoints=zeros(3,7);
stopTime=12;
fEE=0;
fdControl=3;
%% Robust Control - Spring Wall
%In this senario the PUMA starts upright and pushes a spring wall
IC=[0;0;0];
solution1=ik('EE',[0 0 1 0.43;0 1 0 -.1295;-1 0 0 1.058;0 0 0 1],[0.9 0.9 0.9 1 0 0.1],IC)
solution2=ik('EE',[0 0 1 0.83;0 1 0 -.1295;-1 0 0 0.67;0 0 0 1],[0.9 0.9 0.9 1 0 0.7],solution1)
solution3=ik('EE',[0 0 1 0.43;0 1 0 -.1295;-1 0 0 1.058;0 0 0 1],[0.9 0.9 0.9 1 0 0.1],solution2)
solution4=ik('EE',[0 0 1 0.43;0 -1 0 -.1295;1 0 0 1.058;0 0 0 1],[0.9 0.9 0.9 1 0 0.7],solution3)
solution5=ik('EE',[0 0 1 0.83;0 -1 0 -.1295;1 0 0 0.67;0 0 0 1],[0.9 0.9 0.9 1 0 0.7],solution4)
solution6=ik('EE',[0 0 1 0.43;0 -1 0 -.1295;1 0 0 1.058;0 0 0 1],[0.9 0.9 0.9 1 0 0.7],solution3)
pathPoints=[IC,solution1,solution2,solution3,solution4,solution5,solution6];
timePoints=[0 2 4 6 8 10 12];
velPoints=zeros(3,7);
environment=1;
robot=1;
stopTime=12;
fdControl=3;%MAY NEED TO CHANGE AT LATER DATE
%% Impedance Force Control - Spring Wall
%In this senario the PUMA starts upright and pushes a spring wall
IC=ik('EE',[0 0 1 0.43;0 1 0 -.1295;-1 0 0 1.058;0 0 0 1],[0.9 0.9 0.9 1 0 0.1],[0;0;0])
solution1=ik('EE',[0 0 1 0.63;0 1 0 -.1295;-1 0 0 1.058;0 0 0 1],[0.9 0.9 0.9 1 0 0.1],IC)
solution2=ik('EE',[0 0 1 0.63;0 1 0 -.1295;-1 0 0 1.058;0 0 0 1],[0.9 0.9 0.9 1 0 1],solution1)
solution3=ik('EE',[0 0 1 0.63;0 1 0 -.1295;-1 0 0 1.058;0 0 0 1],[0.9 0.9 0.9 1 0 1],solution2)
solution4=ik('EE',[0 0 1 0.63;0 1 0 -.1295;-1 0 0 1.058;0 0 0 1],[0.9 0.9 0.9 1 0 1],solution3)
solution5=ik('EE',[0 0 1 0.63;0 1 0 -.1295;-1 0 0 1.058;0 0 0 1],[0.9 0.9 0.9 1 0 1],solution4)
solution6=ik('EE',[0 0 1 0.63;0 1 0 -.1295;-1 0 0 1.058;0 0 0 1],[0.9 0.9 0.9 1 0 1],solution3)
pathPoints=[IC,solution1,solution1,solution1,solution1,solution1,solution1];
environment=5;
timePoints=[0 2 4 6 8 10 12];
velPoints=zeros(3,7);
stopTime=12;
robot=1;
fdControl=1;
%% Hybrid Motion Force - Spring Wall
%In this senario the PUMA starts upright and pushes a spring wall
IC=ik('EE',[0 0 1 0.43;0 1 0 -.1295;-1 0 0 1.058;0 0 0 1],[0.9 0.9 0.9 1 0 0.1],[0;0;0])
solution1=ik('EE',[0 0 1 0.63;0 1 0 -.1295;-1 0 0 0.67;0 0 0 1],[0.9 0.9 0.9 1 0 0.1],IC)
solution2=ik('EE',[0 0 1 0.63;0 1 0 -.1295;-1 0 0 0.67;0 0 0 1],[0.9 0.9 0.9 1 0 1],solution1)
solution3=ik('EE',[0 0 1 0.63;0 1 0 -.1295;-1 0 0 0.67;0 0 0 1],[0.9 0.9 0.9 1 0 1],solution2)
solution4=ik('EE',[0 0 1 0.63;0 1 0 -.1295;-1 0 0 0.67;0 0 0 1],[0.9 0.9 0.9 1 0 1],solution3)
solution5=ik('EE',[0 0 1 0.63;0 1 0 -.1295;-1 0 0 0.67;0 0 0 1],[0.9 0.9 0.9 1 0 1],solution4)
solution6=ik('EE',[0 0 1 0.63;0 1 0 -.1295;-1 0 0 0.67;0 0 0 1],[0.9 0.9 0.9 1 0 1],solution3)
pathPoints=[IC,solution1,solution1,solution1,solution1,solution1,solution1];
environment=5;
robot=1;
fdControl=2;
%% Empty Environment
IC=[0;0;0];
pathPoints=[IC,solution1,solution2,solution3,solution4,solution5,solution6];
environment=4;

 

%% Trial and error stuff
%Y-Displacement of Link2 for geometry to look correct -0.14224
%Z-Displacement of link3 for geometry to look correct -0.0254

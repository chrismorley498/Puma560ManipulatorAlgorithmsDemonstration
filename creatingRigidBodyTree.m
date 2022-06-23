%Creating ridgid body tree of the PUMA-560

%Link 1, Joint 1
link1=rigidBody('link1');
joint1=rigidBodyJoint('joint1','revolute');
setFixedTransform(joint1,[1 0 0 0;0 1 0 0;0 0 1 0.54864;0 0 0 1]);
link1.Joint=joint1;

%Link2, Joint 2
link2=rigidBody('link2');
joint2=rigidBodyJoint('joint2','revolute');
setFixedTransform(joint2,[1 0 0 0;0 0 1 -0.14224;0 -1 0 0.07493;0 0 0 1])
link2.Joint=joint2;

%Link3, Joint 3
link3=rigidBody('link3');
joint3=rigidBodyJoint('joint3','revolute');
setFixedTransform(joint3,[1 0 0 0.430931;0 1 0 0;0 0 1 -0.0254;0 0 0 1])
joint3.PositionLimits=[0 3.14]%Added this to fix a weird IK solution I was getting
link3.Joint=joint3;

%EE
EE=rigidBody('EE');
jointEE=rigidBodyJoint('jointEE','fixed');
setFixedTransform(jointEE,[1 0 0 0;0 0 -1 -0.4321;0 1 0 0.0381;0 0 0 1])% [1 0 0 0;0 0 -1 -0.35179;0 1 0 0.0381;0 0 0 1]
EE.Joint=jointEE;

%Define all the mass properties of links
link1.Mass=0;
link1.Inertia=[0 0 0 0 0 0];

link2.Mass=15.46;
link2.Inertia=[0 0 0 0 0 0];
link2.CenterOfMass=[0.216 0 0];

link3.Mass=9.55;
link3.Inertia=[0 0 0 0 0 0];
link3.CenterOfMass=[0 -0.164 0];


%Create the rigidBodyObject
PUMA560=rigidBodyTree;
addBody(PUMA560,link1,'base');
addBody(PUMA560,link2,'link1');
addBody(PUMA560,link3,'link2');
addBody(PUMA560,EE,'link3');

PUMA560.DataFormat='column'
PUMA560.Gravity=[0 0 -9.81];
ik=inverseKinematics('RigidBodyTree',PUMA560)


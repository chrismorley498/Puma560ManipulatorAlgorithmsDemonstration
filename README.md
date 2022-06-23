# Puma560ManipulatorAlgorithmsDemonstration
This repo contains a Simulink model and supporting MATLAB scripts to analyse how different non-linear control methods can be used to control the PUMA 560.
There are both force and position control algorithms. Force control algorithms, when run, demonstrate the manipulator's ability to interact with the environment in a stable manor. 

A description of some of the most notable algorithms can be found below.

## Computed Torque/Inverse Dynamics
This algorithm assumes perfect knowledge of the manipulator, including the inertia and geometry of each link. In practice this is not the case. This algorithm's fault can be seen as it fails to track the desired trajectory because certain parameters in the equations are intentionally wrong.

## Sliding Adaptive Control
This algorithm is made up of two parts. Namely the adaptive law and the control law. The adaptive law is responsible for iteratively updating the estimate of unknown model parameters. This estimate is performed on a need to know basis, meaning that the estimated parameters _may_ not be accurate but they are sufficient to guarantee accurate tracking of the desired trajectory. The adaptive law using a regressor matrix where the rows are linear combinations of the unknown model parameters. The regressor matrix is formed assuming that each link of the robot has a unkown point mass located some unkown distance along the z-axis of the frame located at the proxal end of the link. The manipulator links in reality have a mass distribution however because the algorithm working on a need to know basis it can still accurately tracking the desired trajectory. 

The control law uses the estimated model parameters to implement sliding mode control. The sliding surface is a hyperplane defined by the sum of joint positions and velocity errors. The derivative of the sliding surface contains the control input, joint torque, so the state of the system successfully converges to the sliding surface during the reaching phase of the control. Once the state of the system is on the hyperplane, the sliding mode controller will ensure the state does not leave the hyperplane. While on the hyperplane, the state will follow the dynamics described by the sliding surface equation when the equation is set equal to zero.

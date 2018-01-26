%Determing the Jacobian of the transformation matrix

%First build the transformation matrix
syms theta1 theta2 theta3 theta4;
%rotate z theta1
rotateZt1=[cos(theta1) -sin(theta1) 0 0;sin(theta1) cos(theta1) 0 0; 0 0 1 0;0 0 0 1];
%translate x 50
translateX50=[1 0 0 50; 0 1 0 0; 0 0 1 0; 0 0 0 1];
%rotate z theta2
rotateZt2=[cos(theta2) -sin(theta2) 0 0;sin(theta2) cos(theta2) 0 0; 0 0 1 0;0 0 0 1];
%translate x 30
translateX30=[1 0 0 30; 0 1 0 0; 0 0 1 0; 0 0 0 1];
%rotate z theta3
rotateZt3=[cos(theta3) -sin(theta3) 0 0;sin(theta3) cos(theta3) 0 0; 0 0 1 0;0 0 0 1];
%translate z 15
translateZ15=[1 0 0 0; 0 1 0 0; 0 0 1 15; 0 0 0 1];
%rotate x 90
rotateX90=[1 0 0 0;0 cos(pi/2) -sin(pi/2) 0;0 sin(pi/2) cos(pi/2) 0;0 0 0 1];
%rotate z theta4
rotateZt4=[cos(theta4) -sin(theta4) 0 0;sin(theta4) cos(theta4) 0 0; 0 0 1 0;0 0 0 1];

%Overall transformation matrix
TMatrix=rotateZt1*translateX50*rotateZt2*translateX30*rotateZt3*translateZ15*rotateX90*rotateZt4;

%end effector origin in operational space as a function of thetas
OpSpaceTheta=TMatrix*[0;0;0;1];
J=jacobian(OpSpaceTheta,[theta1,theta2,theta3,theta4]);
%find the psuedo inverse
J_inv=(J.'*J)*J.';
display(J_inv);
% % % % % % 0% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  
%  This script was created using Robotic Toolbox v10.3 developed by Peter Corke.  % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

clear;
clc;
clf;

% DH Parameters 
L1 =Link('alpha',0,     'a',0,    'd',0,    'qlim',[-2.96706 2.96706],  'modified'); %rotational
L2 =Link('alpha',-pi/2, 'a',150,  'd',0,    'qlim',[-2.18170 2.18170],  'modified', 'offset', -pi/2); %rotational
L3 =Link('alpha',0,     'a',360,  'd',0,    'qlim',[-2.70530 2.70530],  'modified'); %rotational
L4 =Link('alpha',-pi/2, 'a',100,  'd',430,  'qlim',[-3.31613 3.31613],  'modified'); %rotational
L5 =Link('alpha',pi/2,  'a',0,    'd',0,    'qlim',[-2.44350 2.44350],  'modified'); %rotational
L6 =Link('alpha',-pi/2, 'a',0,    'd',100,  'qlim',[-6.28319 6.28319],  'modified'); %rotational

% Setup an object "Fanuc" that is of class SerialLink. This class describes
% industrial manipulator that is based on the variables L1..L6.
Fanuc=SerialLink([L1 L2 L3 L4 L5 L6],'name', 'FANUC');

% "A" is a method of class SerialLink that creates link transformation matrices.
% This part is commented out as we do not actually use this matrices for
% anything. However, if uncommented, the particula transformation from
% frame i to frame i+1 can be examined.

T01 = L1.A(0);
T12 = L2.A(0);
T23 = L3.A(0);
T34 = L4.A(0);
T45 = L5.A(0);
T56 = L6.A(0);
T06 = T01*T12*T23*T34*T45*T56;

% Set up tool transformation matrix (T56 to TOOL) for the "Fanuc" object 
 
Fanuc.tool = [0.9728 0 0.2314 42;
              0 1 0 0;
              -0.2314 0 0.9728 170;
              0 0 0 1];

% Fanuc.tool = [1 0 0 0; 
%               0 1 0 0; 
%               0 0 1 0;
%               0 0 0 1];

% Set up base transformation matrix (BASE to T01) for the "Fanuc" object 
Fanuc.base = [1 0 0 0; 
              0 1 0 0; 
              0 0 1 450;
              0 0 0 1];

% FORWARD KINEMATICS 
% "q" is the matrix where thetas are defined - number of collums is equal to the number of thetas
Qfkine = [-86   117   -24  -70   71   253];

% For the object "Fanuc" do the class method "fkine" with parameters inside of the
% matrix "Qfkine" with additional info that values inside of the matrix are
% degrees 
Tfkine=Fanuc.fkine(Qfkine,'deg');

% display matrix Tfkine
display(Tfkine);

% Display that fancy graph where you can rotate the robot and stuff. Matrix
% "q0" is the argument that defines initial position in the graph. 
q0 = [0 0 0 0 0 0];
Fanuc.teach(deg2rad(Qfkine));
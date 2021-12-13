clear; clc;
%Differential kinematics torque pallelast
%Dette skriptet definerer robotarmen i vårt forklift with manipulator
%prosjekt ved hjelp av DH-parameter. En kraft tilsvarende 500kg som 
%presser ned på pallegaffel blir deretter simulert, og kraften som påvirker
%hvert ledd i roboten blir deretter beregnet

%Definerer robot vha. DH parameter
qz =([0 0 -pi/2 pi/2 0]); %Parkert forran base
L(1) = Link ( [0 0.5 0 pi/2] );
L(2) = Link ( [0 0 0.7 0] );
L(3) = Link ( [0 0 0.8 0] );
L(4) = Link ( [0 0 0.4 0] );
L(5) = Link ( [0 0 0 0] );
Robot = SerialLink (L);
%Forkorter pallegaffel for å simulere at end effector
%er midt på pallegaffel, slik at kraften fra palle blir plassert
%midt i pallegaffel
Robot.tool = [0,0,1,0.625;0,-1,0,-0.25;1,0,0,0;0,0,0,1];

%Pose for palle løftet fra reol. Forkortet pose
%slik at den skal stemme overens med forkortet pallegaffel
iPalleLifted = SE3(0,1.875,1.65) * SE3.Rx(-pi/2);
%Ledverdier når palle er løftet fra reol
Robot.ikine(iPalleLifted, 'mask', [1 1 1 1 0 1]);

%Definerer last i end effector. 500kg last ned i pallegaffel
W0 = [0;0;50;0;0;0];
%Definerer jacob0matrise
Q = Robot.jacob0(qz)'*W0;
fprintf('Kraft i ledd 1, 2, 3, 4, 5:')
Kraft = Q'

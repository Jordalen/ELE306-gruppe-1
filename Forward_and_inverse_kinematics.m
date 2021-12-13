clear;clc;

%Forklift_robot_baneplanleggin
%Dette skriptet definerer robotarmen i vårt forklift with manipulator
%prosjekt ved hjelp av DH-parameter. Forward og inverse kinematics
%blir deretter testet ved å samenlige resultat fra fkine funksjon med
%matrisene som ble beregnet fra DH parameterene.
%Deretter brukes inverse kinematics for å se at vi får 0-vinkler som svar

%Definerer matrise med 0-vinkler
q1 = 0; q2 = 0; q3 = 0; q4 = 0; q5 = 0;
qi = [q1,q2,q3,q4,q5];

% Definerier robot ved hjelp av DH-parameter
L(1) = Link ( [0 0.5 0 pi/2] );
L(2) = Link ( [0 0 0.7 0] );
L(3) = Link ( [0 0 0.8 0] );
L(4) = Link ( [0 0 0.4 0] );
L(5) = Link ( [0 0 0 0] );
Robot = SerialLink (L);
Robot.tool = [0,0,1,1.25;0,-1,0,-0.25;1,0,0,0;0,0,0,1];
Robot.name = 'Forklift Robot';

%Matriser regnet ut basert på DH parameter
T01 = [cos(q1),0,sin(q1),0;sin(q1),0,-cos(q1),0;0,1,0,0.5;0,0,0,1];
T12 = [cos(q2),-sin(q2),0,0.7*cos(q2);sin(q2),cos(q2),0,0.7*sin(q2);0,0,1,0;0,0,0,1];
T23 = [cos(q3),-sin(q3),0,0.8*cos(q3);sin(q3),cos(q3),0,0.8*sin(q3);0,0,1,0;0,0,0,1];
T34 = [cos(q4),-sin(q4),0,0.4*cos(q4);sin(q4),cos(q4),0,0.4*sin(q4);0,0,1,0;0,0,0,1];
T45 = [cos(q5),-sin(q5),0,0;sin(q5),cos(q5),0,0;0,0,1,0;0,0,0,1];
TOOL = [0,0,1,1.25;0,-1,0,-0.25;1,0,0,0;0,0,0,1];

%Regner ut matrisene
BaseTTooltipSerialLink = Robot.fkine(qi)
BaseTTooltipMatrix = T01*T12*T23*T34*T45*TOOL
VinklerInverseKinematics = Robot.ikine(BaseTTooltipSerialLink, 'mask', [1 1 1 1 0 1])
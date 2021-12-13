
%Forklift_robot_baneplanleggin
%Dette skriptet definerer robotarmen i vårt forklift with manipulator
%prosjekt ved hjelp av DH-parameter. En bevegelse som simulerer at
%robotarmen plukker ned en palle som er plassert i en reol på siden blir
%deretter implementert ved hjelp av mtraj og ctraj funksjon

clear; clc;

% Definerier robot ved hjelp av DH-parameter
L(1) = Link ( [0 0.5 0 pi/2] );
L(2) = Link ( [0 0 0.7 0] );
L(3) = Link ( [0 0 0.8 0] );
L(4) = Link ( [0 0 0.4 0] );
L(5) = Link ( [0 0 0 0] );
Robot = SerialLink (L);
Robot.tool = [0,0,1,1.25;0,-1,0,-0.25;1,0,0,0;0,0,0,1];
Robot.name = 'Forklift Robot';

%Poses, definering av punkt som roboten skal bevege seg gjennom
qz =([0 0 -pi/2 pi/2 0]); %Parkert forran base
startPos = Robot.fkine(qz);
readyForTurn = SE3(1.5, 0, 1.5) * SE3.Ry(pi/2) * SE3.Rz(-pi/2);
readyForTurnFront = SE3(2,0,1.5) * SE3.Ry(pi/2) * SE3.Rz(-pi/2);
forranPalle = SE3(0,1.5,1.5) * SE3.Rx(-pi/2);
iPalle = SE3(0,2.5,1.5) * SE3.Rx(-pi/2);
iPalleLifted = SE3(0,2.5,1.65) * SE3.Rx(-pi/2);
forranPalleLifted = SE3(0,1.5,1.65) * SE3.Rx(-pi/2);

% Inverse kinematic
qReadyForTurn = Robot.ikine(readyForTurn, 'mask', [1 1 1 1 0 1]);
qforranPalle = Robot.ikine(forranPalle, 'mask', [1 1 1 1 0 1]);
qiPalle = Robot.ikine(iPalle, 'mask', [1 1 1 1 0 1]);
qiPalleLifted = Robot.ikine(iPalleLifted, 'mask', [1 1 1 1 0 1]);
qforranPalleLifted = Robot.ikine(forranPalleLifted, 'mask', [1 1 1 1 0 1]);

%Tidsvektor
t = [0:0.05:2];

%Trajectory planing cartesian space
Ts1 = ctraj(readyForTurnFront, readyForTurn, length(t)/2);
Ts3 = ctraj(forranPalle, iPalle, length(t));
Ts5 = ctraj(iPalleLifted, forranPalleLifted, length(t));
Ts7 = ctraj(readyForTurn, readyForTurnFront, length(t)/2);

%Inverse kinematic on the planned trajectory
qi1 = Robot.ikine(Ts1, 'mask', [1 1 1 1 0 1]);
qi3 = Robot.ikine(Ts3, 'mask', [1 1 1 1 0 1]);
qi5 = Robot.ikine(Ts5, 'mask', [1 1 1 1 0 1]);
qi7 = Robot.ikine(Ts7, 'mask', [1 1 1 1 0 1]);

%Trajectory planing joint space
qi0 = mtraj(@tpoly, qz, qi1(1,:), t);
qi2 = mtraj(@tpoly, qi1(end,:), qi3(1,:), t);
qi4 = mtraj(@tpoly, qi3(end,:), qi5(1,:), t);
qi6 = mtraj(@tpoly, qi5(end,:), qi7(1,:), t);
qi8 = mtraj(@tpoly, qi7(end,:), qz, t);

%Ploter grafisk framstilling av banen
Robot.plot(qi0); Robot.plot(qi1); Robot.plot(qi2); Robot.plot(qi3); Robot.plot(qi4); 
Robot.plot(qi5); Robot.plot(qi6); Robot.plot(qi7); Robot.plot(qi8);
%Forklift_robot_baneplanleggin
%Dette skriptet definerer robotarmen i vårt forklift with manipulator
%prosjekt ved hjelp av DH-parameter. Deretter simuleres at et objekt
%oppdages av kamera. Deretter skal vinkler som robotarm må ha for
%å nå dette objektet med tool beregnes

L(1) = Link ( [0 0.5 0 pi/2] );
L(2) = Link ( [0 0 0.7 0] );
L(3) = Link ( [0 0 0.8 0] );
L(4) = Link ( [0 0 0.4 0] );
L(5) = Link ( [0 0 0 0] );
Robot = SerialLink (L);
Robot.tool = [0,0,1,1.25;0,-1,0,-0.25;1,0,0,0;0,0,0,1];
Robot.name = 'Forklift Robot';

%Transformasjons matriser
r5Ttool = [0,0,1,1.25;0,-1,0,-0.25;1,0,0,1;0,0,0,1];
r5Tc2 = [1,0,0,0.1;0,1,0,0;0,0,1,0;0,0,0,1];
c2Tobjekt = [1,0,0,1;0,1,0,0;0,0,1,1;0,0,0,1];

toolTobject = inv(r5Ttool) * r5Tc2 * c2Tobjekt

%Beregner vinkler som robotarm må ha for å nå dette objektet
qRobotarmTilObjekt = Robot.ikine(toolTobject, 'mask', [1 1 1 1 0 1])

%%
%Robotarm control

clear all;
clc;

qParkFront = ([0 0 -pi/2 pi/2 0]);
qParkBase = degtorad([180 133.2 -154.8 -61.2 82.8]); %Parkert pa lasteplan
qSetteAvPalle = degtorad([0 -19 -11 3.4 26.5]); %Pallegaffel forran base for aa lesse av palle
%qTrekkeUtAvPalle = degtorad([0 35 -126.6 3.4 87.7]); %Parkere palle
qTrekkeUtAvPalle = degtorad([0 42.2 -137 3.4 91.3]); %Parkere palle

L(1) = Link ( [0 0.5 0 pi/2] );
L(2) = Link ( [0 0 0.7 0] );
L(3) = Link ( [0 0 0.8 0] );
L(4) = Link ( [0 0 0.4 0] );
L(5) = Link ( [0 0 0 0] );
Robot = SerialLink (L);
Robot.tool = [0,0,1,1.25;0,-1,0,-0.25;1,0,0,0;0,0,0,1];
%Robot.plot(qParkBase)
%hold on


%Poses
ParkFront = Robot.fkine(qParkFront);
palleParkFront = SE3(3.5, 0, 2) * SE3.Ry(pi/2) * SE3.Rz(-pi/2);
readyForTurnFront = SE3(1.85, 0, 1.4) * SE3.Ry(pi/2) * SE3.Rz(-pi/2);
readyForTurnBack = SE3(-1.85,0,1.4) * SE3.Ry(-pi/2) * SE3.Rz(pi/2);
forranPalle = SE3(0,1.85,1.4) * SE3.Rx(-pi/2);
iPalle = SE3(0,2.65,1.4) * SE3.Rx(-pi/2);
iPalleLifted = SE3(0,2.65,1.5) * SE3.Rx(-pi/2);
forranPalleLifted = SE3(0,1.85,1.5) * SE3.Rx(-pi/2);

%trplot(ParkFront)
%trplot(readyForTurnFront)
%trplot(readyForTurnBack)
%trplot(forranPalle)
%trplot(iPalle)

%qReadyForTurnBack = Robot.ikine(readyForTurnBack,[180 0.8921 0.9340 -2.2621 0.4361], 'mask', [1 1 1 1 0 0 ])


%Tidsvektor
t = [0:0.05:2];

%Trajectory planing cartesian space
Ts1 = ctraj(ParkFront, readyForTurnFront, 20);

%Ts2 = ctraj(readyForTurnFront, readyForTurnBack, 20);

%Ts3 = ctraj(forranPalle, iPalle, 20);

%Trajectory planning joint space

%Inverse kinematic on the planned trajectory
%qi1 = Robot.ikine(Ts1, 'mask', [1 1 1 1 0 0 ])
%qi2 = Robot.ikine(Ts2, 'mask', [1 1 1 1 0 0 ]);
%qi3 = Robot.ikine(Ts3, 'mask', [1 1 1 1 0 0 ]);
%qi2 = mtraj (@tpoly,qi1(length(qi1),:),qi3(1,:), t) 

%Robot.plot(qi1);
%Robot.plot(qi2);
%Robot.plot(qi3);
%Robot.plot(qi4);
%Robot.plot(qi5);

%q = [qi1;qi2;qi3]

qReadyForTurnFront = ([0 1.0508 1.1911 -2.2018 -0.0401]);
qForranPalle = ([1.5708 1.0508 1.1911 -2.2018 -0.0401]);
qReadyForTurnBack = ([pi 1.0508 1.1911 -2.2018 -0.0401]);



%qi2 = mtraj (@tpoly,qi1(length(qi1),:),qi3(1,:), t)

%Arm: fra parkering forran til parkering pa base
qi1 = mtraj (@tpoly, qParkFront, qReadyForTurnFront, t);
qi2 = mtraj (@tpoly, qReadyForTurnFront,qForranPalle, t);
qi3 = mtraj (@tpoly,qForranPalle,qReadyForTurnBack, t);
qi4 = mtraj (@tpoly,qReadyForTurnBack, qParkBase, t);

%Arm: Fra parkeringg pa base, plukke ned palle, parkering pa base
qi5 = mtraj (@tpoly, qParkBase, qReadyForTurnBack, t);
qi6 = mtraj (@tpoly, qReadyForTurnBack, qForranPalle, t);
Ts7 = ctraj (Robot.fkine(qForranPalle), iPalle, 20);
qi7 = Robot.ikine(Ts7, 'mask', [1 1 1 1 0 1]);
Ts8 = ctraj (iPalle, iPalleLifted, 20);
qi8 = Robot.ikine(Ts8, 'mask', [1 1 1 1 0 1]);
Ts9 = ctraj (iPalleLifted, forranPalleLifted, 20);
qi9 = Robot.ikine(Ts9, 'mask', [1 1 1 1 0 1]);
qi10 = mtraj (@tpoly, qi9(length(qi9),:), qReadyForTurnBack, t);
qi11 = mtraj (@tpoly,qReadyForTurnBack, qParkBase, t);

%Arm: Lofter av palle og parkerer pa avleveringspunkt
qi12 = mtraj (@tpoly, qParkBase, qReadyForTurnBack, t);
qi13 = mtraj (@tpoly, qReadyForTurnBack, qReadyForTurnFront, t);
qi14 = mtraj (@tpoly, qReadyForTurnFront, qSetteAvPalle, t);


qi15 = mtraj (@tpoly, qSetteAvPalle, qTrekkeUtAvPalle, t);


q = [qi1;qi2;qi3;qi4;qi5;qi6;qi7;qi8;qi9;qi10;qi11;qi12;qi13;qi14;qi15];


%Robot.plot(q)
%%
rosinit
global odom

sub_odom = rossubscriber("/odom",@odom_callback);

[pub_q1,msg_q1] = rospublisher('/Forklift_robot/arm_base_position/command','std_msgs/Float64');
[pub_q2,msg_q2] = rospublisher('/Forklift_robot/arm_link1_position/command','std_msgs/Float64');
[pub_q3,msg_q3] = rospublisher('/Forklift_robot/arm_link2_position/command','std_msgs/Float64');
[pub_q4,msg_q4] = rospublisher('/Forklift_robot/arm_link3_position/command','std_msgs/Float64');
[pub_q5,msg_q5] = rospublisher('/Forklift_robot/pallegaffel_position/command','std_msgs/Float64');
[pub_vel,msg_vel] = rospublisher('/cmd_vel','geometry_msgs/Twist');

x=0;

rate = robotics.Rate(10);
while rate.TotalElapsedTime < 100
    %odom.Pose.Pose.Position

    x=x+1;
    

    msg_q1.Data = 0;
    msg_q2.Data = 0;
    msg_q3.Data = -pi/2;
    msg_q4.Data = pi/2;
    msg_q5.Data = q(5);
    
    msg_q1.Data = q(x,1);
    msg_q2.Data = q(x,2);
    msg_q3.Data = q(x,3);
    msg_q4.Data = q(x,4);
    msg_q5.Data = q(x,5);
    
    send(pub_q1,msg_q1)
    send(pub_q2,msg_q2)
    send(pub_q3,msg_q3)
    send(pub_q4,msg_q4)
    send(pub_q5,msg_q5)
    
    send(pub_vel,msg_vel)

    waitfor(rate);
end

msg_vel.Linear.X = 0.0;
msg_vel.Angular.Z = 0.0;
send(pub_vel,msg_vel)

rosshutdown

function odom_callback(src,msg)
    global odom
    odom = msg; 
    
end

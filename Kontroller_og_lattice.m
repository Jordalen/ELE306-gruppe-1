 %%Lager kart
map = binaryOccupancyMap(100, 100, 1);
lager = zeros(100,100);

%beregner kart
lager(30:100,[1:15 42:57 84:100]) = 1;
lager(1:15,84:100) = 1;





lp = Lattice(lager, 'grid', 5, 'root', [2 2 0], 'inflate', 1);
lp.plan('iterations', 200, 'cost', [1 100 100]);
lp.plot()

%% Lattice goal
p = lp.query( [12 12 0], [27 57 0]);
lp.plot(p)
o = lp.query([27 57 0], [82 7 0]);
lp.plot(o)

%% Lager path for å hente pall

path = [1.2   1.2;
        2.2   1.2;
        2.7   1.7;
        2.7   5.7];



robotInitialLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = 0;
robotCurrentPose = [robotInitialLocation initialOrientation]';
robot = differentialDriveKinematics("TrackWidth", 1.0, "VehicleInputs", "VehicleSpeedHeadingRate");
figure
plot(path(:,1), path(:,2));
xlim([0 10])
ylim([0 10])

% define path following robot
controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.3;
controller.MaxAngularVelocity = 45;
controller.LookaheadDistance =0.1;

% using the path following controller
goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);

sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

figure

frameSize = robot.TrackWidth/3;

while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose); % bruke x og z
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
    
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(path(:,1), path(:,2),"k--d")
    hold all
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 10])
    ylim([0 10])
    
    waitfor(vizRate);
end

%% lager fra henting av pall og kjører til hentepunkt
path = [2.7   5.7
        2.7   1.2;
        3.2   0.7;
        8.2   0.7];
    
robotInitialLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = 0;
robotCurrentPose = [robotInitialLocation initialOrientation]';
robot = differentialDriveKinematics("TrackWidth", 1.0, "VehicleInputs", "VehicleSpeedHeadingRate");
figure
plot(path(:,1), path(:,2));
xlim([0 10])
ylim([0 10])

% define path following robot
controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.3;
controller.MaxAngularVelocity = 45;
controller.LookaheadDistance =0.1;

% using the path following controller
goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);

sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

figure

frameSize = robot.TrackWidth/3;

while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose); % bruke x og z
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
    
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(path(:,1), path(:,2),"k--d")
    hold all
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 10])
    ylim([0 10])
    
    waitfor(vizRate);
end
    

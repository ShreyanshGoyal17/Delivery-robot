clc;clear all;

image = imread('map3.png');
grayimage = rgb2gray(image);
bwimage = grayimage < 100;
map= binaryOccupancyMap(bwimage);

%show(grid)

costmap = vehicleCostmap(map);
plot(costmap)
validator = validatorOccupancyMap;
validator.Map = map;
space = stateSpaceSE2;

planner = plannerHybridAStar(validator,'MinTurningRadius',4,'MotionPrimitiveLength',6);

startPose = [5 45 0]; % [meters, meters, radians]
goalPose = [34 4 0];

refpath = plan(planner,startPose,goalPose);
show(planner)

controller = controllerPurePursuit;
controller.Waypoints = refpath.States(:,1:2);
controller.DesiredLinearVelocity = 2;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 1;


robotInitialLocation = refpath.States(1,1:2);
robotGoal = refpath.States(end,1:2);
initialOrientation = 0;
robotCurrentPose = [robotInitialLocation initialOrientation]';
distanceToGoal = norm(robotInitialLocation - robotGoal);
goalRadius = 1.5;

robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");

sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);
% Initialize the figure
figure
frameSize = robot.TrackWidth/0.4;

while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
    show(map);
    hold all

    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(refpath.States(:,1), refpath.States(:,2),"k--d")
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 90])
    ylim([0 60])
    
    waitfor(vizRate);
end

clear variables; close all; clc;

rosshutdown; % ends ROS if it is already running


ipaddress= "localhost";
rosinit(ipaddress)


init_pos = rospublisher('/gazebo/set_model_state','gazebo_msgs/ModelState');
init_msg = rosmessage(init_pos);
init_msg.ModelName = "turtlebot3_burger";
% init_msg.Pose.Position.X = 6;
% init_msg.Pose.Position.Y = 7;
% init_msg.Pose.Position.Z = 0;
% init_msg.Pose.Orientation.X = 0;
% init_msg.Pose.Orientation.Y = 0;
% init_msg.Pose.Orientation.Z = 1;
% init_msg.Pose.Orientation.W = 0;

init_msg.Pose.Position.X = 0;
init_msg.Pose.Position.Y = -6;
init_msg.Pose.Position.Z = 0;
init_msg.Pose.Orientation.X = 0;
init_msg.Pose.Orientation.Y = 0;
init_msg.Pose.Orientation.Z = 0;
init_msg.Pose.Orientation.W = 1;
send(init_pos,init_msg)

robot_x = [];
robot_y = [];

SubOdom = rossubscriber('/odom');

SubModelState = rossubscriber('/gazebo/model_states');
restrict_range=5;
xGazebo = [];
yGazebo = [];
k=0.5;
min_acc=0.3;
% Define robot control parameters
forwardVelocity = 0.2;    % Linear velocity (m/s)
distanceThreshold = 1;  % Distance threshold (m)

% Create ROS publishers and messages to control robots
robot = rospublisher('/cmd_vel');
velmsg = rosmessage(robot);

% Create ROS subscribers to receive LiDAR data
laser = rossubscriber('/scan');

tic;
pause(1);
% Runing for 120 seconds
while toc < 120

    % Record robot position
    Odom = receive(SubOdom);
    [OdomPos(1), OdomPos(2)] = deal(Odom.Pose.Pose.Position.X, ...
        Odom.Pose.Pose.Position.Y);
    robot_x = [robot_x OdomPos(1)];
    robot_y = [robot_y OdomPos(2)];
    % Obtaining data from LiDAR and filtering
    scan = receive(laser,1);
    range = scan.Ranges;
    % iScan = find(range>1.75);
    % range(iScan) = inf; 
    range(91:269) = inf; 
    % Plot unfiltered scan points
    title('Test Only')
    plot(scan);
    hold on;
    plot(0,0,"+");
    % Check if there is obstance in 1m distance of the robot
    data = readCartesian(scan);
    x1 = data(:, 1);
    y1 = data(:, 2);
    sum_of_squares = sqrt(x1.^2 + y1.^2);
    result = sum_of_squares < 1;
    % If obstance near the robot, slow down the robot
    if(max(result)==1)
        forwardVelocity = 0.1; 
        disp("Danger");
    else
        forwardVelocity = 0.2; 
    end
    
    % Extracting boundary points of obstacles
    obstacleLeftBoundary = findLeftBoundary(range, distanceThreshold);
    obstacleRightBoundary = findRightBoundary(range, distanceThreshold);
    % If right side have obstacles that may stop the robot
    if(range(360)<3.5|| range(359)<3.5|| range(358)<3.5|| range(357)<3.5||range(356)<3.5||range(355)<3.5 )
        velmsg.Linear.X = forwardVelocity;
        velmsg.Angular.Z = 0;

        if ~isnan(obstacleLeftBoundary)

            % Calculate the angle between the robot and the next boundary point
    
            desiredAngle= (obstacleLeftBoundary)/180*pi ;
            x=sin(obstacleLeftBoundary/180*pi)*range(obstacleLeftBoundary);
            y=cos(obstacleLeftBoundary/180*pi)*range(obstacleLeftBoundary);
            plot(y,x, "ro")
            disp("L");
            % Set the angular velocity to k times the target angle of the robot, 
            % keeping the linear velocity unchanged

            velmsg.Linear.X = forwardVelocity*0.8;
            disp(obstacleLeftBoundary);
            if(k *desiredAngle>min_acc)
                velmsg.Angular.Z = k *desiredAngle;
            else
                velmsg.Angular.Z = min_acc;
            end
        else
            velmsg.Angular.Z = min_acc;
            disp("No Boundary");
        end
    
    elseif  ( range(1)<3.5 || range(2)<3.5|| range(3)<3.5|| range(4)<3.5||range(5)<3.5||range(6)<3.5)
        if ~isnan(obstacleRightBoundary)

            % Calculate the angle between the robot and the next boundary point
            desiredAngle = (obstacleRightBoundary-360)/180*pi ;
            x=sin(obstacleRightBoundary/180*pi)*range(obstacleRightBoundary);
            y=cos(obstacleRightBoundary/180*pi)*range(obstacleRightBoundary);
            plot(y,x, "ro")
            disp("R")
            disp(k *desiredAngle);
        
        
            % Set the angular velocity to k times the target angle of the robot, 
            % keeping the linear velocity unchanged
            velmsg.Linear.X = forwardVelocity*0.8;
            if(k *desiredAngle<-min_acc)
                velmsg.Angular.Z = k *desiredAngle;
            else
                velmsg.Angular.Z = -min_acc;
            end
        else
            velmsg.Angular.Z = -min_acc;
            disp("No Boundary");
        end

    else

        % If no obstacles are found, the robot continues to move forward
        velmsg.Linear.X = forwardVelocity;
        velmsg.Angular.Z = 0;
    end

    
    hold off;
    % If the distance between the obstacles on the left is less than 1, the right turn speed will increase. 
    % The conditions on the right are the same
    if(range(45)<1)
        velmsg.Angular.Z =  velmsg.Angular.Z - min_acc;
    elseif(range(315)<1)
        velmsg.Angular.Z = velmsg.Angular.Z + min_acc;
    end
    send(robot, velmsg);
end


% Shut down ROS
rosshutdown;

plot(robot_x, robot_y);
grid on
title('Odometry')
xlabel('x(m)')
ylabel('y(m)')
axis equal

% Define a function to find the boundary point of the left obstacle
function leftBoundary = findLeftBoundary(range, threshold)
    leftBoundary = NaN;
    for i = 2:1:90
        if range(i)-range(i-1)> threshold && range(i-1)~=Inf
            leftBoundary = i-1;
            break;
        elseif range(i-1)-range(i)> threshold && range(i)~=Inf
            leftBoundary = i;
            break;
        end
    end
end

% Define a function to find the boundary point of the right obstacle
function rightBoundary = findRightBoundary(range, threshold)
    rightBoundary =NaN;
    for i = 360:-1:270
        if range(i-1)-range(i)> threshold && range(i)~=Inf
            rightBoundary = i;
            break;
        elseif range(i)-range(i-1)> threshold && range(i-1)~=Inf
            rightBoundary = i-1;
            break;
        end
    end
end
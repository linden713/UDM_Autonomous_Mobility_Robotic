% Clear MATLAB workspace, close figures, and clear command window

clear variables; close all; clc;
rosshutdown;
% Shutdown any existing ROS nodes

ipaddress= "localhost";
rosinit(ipaddress);
% ROS publishers and subscribers

global SubOdom;
global PubVel;
global SubLiDAR;
global SubModelState;
PubVel = rospublisher('/cmd_vel','geometry_msgs/Twist');
SubOdom = rossubscriber('/odom');
SubModelState = rossubscriber('/gazebo/model_states');
SubLiDAR = rossubscriber("/scan", "sensor_msgs/LaserScan");
PubVel = rospublisher('/cmd_vel','geometry_msgs/Twist');
SubLiDAR = rossubscriber("/scan", "sensor_msgs/LaserScan");
% Initialize velocity and stop messages

global velmsg;
velmsg = rosmessage(PubVel);
velmsg.Linear.X = 0;
velmsg.Angular.Z = 0;
send(PubVel,velmsg);

global stopmsg;
stopmsg = rosmessage(PubVel);
stopmsg.Linear.X = 0;
stopmsg.Angular.Z = 0;
send(PubVel,stopmsg);
% Set initial robot pose in Gazebo

init_pos = rospublisher('/gazebo/set_model_state','gazebo_msgs/ModelState');
init_msg = rosmessage(init_pos);
init_msg.ModelName = "turtlebot3_burger";
init_msg.Pose.Position.X = 0;
init_msg.Pose.Position.Y = 0;
init_msg.Pose.Position.Z = 0;
init_msg.Pose.Orientation.X = 0;
init_msg.Pose.Orientation.Y = 0;
init_msg.Pose.Orientation.Z = 0;
init_msg.Pose.Orientation.W = 0;
send(init_pos,init_msg)

w_max = 4;
v_max = 0.2;

% Initialize global variables for storing positions

global gazebo_x;
global gazebo_y;
global robot_x;
global robot_y;
gazebo_x = [];
gazebo_y = [];
robot_x = [];
robot_y = [];


% Set up a ROS rate for controlling loop frequency

r = rosrate(10);
reset(r)

% Initialize variables for trajectory plotting

x = [];
y = [];
% Flag and parameters for the left turn phase

flag = 1;
k = 0.4;
target_dist = 1;
left=1;
% Left follow

while (flag)
    % Read most recent frame of LiDAR data
    scan = receive(SubLiDAR);
    dist = scan.Ranges;
    % Determine LiDAR data for left or right side
    if(left==1)
        right_dist = dist(1:90);  
        Langle = linspace(1,90,90);
    else
        right_dist = dist(271:end);
        Langle = linspace(271,360,90);
    end
	% Process LiDAR data
    h_dist = abs(right_dist'.*sind(Langle));
    average_dist = mean(h_dist(isfinite(h_dist)));

    % Create a 'polarplot'; convert from deg to rad
    polarplot(deg2rad(Langle), right_dist, '*r');
    % Access polar axes properties using 'gca'
    ax = gca;
    % Change axes property to plot 0 deg on top (matches LiDAR data layout)
    ax.ThetaZeroLocation = 'top';
	% Read odometry data
    Odom = receive(SubOdom);
    [OdomPos(1), OdomPos(2)] = deal(Odom.Pose.Pose.Position.X, ...
        Odom.Pose.Pose.Position.Y);

    % Augment the plot vector with new data point
    x = [x,OdomPos(1)];
    y = [y,OdomPos(2)];
	% Calculate angular velocity based on distance
    if(left==1)
        omiga = k * -(target_dist - average_dist);
    else
        omiga = k * (target_dist - average_dist);
    end
	% Clamp angular velocity within limits
    if omiga>w_max
        omiga = w_max;
    elseif omiga<-w_max
        omiga = -w_max;
    end
    velmsg.Linear.X = v_max;
    velmsg.Angular.Z = omiga;

    send(PubVel,velmsg);

    waitfor(r);

    if (~isfinite(right_dist(90)))
        flag = 0;
        break;
    end
end
% Transition to the right follow
disp("next");
left=0;
flag = 1;
k = 1;
while (flag)
    % Read most recent frame of LiDAR data
    scan = receive(SubLiDAR);
    dist = scan.Ranges;
    % Determine LiDAR data for left or right side
    if(left==1)
        right_dist = dist(1:90);  
        Langle = linspace(1,90,90);
    else
        right_dist = dist(271:end);
        Langle = linspace(271,360,90);
    end
% Process LiDAR data
    h_dist = abs(right_dist'.*sind(Langle));
    average_dist = mean(h_dist(isfinite(h_dist)));

    % Create a 'polarplot'; convert from deg to rad
    polarplot(deg2rad(Langle), right_dist, '*r');
    % Access polar axes properties using 'gca'
    ax = gca;
    % Change axes property to plot 0 deg on top (matches LiDAR data layout)
    ax.ThetaZeroLocation = 'top';

    Odom = receive(SubOdom);
    [OdomPos(1), OdomPos(2)] = deal(Odom.Pose.Pose.Position.X, ...
        Odom.Pose.Pose.Position.Y);

    % Augment the plot vector with new data point
    x = [x,OdomPos(1)];
    y = [y,OdomPos(2)];
	% Calculate angular velocity based on distance
    if(left==1)
        omiga = k * -(target_dist - average_dist);
    else
        omiga = k * (target_dist - average_dist);
    end

    if omiga>w_max
        omiga = w_max;
    elseif omiga<-w_max
        omiga = -w_max;
    end
    velmsg.Linear.X = v_max;
    velmsg.Angular.Z = omiga;
    % Keep sending velocity commands to ensure uninterrupted motion
    send(PubVel,velmsg);

    waitfor(r);

    if (sum(right_dist(isfinite(right_dist)))==0)
        flag = 0;
        break;
    end
end




velmsg.Linear.X = 0;
velmsg.Angular.Z = 0;
send(PubVel,velmsg);

% shut down ROS here
rosshutdown
close
f1 = figure;
plot(x, y);
grid on
title('Odometry')
xlabel('x(m)')
ylabel('y(m)')


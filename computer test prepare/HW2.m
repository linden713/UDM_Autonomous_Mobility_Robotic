% Clear existing variables, close figures, and clear command window

clear variables; close all; clc;
rosshutdown;
% ROS initialization with specified IP address

ipaddress= "localhost";
rosinit(ipaddress);
% Global ROS publishers and subscribers

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
% Global ROS message variables

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
% Initialize robot's initial position in Gazebo

init_pos = rospublisher('/gazebo/set_model_state','gazebo_msgs/ModelState');
init_msg = rosmessage(init_pos);
init_msg.ModelName = "turtlebot3_burger";
init_msg.Pose.Position.X = 0;
init_msg.Pose.Position.Y = 0.;
init_msg.Pose.Position.Z = 0;
init_msg.Pose.Orientation.X = 0;
init_msg.Pose.Orientation.Y = 0;
init_msg.Pose.Orientation.Z = 0;
init_msg.Pose.Orientation.W = 0;
send(init_pos,init_msg)
% Set maximum angular and linear velocities

w_max = 4;
v_max = 0.2;

% Global variables for recording positions

global gazebo_x;
global gazebo_y;
global robot_x;
global robot_y;
gazebo_x = [];
gazebo_y = [];
robot_x = [];
robot_y = [];




% Rotate the robot 

rotate_robot(pi*2);
send(PubVel, stopmsg);
rotate_robot2(pi*2);
send(PubVel, stopmsg);

% shut down ROS here
rosshutdown
close
f1 = figure;
plot(robot_x, robot_y);
grid on
title('Odometry')
xlabel('x(m)')
ylabel('y(m)')

function rotate_robot(goal_theta)
    global SubOdom;
    global SubModelState;
    global PubVel;
    global velmsg;
    global robot_x;
    global robot_y;
    global gazebo_x;
    global gazebo_y;

    Kp = 1;
    target_angle = mod(goal_theta, 2 * pi);

    % Convert goal angle to a valid range
    control_rate = rosrate(10); 
	% Set control rate
    distance=0;
    last_x=0;
    last_y=0;
    while true

        % for record positions
        Odom = receive(SubOdom);
        [OdomPos(1), OdomPos(2)] = deal(Odom.Pose.Pose.Position.X, Odom.Pose.Pose.Position.Y);
        robot_x = [robot_x OdomPos(1)];
        robot_y = [robot_y OdomPos(2)];

        ModelState = receive(SubModelState);
        [GazeboPos(1),GazeboPos(2)] = deal(ModelState.Pose(2,1).Position.X, ModelState.Pose(2,1).Position.Y);
        gazebo_x = [gazebo_x GazeboPos(1)];
        gazebo_y = [gazebo_x GazeboPos(2)];
        % Calculate orientation error
        Odomquat = [Odom.Pose.Pose.Orientation.X Odom.Pose.Pose.Orientation.Y ...
            Odom.Pose.Pose.Orientation.Z Odom.Pose.Pose.Orientation.W];
        
        Odomangle = quat2eul(Odomquat);
        Odomyaw = wrapTo2Pi(Odomangle(3));
        error = Odomyaw-target_angle;
        % Accumulate distance traveled
        distance=distance+norm([OdomPos(1) OdomPos(2)] - [last_x last_y])
        last_x=OdomPos(1);
        last_y=OdomPos(2);


        % Set P-controller output for angular and linear velocity

        
        velmsg.Angular.Z = 0.2;
        velmsg.Linear.X = 0.4;
        send(PubVel, velmsg);
    	% Break the loop if the robot completes the desired rotation
        if distance > 2*pi*2
            
            velmsg.Linear.X = 0;
            velmsg.Angular.Z = 0;
            send(PubVel, velmsg);
            break;
        end
 
        waitfor(control_rate);
    end

end





function rotate_robot2(goal_theta)
    global SubOdom;
    global SubModelState;
    global PubVel;
    global velmsg;
    global robot_x;
    global robot_y;
    global gazebo_x;
    global gazebo_y;

    Kp = 1;
    target_angle = mod(goal_theta, 2 * pi);

    % Convert goal angle to a valid range
    control_rate = rosrate(10); 
	% Set control rate
    distance=0;
    last_x=0;
    last_y=0;
    while true

        % for record positions
        Odom = receive(SubOdom);
        [OdomPos(1), OdomPos(2)] = deal(Odom.Pose.Pose.Position.X, Odom.Pose.Pose.Position.Y);
        robot_x = [robot_x OdomPos(1)];
        robot_y = [robot_y OdomPos(2)];

        ModelState = receive(SubModelState);
        [GazeboPos(1),GazeboPos(2)] = deal(ModelState.Pose(2,1).Position.X, ModelState.Pose(2,1).Position.Y);
        gazebo_x = [gazebo_x GazeboPos(1)];
        gazebo_y = [gazebo_x GazeboPos(2)];
        % Calculate orientation error
        Odomquat = [Odom.Pose.Pose.Orientation.X Odom.Pose.Pose.Orientation.Y ...
            Odom.Pose.Pose.Orientation.Z Odom.Pose.Pose.Orientation.W];
        
        Odomangle = quat2eul(Odomquat);
        Odomyaw = wrapTo2Pi(Odomangle(3));
        error = Odomyaw-target_angle;
        % Accumulate distance traveled
        distance=distance+norm([OdomPos(1) OdomPos(2)] - [last_x last_y])
        last_x=OdomPos(1);
        last_y=OdomPos(2);


        % Set P-controller output for angular and linear velocity

        
        velmsg.Angular.Z = -0.2;
        velmsg.Linear.X = 0.4;
        send(PubVel, velmsg);
    	% Break the loop if the robot completes the desired rotation
        if distance > 2*pi*2
            
            velmsg.Linear.X = 0;
            velmsg.Angular.Z = 0;
            send(PubVel, velmsg);
            break;
        end
 
        waitfor(control_rate);
    end

end



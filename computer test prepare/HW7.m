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
init_msg.Pose.Position.Y = 0;
init_msg.Pose.Position.Z = 0;
init_msg.Pose.Orientation.X = 0;
init_msg.Pose.Orientation.Y = 0;
init_msg.Pose.Orientation.Z = 0;
init_msg.Pose.Orientation.W = 0;
send(init_pos,init_msg)
% Set maximum angular and linear velocities

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
% Flag and parameters 
x = [];
y = [];

k = 0.8;
target_dist = 1.25;
% goto position 1,-3
goto(1,-3);
send(PubVel,stopmsg);
% goto angle pi/2
rotate_robot(pi/2);

count=0;
max_count=0;
temp_x=[];
temp_y=[];
temp_theta=[];
goal_x=0;
goal_y=0;
goal_theta=0;
while (1)
    left=0;
    % for record position
    Odom = receive(SubOdom);
    [OdomPos(1), OdomPos(2)] = deal(Odom.Pose.Pose.Position.X, Odom.Pose.Pose.Position.Y);
    robot_x = [robot_x OdomPos(1)];
    robot_y = [robot_y OdomPos(2)];

    ModelState = receive(SubModelState);
    [GazeboPos(1),GazeboPos(2)] = deal(ModelState.Pose(2,1).Position.X, ModelState.Pose(2,1).Position.Y);
    gazebo_x = [gazebo_x GazeboPos(1)];
    gazebo_y = [gazebo_x GazeboPos(2)];

    % Read most recent frame of LiDAR data
    scan = receive(SubLiDAR);
    dist = scan.Ranges;

    if(left==1)
        right_dist = dist(1:90);  
        Langle = linspace(1,90,90);
    else
        right_dist = dist(271:end);
        Langle = linspace(271,360,90);
    end

    h_dist = abs(right_dist'.*sind(Langle));
    average_dist = mean(h_dist(isfinite(h_dist)));

    % Create a 'polarplot'; convert from deg to rad
    polarplot(deg2rad(Langle), right_dist, '*r');
    ax = gca;
    ax.ThetaZeroLocation = 'top';

  
    Odomquat = [Odom.Pose.Pose.Orientation.X Odom.Pose.Pose.Orientation.Y ...
            Odom.Pose.Pose.Orientation.Z Odom.Pose.Pose.Orientation.W];
    Odomangle = quat2eul(Odomquat);
    Odomyaw = wrapTo2Pi(Odomangle(3));
    % Augment the plot vector with new data point
    x = [x,OdomPos(1)];
    y = [y,OdomPos(2)];

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
    send(PubVel,velmsg);
    %measure how open the gap is
    if (dist(270)==inf)
        count=count + 1
        temp_x = [temp_x,OdomPos(1)];
        temp_y = [temp_y,OdomPos(2)];
        temp_theta = [temp_theta,Odomyaw];

    elseif isfinite(dist(270)) && count~=0
        
        if count>max_count
            max_count=count;
            goal_x=mean(temp_x);
            goal_y=mean(temp_y);
            goal_theta=mean(temp_theta);

        end
        count=0;
        temp_x=[];
        temp_y=[];
    end
    %if there is no wall ->exit
    if (sum(right_dist(isfinite(right_dist)))==0) || ((norm([OdomPos(1) OdomPos(2)] - [0 0.75]) < 0.5)&& (goal_x~=0))
        disp("a");
        break;
    end

    waitfor(r);

end

send(PubVel, stopmsg);
goto(goal_x,goal_y)
send(PubVel,stopmsg);
rotate_robot(goal_theta-pi/2);
send(PubVel, stopmsg);
move_robot_straight(10)

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

    % set control rate
    control_rate = rosrate(10); 

    while true
        % for record the traj
        Odom = receive(SubOdom);
        [OdomPos(1), OdomPos(2)] = deal(Odom.Pose.Pose.Position.X, Odom.Pose.Pose.Position.Y);
        robot_x = [robot_x OdomPos(1)];
        robot_y = [robot_y OdomPos(2)];

        ModelState = receive(SubModelState);
        [GazeboPos(1),GazeboPos(2)] = deal(ModelState.Pose(2,1).Position.X, ModelState.Pose(2,1).Position.Y);
        gazebo_x = [gazebo_x GazeboPos(1)];
        gazebo_y = [gazebo_x GazeboPos(2)];

        % find robot yaw and calculate error
        Odomquat = [Odom.Pose.Pose.Orientation.X Odom.Pose.Pose.Orientation.Y ...
            Odom.Pose.Pose.Orientation.Z Odom.Pose.Pose.Orientation.W];
        
        Odomangle = quat2eul(Odomquat);
        Odomyaw = wrapTo2Pi(Odomangle(3));
        error = Odomyaw-target_angle;

        % use KP to control
        control_output = -Kp * error;
        velmsg.Angular.Z = control_output;
        velmsg.Linear.X = 0;
        send(PubVel, velmsg);

        if abs(error) < 0.01
            disp('Reached target angle');
            velmsg.Linear.X = 0;
            velmsg.Angular.Z = 0;
            send(PubVel, velmsg);
            break;
        end
 
        waitfor(control_rate);
    end

end

function move_robot_straight(distance)
  
    global SubOdom;
    global SubModelState;
    global PubVel;
    global velmsg;
    global robot_x;
    global robot_y;
    global gazebo_x;
    global gazebo_y;
    
    Kp = 1;
    Odom = receive(SubOdom);
    [init_pos(1), initPos(2)] = deal(Odom.Pose.Pose.Position.X, Odom.Pose.Pose.Position.Y);

    control_rate = rosrate(10);
    velmsg.Linear.X = 0.1;
            velmsg.Angular.Z = 0;
            send(PubVel, velmsg);
    while true
        % for record
        Odom = receive(SubOdom);
        [OdomPos(1), OdomPos(2)] = deal(Odom.Pose.Pose.Position.X, Odom.Pose.Pose.Position.Y);
        robot_x = [robot_x OdomPos(1)];
        robot_y = [robot_y OdomPos(2)];

        ModelState = receive(SubModelState);
        [GazeboPos(1),GazeboPos(2)] = deal(ModelState.Pose(2,1).Position.X, ModelState.Pose(2,1).Position.Y);
        gazebo_x = [gazebo_x GazeboPos(1)];
        gazebo_y = [gazebo_x GazeboPos(2)];


        error_x = distance-norm([OdomPos(1) OdomPos(2)] - [init_pos(1) initPos(2)]);
        control_output_x = Kp * error_x;

        if control_output_x>0.2
            control_output_x=0.2;
        end
         
        velmsg.Angular.Z = 0;
        velmsg.Linear.X = control_output_x;
        send(PubVel, velmsg);

        % if reached brek the loop
        if abs(error_x) < 0.01
            disp('Reached target position');
            velmsg.Linear.X = 0;
            velmsg.Angular.Z = 0;
            send(PubVel, velmsg);
            break;
        end

        % Check if there is wall
        global SubLiDAR;
        scan = receive(SubLiDAR);
        dist = scan.Ranges;
        dist(91:269) = inf; 
        if all(isinf(dist))
            velmsg.Linear.X = 0;
            velmsg.Angular.Z = 0;
            send(PubVel, velmsg);
            break;
        end
        waitfor(control_rate);
    end
end

function goto(x, y)  
    
    k=0.7;
    v_max=0.4;
    global SubOdom;
    global SubModelState;
    global PubVel;
    global velmsg;
    global robot_x;
    global robot_y;
    global gazebo_x;
    global gazebo_y;

    r = rosrate(10);
    while (1)
        % for record position
        Odom = receive(SubOdom);
        [OdomPos(1), OdomPos(2)] = deal(Odom.Pose.Pose.Position.X, Odom.Pose.Pose.Position.Y);
        robot_x = [robot_x OdomPos(1)];
        robot_y = [robot_y OdomPos(2)];

        ModelState = receive(SubModelState);
        [GazeboPos(1),GazeboPos(2)] = deal(ModelState.Pose(2,1).Position.X, ModelState.Pose(2,1).Position.Y);
        gazebo_x = [gazebo_x GazeboPos(1)];
        gazebo_y = [gazebo_x GazeboPos(2)];

	
        Odomquat = [Odom.Pose.Pose.Orientation.X Odom.Pose.Pose.Orientation.Y ...
            Odom.Pose.Pose.Orientation.Z Odom.Pose.Pose.Orientation.W];      
        Odomangle = quat2eul(Odomquat);
        Odomyaw = wrapTo2Pi(Odomangle(3));
        omiga = k * wrapToPi(wrapTo2Pi(atan2(y - OdomPos(2),x - OdomPos(1))) - Odomyaw);

        velmsg.Linear.X = v_max;
        velmsg.Angular.Z = omiga;
        send(PubVel, velmsg);
    
        if norm([OdomPos(1) OdomPos(2)] - [x y]) < 0.1
            velmsg.Linear.X = 0;
            velmsg.Angular.Z = 0;
            send(PubVel, velmsg);
            break;
        end   
        waitfor(r);
    end
end

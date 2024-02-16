clear variables; close all; clc;

rosshutdown; % ends ROS if it is already running
global SubOdom;
global PubVel;
global velmsg;
global SubLiDAR;
ipaddress= "localhost";
rosinit(ipaddress)
PubVel = rospublisher('/cmd_vel','geometry_msgs/Twist');
velmsg = rosmessage(PubVel);
velmsg.Linear.X = 0;
velmsg.Angular.Z = 0;
send(PubVel,velmsg);

init_pos = rospublisher('/gazebo/set_model_state','gazebo_msgs/ModelState');
init_msg = rosmessage(init_pos);
init_msg.ModelName = "turtlebot3_burger";
init_msg.Pose.Position.X = 0;
init_msg.Pose.Position.Y = 0.;
init_msg.Pose.Position.Z = 0;
init_msg.Pose.Orientation.X = 0;
init_msg.Pose.Orientation.Y = 0;
init_msg.Pose.Orientation.Z = 0;
init_msg.Pose.Orientation.W = 1;
send(init_pos,init_msg)

% tftree = rostf;
% pause(1);
w_max = 4;
v_max = 0.2;
% Create a Publisher for velocity commmands & two Subscribers for ...
% Odometry and Model State data
% Following pertains to the Turtlebot3
PubVel = rospublisher('/cmd_vel','geometry_msgs/Twist');
SubOdom = rossubscriber('/odom');
% Create a Subscriber for LiDAR topic
SubLiDAR = rossubscriber("/scan", "sensor_msgs/LaserScan");
xGazebo = [];
yGazebo = [];
% Create the velocity message container variable
velmsg = rosmessage(PubVel);
stopmsg = rosmessage(PubVel);
stopmsg.Linear.X = 0;
stopmsg.Angular.Z = 0;
r = rosrate(10);
reset(r)

x = [];
y = [];

k = 0.8;
target_dist = 1.25;

count=0;
max_count=0;
temp_x=[];
temp_y=[];
temp_theta=[];
goal_x=0;
goal_y=0;
goal_theta=0;
while (1)
    % Read most recent frame of LiDAR data
    scan = receive(SubLiDAR);
    dist = scan.Ranges;
    right_dist = dist(1:90);  % dist(271:end);

    Langle = linspace(1,90,90);
    h_dist = abs(right_dist'.*sind(Langle));
    average_dist = mean(h_dist(isfinite(h_dist)));

    % Create a 'polarplot'; convert from deg to rad
    polarplot(deg2rad(Langle), right_dist, '*r');
    ax = gca;
    ax.ThetaZeroLocation = 'top';

    Odom = receive(SubOdom);
    [OdomPos(1), OdomPos(2)] = deal(Odom.Pose.Pose.Position.X, ...
        Odom.Pose.Pose.Position.Y);
    Odomquat = [Odom.Pose.Pose.Orientation.X Odom.Pose.Pose.Orientation.Y ...
            Odom.Pose.Pose.Orientation.Z Odom.Pose.Pose.Orientation.W];
    Odomangle = quat2eul(Odomquat);
    Odomyaw = wrapTo2Pi(Odomangle(3));
    % Augment the plot vector with new data point
    x = [x,OdomPos(1)];
    y = [y,OdomPos(2)];

    omiga = k * -(target_dist - average_dist);

    if omiga>w_max
        omiga = w_max;
    elseif omiga<-w_max
        omiga = -w_max;
    end
    velmsg.Linear.X = v_max;
    velmsg.Angular.Z = omiga;
    % Keep sending velocity commands to ensure uninterrupted motion
    send(PubVel,velmsg);

    if (dist(90)==inf)
        count=count + 1
        temp_x = [temp_x,OdomPos(1)];
        temp_y = [temp_y,OdomPos(2)];
        temp_theta = [temp_theta,Odomyaw];

    elseif isfinite(dist(90)) && count~=0
        
        if count>max_count
            max_count=count;
            goal_x=mean(temp_x)
            goal_y=mean(temp_y)
            goal_theta=mean(temp_theta);

        end
        count=0;
        temp_x=[];
        temp_y=[];
    end
    
    if (sum(right_dist(isfinite(right_dist)))==0) || ((norm([OdomPos(1) OdomPos(2)] - [0 0.75]) < 0.5)&& (goal_x~=0))
        break;
    end

    waitfor(r);

end

send(PubVel, stopmsg);
goto(goal_x,goal_y)
send(PubVel,stopmsg);
rotate_robot(goal_theta);
send(PubVel, stopmsg);
move_robot_straight(100)

% shut down ROS here
rosshutdown
close
f1 = figure;
plot(x, y);
grid on
title('Odometry')
xlabel('x(m)')
ylabel('y(m)')

function goto(x, y)  
    % global robot_x;
    % global robot_y;
    k=1.5;
    v_max=0.3;
    global SubOdom;
    global PubVel;
    global velmsg;

    r = rosrate(10);
    while (1)
        Odom = receive(SubOdom);
        [OdomPos(1), OdomPos(2)] = deal(Odom.Pose.Pose.Position.X, Odom.Pose.Pose.Position.Y);
        % robot_x = [robot_x OdomPos(1)];
        % robot_y = [robot_y OdomPos(2)];
        Odomquat = [Odom.Pose.Pose.Orientation.X Odom.Pose.Pose.Orientation.Y ...
            Odom.Pose.Pose.Orientation.Z Odom.Pose.Pose.Orientation.W];
        
        Odomangle = quat2eul(Odomquat);
        Odomyaw = wrapTo2Pi(Odomangle(3));
        dy = y - OdomPos(2);
        dx = x - OdomPos(1);
        wrapTo2Pi(cart2pol(x - OdomPos(1),y - OdomPos(2)));
        omiga = k * (wrapTo2Pi(cart2pol(x - OdomPos(1),y - OdomPos(2))) - Odomyaw);

        velmsg.Linear.X = v_max;
        velmsg.Angular.Z = omiga;
        send(PubVel, velmsg);
    
        if norm([OdomPos(1) OdomPos(2)] - [x y]) < 0.06
            velmsg.Linear.X = 0;
            velmsg.Angular.Z = 0;
            send(PubVel, velmsg);
            break;
        end



        
        waitfor(r);
    end
end
function rotate_robot(goal_theta)
    global SubOdom;
    global PubVel;
    global velmsg;
    
    Kp = 1;

    target_angle = mod(goal_theta + (pi/2), 2 * pi);

    % 设置控制周期
    control_rate = rosrate(10); 

    while true
        Odom = receive(SubOdom);
    % [OdomPos(1), OdomPos(2)] = deal(Odom.Pose.Pose.Position.X, Odom.Pose.Pose.Position.Y);
        % robot_x = [robot_x OdomPos(1)];
        % robot_y = [robot_y OdomPos(2)];
    Odomquat = [Odom.Pose.Pose.Orientation.X Odom.Pose.Pose.Orientation.Y ...
            Odom.Pose.Pose.Orientation.Z Odom.Pose.Pose.Orientation.W];
        
    Odomangle = quat2eul(Odomquat);
    Odomyaw = wrapTo2Pi(Odomangle(3));
        error = Odomyaw-target_angle;

        % 计算P控制器输出
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
    global PubVel;
    global velmsg;
    
    Kp = 1;
    Odom = receive(SubOdom);
    [init_pos(1), initPos(2)] = deal(Odom.Pose.Pose.Position.X, Odom.Pose.Pose.Position.Y);

    control_rate = rosrate(10);

    while true
        Odom = receive(SubOdom);
    [OdomPos(1), OdomPos(2)] = deal(Odom.Pose.Pose.Position.X, Odom.Pose.Pose.Position.Y);

        error_x = distance-norm([OdomPos(1) OdomPos(2)] - [init_pos(1) initPos(2)]);
        control_output_x = Kp * error_x;
        if control_output_x>0.2
            control_output_x=0.2;
        end
         
        velmsg.Angular.Z = 0;
        velmsg.Linear.X = control_output_x;
        send(PubVel, velmsg);

        % 如果小车到达目标位置，退出循环
        if abs(error_x) < 0.01
            disp('Reached target position');
            velmsg.Linear.X = 0;
            velmsg.Angular.Z = 0;
            send(PubVel, velmsg);
            break;
        end
        global SubLiDAR;
        scan = receive(SubLiDAR);
        dist = scan.Ranges;
        if all(isinf(dist))
            velmsg.Linear.X = 0;
            velmsg.Angular.Z = 0;
            send(PubVel, velmsg);
            break;
        end

        waitfor(control_rate);
    end
end


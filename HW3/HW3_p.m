% HW2: Open Loop vs Closed Loop
% Partial solution only - single forward & single turn motions

% Clean up!
clear variables; close all; clc;

rosshutdown; % ends ROS if it is already running

%   Initialize ROS (that is get 'roscore' going)
%   If running MATLAB on a host computer and a VM with ROS on a guest
%   computer pick up correct IP address of guest computer using "ipconfig"
%   command in Ubuntu terminal. Then, replace IP address in next line.
%   For native installation of Ubuntu, use the following instead by
%   commenting out the previous statement and uncommenting this one.
ipaddress= "localhost";
rosinit(ipaddress)
init_pos = rospublisher('/gazebo/set_model_state','gazebo_msgs/ModelState');
init_msg = rosmessage(init_pos);
init_msg.ModelName = "turtlebot3_burger";
init_msg.Pose.Position.X = 9;
init_msg.Pose.Position.Y = 8;
init_msg.Pose.Position.Z = 0;
init_msg.Pose.Orientation.X = 0;
init_msg.Pose.Orientation.Y = 0;

init_msg.Pose.Orientation.Z = 0.7071;
init_msg.Pose.Orientation.W = 0.7071;
send(init_pos,init_msg)
pause(1)

% tftree = rostf;
% pause(1);
w_max = 5.4;
v_max = 0.4;
% Create a Publisher for velocity commmands & two Subscribers for ...
% Odometry and Model State data
% Following pertains to the Turtlebot3
PubVel = rospublisher('/cmd_vel','geometry_msgs/Twist');
SubOdom = rossubscriber('/odom');
SubModelState = rossubscriber('/gazebo/model_states');
pause(2)
x = [];
y = [];
hw = [];
tt = [];
xGazebo = [];
yGazebo = [];
% Create the velocity message container variable
velmsg = rosmessage(PubVel);


%   Create rate object to run loop at 10 Hz; loop duration = 1000/10 = 100 ms
r = rosrate(50);
% initialize
reset(r)
Odomyaw = pi/2;
% Drive forward for estimated time
rTT = 0;
xi = [9 7 5 1];
yi = [8 1 3 9];
x = [];
y = [];
vi = [];
wi = [];
vr = [];
wr = [];
flag = 1;
k = 0.6;
while (flag)
    Odom = receive(SubOdom);
    [OdomPos(1), OdomPos(2)] = deal(Odom.Pose.Pose.Position.X, ...
        Odom.Pose.Pose.Position.Y);

    Odomquat = [Odom.Pose.Pose.Orientation.X Odom.Pose.Pose.Orientation.Y ...
        Odom.Pose.Pose.Orientation.Z Odom.Pose.Pose.Orientation.W];
    vvr = Odom.Twist.Twist.Linear.X;
    vr = [vr vvr];
    wrr = Odom.Twist.Twist.Angular.Z;
    wr = [wr wrr];
    % Augment the plot vector with new data point

    x = [x OdomPos(1)];
    y = [y OdomPos(2)];
    Odomangle = quat2eul(Odomquat);
    Odomyaw = wrapTo2Pi(Odomangle(3));
    hw = [hw Odomyaw];
    omiga = k *(atan2((OdomPos(2)-1),(OdomPos(1)-7))+pi-Odomyaw);
    velmsg.Linear.X = v_max;
    velmsg.Angular.Z = omiga;
    % Keep sending velocity commands to ensure uninterrupted motion
    send(PubVel,velmsg);
    vi = [vi v_max];
    wi = [wi omiga];
    rTT = r.TotalElapsedTime;
    tt = [tt rTT];
    % Update the plot variables here
    % Get fresh odomometer reading
    ModelState = receive(SubModelState);
    [GazeboPos(1),GazeboPos(2)] = deal(ModelState.Pose(2,1).Position.X, ...
        ModelState.Pose(2,1).Position.Y);

    xGazebo = [xGazebo GazeboPos(1)];
    yGazebo = [yGazebo GazeboPos(2)];
    if norm([OdomPos(1) OdomPos(2)]-[7 1])<0.25
        flag = 0;
    end
    % Burn the rest of the loop time
    waitfor(r);
end
flag = 1;
while (flag)
    Odom = receive(SubOdom);
    [OdomPos(1), OdomPos(2)] = deal(Odom.Pose.Pose.Position.X, ...
        Odom.Pose.Pose.Position.Y);

    Odomquat = [Odom.Pose.Pose.Orientation.X Odom.Pose.Pose.Orientation.Y ...
        Odom.Pose.Pose.Orientation.Z Odom.Pose.Pose.Orientation.W];
    vvr = Odom.Twist.Twist.Linear.X;
    vr = [vr vvr];
    wrr = Odom.Twist.Twist.Angular.Z;
    wr = [wr wrr];
    % Augment the plot vector with new data point
    x = [x OdomPos(1)];
    y = [y OdomPos(2)];
    Odomangle = quat2eul(Odomquat);
    Odomyaw = wrapTo2Pi(Odomangle(3));
    hw = [hw Odomyaw];
    omiga = k *(atan2((4-OdomPos(2)),(5-OdomPos(1)))-Odomyaw);
    velmsg.Linear.X = v_max;
    velmsg.Angular.Z = omiga;
    % Keep sending velocity commands to ensure uninterrupted motion
    vi = [vi v_max];
    wi = [wi omiga];
    send(PubVel,velmsg);
    rTT = r.TotalElapsedTime;
    tt = [tt rTT];
    % Update the plot variables here
    % Get fresh odomometer reading
    ModelState = receive(SubModelState);
    [GazeboPos(1),GazeboPos(2)] = deal(ModelState.Pose(2,1).Position.X, ...
        ModelState.Pose(2,1).Position.Y);

    xGazebo = [xGazebo GazeboPos(1)];
    yGazebo = [yGazebo GazeboPos(2)];
    if norm([OdomPos(1) OdomPos(2)]-[5 4])<0.25
        flag = 0;
    end
    % Burn the rest of the loop time
    waitfor(r);
end
flag = 1;
while (flag)
    Odom = receive(SubOdom);
    [OdomPos(1), OdomPos(2)] = deal(Odom.Pose.Pose.Position.X, ...
        Odom.Pose.Pose.Position.Y);

    Odomquat = [Odom.Pose.Pose.Orientation.X Odom.Pose.Pose.Orientation.Y ...
        Odom.Pose.Pose.Orientation.Z Odom.Pose.Pose.Orientation.W];
    vvr = Odom.Twist.Twist.Linear.X;
    vr = [vr vvr];
    wrr = Odom.Twist.Twist.Angular.Z;
    wr = [wr wrr];
    % Augment the plot vector with new data point
    x = [x OdomPos(1)];
    y = [y OdomPos(2)];
    Odomangle = quat2eul(Odomquat);
    Odomyaw = wrapTo2Pi(Odomangle(3));
    hw = [hw Odomyaw];
    omiga = k *(atan2((9-OdomPos(2)),(1-OdomPos(1)))-Odomyaw);
    velmsg.Linear.X = v_max;
    velmsg.Angular.Z = omiga;
    % Keep sending velocity commands to ensure uninterrupted motion
    vi = [vi v_max];
    wi = [wi omiga];
    send(PubVel,velmsg);
    rTT = r.TotalElapsedTime;
    tt = [tt rTT];
    % Update the plot variables here
    % Get fresh odomometer reading
    ModelState = receive(SubModelState);
    [GazeboPos(1),GazeboPos(2)] = deal(ModelState.Pose(2,1).Position.X, ...
        ModelState.Pose(2,1).Position.Y);

    xGazebo = [xGazebo GazeboPos(1)];
    yGazebo = [yGazebo GazeboPos(2)];
    if norm([OdomPos(1) OdomPos(2)]-[1 9])<0.25
        flag = 0;
    end
    % Burn the rest of the loop time
    waitfor(r);
end
% shut down ROS here

rosshutdown
T = tt(end)

subplot(4,1,1);
plot(x, y);
grid on

title('Odometry')
xlabel('x(m)')
ylabel('y(m)')
pbaspect([1 1 1])
axis equal


subplot(4,1,2);
plot(tt, hw);
grid on
title('Heading Angle');
xlabel('t')
ylabel('\Theta (t)')


subplot(4,1,3);
plot(tt, vi);
hold on
plot(tt, wi);
grid on
title('Commanded Robot Velocities')
xlabel('t')
ylabel('v(t) \omega (t)');
legend('v(t)','\omega (t)');
hold off
xr = [];
for i= 1: numel(x)-1
    dis = norm([xGazebo(i+1) yGazebo(i+1)]-[xGazebo(i) yGazebo(i)]);
    xr = [xr dis];
end
xr = sum(xr)
% for i= 1: numel(x)-1
%     wr(i) = (hw(i+1)-hw(i))/(1/50);
% end
% vr = [0 vr];
% wr = [0 wr];

subplot(4,1,4);
plot(tt, vr);
hold on;
plot(tt,wr)
grid on
title('Resultant Robot Velocities')
xlabel('t')
ylabel('v(t) \omega (t)');
legend('v(t)','\omega (t)');
hold off
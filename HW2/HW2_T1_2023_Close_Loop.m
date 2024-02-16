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

% Create a Publisher for velocity commmands & two Subscribers for ...
% Odometry and Model State data
% Following pertains to the Turtlebot3
PubVel = rospublisher('/cmd_vel','geometry_msgs/Twist');
SubOdom = rossubscriber('/odom');
SubModelState = rossubscriber('/gazebo/model_states');
pause(2)

% Create the velocity message container variable
velmsg = rosmessage(PubVel);
velStopmsg = rosmessage(PubVel);
velTurnmsg = rosmessage(PubVel);
% Set values for pure forward motion
VelStraight = 0.2; % forward velocity (m/s)
RotStraight = 0; % rotational rate (rad/s)

% Set values for pure turn motion
VelTurn = 0;
RotTurn = pi/40;

% Initialize data for task
% You may ge lewant to generalize this further including using a function for
% straight driving and a function for pure turn
dS = 3; % next edngth
angT = 60; % next angle of triangle in degrees
aT = 180-angT; % next turn angle in degrees
% Convert to radians
% aTR = (180-aT)*pi/180; % next turn angle in radians

% Read initial odom data
Odom = receive(SubOdom);
% Read initial model state data
ModelState = receive(SubModelState);

% Deconstruct odom data to obtain initial position
[OdomPos(1), OdomPos(2)] = deal(Odom.Pose.Pose.Position.X, ...
    Odom.Pose.Pose.Position.Y);
[GazeboPos(1),GazeboPos(2)] = deal(ModelState.Pose(2,1).Position.X, ...
    ModelState.Pose(2,1).Position.Y);
%   Initialize the plot variables here
%   Pre-allocation with appropriate size will increase efficiency; later!
%   For now.
x = OdomPos(1);
y = OdomPos(2);
xGazebo = GazeboPos(1);
yGazebo = GazeboPos(2);
% Fill velocities for forward/turn/stop motion in appropriate variable
velmsg.Linear.X = VelStraight;
velmsg.Angular.Z = RotStraight;
velStopmsg.Linear.X = 0;
velStopmsg.Angular.Z = 0;
velTurnmsg.Linear.X = VelTurn;
velTurnmsg.Angular.Z = RotTurn;



 for i=1:3
    %   Create rate object to run loop at 50 Hz;
    r = rosrate(50);
    reset(r) % initialize
    % Get odom and modelstate data
     Odom = receive(SubOdom);
     [InitOdomPos(1), InitOdomPos(2)] = deal(Odom.Pose.Pose.Position.X, ...
     Odom.Pose.Pose.Position.Y);
     ModelState = receive(SubModelState);
     [InitGazeboPos(1),InitGazeboPos(2)] = deal(ModelState.Pose(2,1).Position.X, ...
     ModelState.Pose(2,1).Position.Y);
    % Set distance = 0
     distanceGazebo = 0;
     distanceOdom = 0;
    rTT = 0;
    % Drive forward till distance>3*0.99
    while (distanceGazebo < 3*0.99)
        %  Keep sending velocity commands to ensure uninterrupted motion
        send(PubVel,velmsg);
        rTT = r.TotalElapsedTime;
        %   Update the plot variables here
        % Get fresh odomometer reading
        Odom = receive(SubOdom);
        [OdomPos(1), OdomPos(2)] = deal(Odom.Pose.Pose.Position.X, ...
        Odom.Pose.Pose.Position.Y);
        % Augment the plot vector with new data point
        x = [x OdomPos(1)];
        y = [y OdomPos(2)];
        % Do the same for ModelSate
        ModelState = receive(SubModelState);
        [GazeboPos(1),GazeboPos(2)] = deal(ModelState.Pose(2,1).Position.X, ...
    ModelState.Pose(2,1).Position.Y);
        xGazebo = [xGazebo GazeboPos(1)];
        yGazebo = [yGazebo GazeboPos(2)];
        % Calculate the distance
        distanceGazebo = sqrt((InitGazeboPos(1)-GazeboPos(1))^2 + (InitGazeboPos(2)-GazeboPos(2))^2);
        distanceOdom = sqrt((InitOdomPos(1)-OdomPos(1))^2 + (InitOdomPos(2)-OdomPos(2))^2);
        waitfor(r);
    end
    
    %   Stop the robot
    send(PubVel,velStopmsg);
    pause(0.5);
    %   Initialize loop again
    rTT=0;
    %   Initialize time
    reset(r)
    % First turn
    if i==1
    Odomyaw=0;
    while (Odomyaw < 2*pi/3*0.99)
        % Keep sending velocity commands to ensure uninterrupted motion
        send(PubVel,velTurnmsg);
        rTT = r.TotalElapsedTime;
        %   Update the plot variables here
        % Get fresh odomometer reading
        Odom = receive(SubOdom);
        [OdomPos(1), OdomPos(2)] = deal(Odom.Pose.Pose.Position.X, ...
        Odom.Pose.Pose.Position.Y);
        % Augment the plot vector with new data point
        x = [x OdomPos(1)];
        y = [y OdomPos(2)];
        % Try to get robot's yaw angle
        Odomquat = [Odom.Pose.Pose.Orientation.X Odom.Pose.Pose.Orientation.Y ...
    Odom.Pose.Pose.Orientation.Z Odom.Pose.Pose.Orientation.W];
        Odomangle = quat2eul(Odomquat);
        Odomyaw = Odomangle(3)
        ModelState = receive(SubModelState);
        % Get fresh modelstate reading
        [GazeboPos(1),GazeboPos(2)] = deal(ModelState.Pose(2,1).Position.X, ...
    ModelState.Pose(2,1).Position.Y);
        xGazebo = [xGazebo GazeboPos(1)];
        yGazebo = [yGazebo GazeboPos(2)];
        % Try to get robot's yaw angle
        Gazeboquat = [ModelState.Pose(2,1).Orientation.X ModelState.Pose(2,1).Orientation.Y ...
    ModelState.Pose(2,1).Orientation.Z ModelState.Pose(2,1).Orientation.W];
        GazeboAngle = quat2eul(Gazeboquat);
        Gazeboyaw=GazeboAngle(3)
        % Burn the rest of the loop time
        waitfor(r);
    end
    % Second turn
    elseif i==2

    while ((Odomyaw >pi/2 && Odomyaw<pi) || Odomyaw <-2*pi/3*0.99)
        % Keep sending velocity commands to ensure uninterrupted motion
        send(PubVel,velTurnmsg);
        rTT = r.TotalElapsedTime;
        %   Update the plot variables here
        % Get fresh odomometer reading
        Odom = receive(SubOdom);
        [OdomPos(1), OdomPos(2)] = deal(Odom.Pose.Pose.Position.X, ...
        Odom.Pose.Pose.Position.Y);
        % Augment the plot vector with new data point
        x = [x OdomPos(1)];
        y = [y OdomPos(2)];
        % Try to get robot's yaw angle
        Odomquat = [Odom.Pose.Pose.Orientation.X Odom.Pose.Pose.Orientation.Y ...
    Odom.Pose.Pose.Orientation.Z Odom.Pose.Pose.Orientation.W];
        Odomangle = quat2eul(Odomquat);
        Odomyaw = Odomangle(3)
        % Get fresh modelstate reading
        ModelState = receive(SubModelState);
        [GazeboPos(1),GazeboPos(2)] = deal(ModelState.Pose(2,1).Position.X, ...
    ModelState.Pose(2,1).Position.Y);
        xGazebo = [xGazebo GazeboPos(1)];
        yGazebo = [yGazebo GazeboPos(2)];
        % Try to get robot's yaw angle
        Gazeboquat = [ModelState.Pose(2,1).Orientation.X ModelState.Pose(2,1).Orientation.Y ...
    ModelState.Pose(2,1).Orientation.Z ModelState.Pose(2,1).Orientation.W];
        GazeboAngle = quat2eul(Gazeboquat);
        Gazeboyaw=GazeboAngle(3)
        % Burn the rest of the loop time
        waitfor(r);
    end
    send(PubVel,velStopmsg);
    pause(0.5);

    
    end
end

%   Stop the robot

send(PubVel,velStopmsg);

%   Since loop exit is on mission completion, you can shut down ROS here

rosshutdown

%   Plot the accumulated values in the plot variable here
subplot(1,2,1);
plot(x, y);
grid on

title('Odometry')
xlabel('x(m)')
ylabel('y(m)')
pbaspect([1 1 1])
axis equal


subplot(1,2,2);
plot(xGazebo,yGazebo);
grid on

title('Model State')
xlabel('x(m)')
ylabel('y(m)')
pbaspect([1 1 1])
axis equal

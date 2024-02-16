# HW2

Author: Chenghao Lin, Naian Tao

# 1

1.
   <img src="/home/tao/HW_2/open.jpg" style="zoom:150%;" />

2. Explanation of algorithm
   ```matlab
   VelStraight = 0.2;
   StraightTime = 3/VelStraight;
   RotTurn = pi/10;
   RotTime = (120*pi/180)/RotTurn;
   for i=1:3
   	Publish forward speed for "StraightTime" seconds;
   	Publish rotation speed for "RotTime" seconds;
   end
   ```
<div STYLE="page-break-after: always;"></div>
# 2
1. 
   ![](/home/tao/HW_2/close.jpg)
   
2. Explanation of algorithm
   ```matlab
   for i=1:3 % Three stages
   	distanceOdom = 0; % the Distance from the Current Position to the Initial Position of the Stage
   	renew InitOdomPos; % Starting Position for Each Stage
       while (distanceOdom < 3)
       	Publish forward speed;
       	Renew distanceOdom; %Calculate the Distance from the Current Position to the Initial Position of the Stage
       end
       if i==1
       	Odomyaw=0; % Robot's Orientation
       	while (Odomyaw < 2*pi/3)
       		Publish rotation speed;
       		Renew Odomyaw;
       	end
       elseif i==2
           while ((Odomyaw >pi/2 && ...
           		Odomyaw<pi) || Odomyaw <-2*pi/3)
               Publish rotation speed;
       		Renew Odomyaw;
           end
       end
   end
   ```

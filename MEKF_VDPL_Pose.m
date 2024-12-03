
% produced by Ozan Vahit Altınpınar (altinpinaro@itu.edu.tr) (2024)
% Matlab script MEKF_VDPL_Pose.m
% Description: This script generates the position estimation vector of the MEKF_VDPL localization algorithm in a format that can be broadcast to the ROS environment.

clc
clear all;

Nx1 = -3.175;
Nx2 = 2.875;
Ny1 = -2.775 ;
Ny2 = 2.725;

%Error parameters of the robot

alfa1=0.15;
alfa2=0.15;
alfa3=0.20;
alfa4=0.20;


Th = 0.002; %The threshold value in the VDPL algorithm. If there are dynamic obstacles or unexpected static objects in the environment, it is recommended to set this value to 0.01.
kal_noise = 0;
Error_pose_cov = [0.01 0 0;0 0.01 0;0 0 0.01]; % Initial value of the position error covariance matrix of the robot.


% measurement model noise parameters
measurement_noise = 0.02;
bearing_noise = 0.0075;
Qt = [measurement_noise ^2 0 ;0 bearing_noise^2];

currentTime = 0;
d_robot = 0.15;
dmax = 3.5; %maximum range of the LiDAR
range_intervals = [0 dmax];



rosshutdown  
rosinit('................') % Enter the IP address of your computer here
rosparam('set','/use_sim_time',true); % In real environment experiments, this should be "false".


myPub2 = rospublisher("/MEKF_pose",'geometry_msgs/PoseWithCovarianceStamped');
mySub = rossubscriber("/gazebo/model_states","DataFormat","struct");
mySub2 = rossubscriber("/scan","DataFormat","struct");
mySub3 = rossubscriber("/odom","DataFormat","struct");
mySub4 = rossubscriber("/cmd_vel","DataFormat","struct");
mySubMap = rossubscriber("/map","nav_msgs/OccupancyGrid");
msgMap = receive(mySubMap,1);

msgStruct = struct(msgMap);
map_robot = rosReadBinaryOccupancyGrid(msgStruct);


figure(1)
show(map_robot)
axis([Nx1 Nx2 Ny1 Ny2])


msgrobot = mySub.LatestMessage;
    
    q2 = msgrobot.Pose(3).Orientation.X;
    q3 = msgrobot.Pose(3).Orientation.Y;
    q4 = msgrobot.Pose(3).Orientation.Z;
    q1 = msgrobot.Pose(3).Orientation.W;

   robot_teta=atan2((2*(q2.*q3 + q1.*q4)),(2*(q1.*q1 + q2.*q2)-1));
   % To compensate for the offset between the map and the real environment, the following off_set values are set.
   off_set_x = 0.025;
   off_set_y = 0.015;
   l = 0.06;

    robot_x = msgrobot.Pose(3).Position.X - l*cos(robot_teta) + off_set_x;
    robot_y = msgrobot.Pose(3).Position.Y - l*sin(robot_teta)  + off_set_y;
 
    % the initial positions are being assigned.
    mekf_x = robot_x;
    mekf_y = robot_y;
    mekf_teta = robot_teta ;
    
    odom_x = robot_x;
    odom_y = robot_y;
    odom_teta = robot_teta;

i = 1;
scan_time(i) = 0;
odom_time(i) = 0;
currentTime(i) = 0;
key = 0;

Covrc = zeros(6,6);

poseMsg = rosmessage(myPub2);
poseMsg.Header.FrameId = "map";
poseMsg.Pose.Pose.Position.X = mekf_x;
poseMsg.Pose.Pose.Position.Y = mekf_y;
poseMsg.Pose.Pose.Position.Z = 0;
     
my_orientation = eul2quat([(mekf_teta) 0 0], "ZYX");
     
poseMsg.Pose.Pose.Orientation.X = my_orientation(2);
poseMsg.Pose.Pose.Orientation.Y = my_orientation(3);
poseMsg.Pose.Pose.Orientation.Z = my_orientation(4);
poseMsg.Pose.Pose.Orientation.W = my_orientation(1);
     
Covrc(1,1) = Error_pose_cov(1,1);
Covrc(2,2) = Error_pose_cov(2,2);
Covrc(6,6) = Error_pose_cov(3,3);
     
poseMsg.Pose.Covariance = Covrc;
     
poseMsg.Header.Stamp = rostime("now");
send(myPub2, poseMsg)

%=======>START<=========

tic

while(currentTime(i)<160) % You can change the runtime.
    
  i= i+1;
  
  
  msgOdom = mySub3.LatestMessage;
  time_sec = msgOdom.Header.Stamp.Sec;
  time_Nsec = msgOdom.Header.Stamp.Nsec;
  
  odom_time(i) = double(time_sec) + (double(time_Nsec)/(10^9));
  odom_dst = odom_time(i)-odom_time(i-1);
  
   while (odom_dst==0)
        msgOdom = mySub3.LatestMessage;  
        time_sec =  msgOdom.Header.Stamp.Sec;
        time_Nsec =  msgOdom.Header.Stamp.Nsec;
  
        odom_time(i) = double(time_sec) + (double(time_Nsec)/(10^9));
        odom_dst = odom_time(i)-odom_time(i-1);
   end
  
  vn = msgOdom.Twist.Twist.Linear.X;
  wn = msgOdom.Twist.Twist.Angular.Z;
  
  msgScan = mySub2.LatestMessage;  
  time_sec = msgScan.Header.Stamp.Sec;
  time_Nsec = msgScan.Header.Stamp.Nsec;
  
  scan_time(i) = double(time_sec) + (double(time_Nsec)/(10^9));
  dst = scan_time(i)-scan_time(i-1);
  
     if (dst~=0)
       
      key = 1;
      msgScan = mySub2.LatestMessage;  
      Robot_ranges = (msgScan.Ranges)';
      N = length(Robot_ranges);
      angle_max =  msgScan.AngleMax;
      angle_min =  msgScan.AngleMin;
      inc = msgScan.AngleIncrement;
      Robot_angles = inc*(0:1:N-1) + bearing_noise*randn(1,N);
      
     end

      
  
    msgrobot = mySub.LatestMessage;
    
    q2 = msgrobot.Pose(3).Orientation.X;
    q3 = msgrobot.Pose(3).Orientation.Y;
    q4 = msgrobot.Pose(3).Orientation.Z;
    q1 = msgrobot.Pose(3).Orientation.W;

   robot_teta=atan2((2*(q2.*q3 + q1.*q4)),(2*(q1.*q1 + q2.*q2)-1));
   
    robot_x = msgrobot.Pose(3).Position.X - l*cos(robot_teta) + off_set_x;
    robot_y = msgrobot.Pose(3).Position.Y - l*sin(robot_teta) + off_set_y;
    

   
   [vn wn] = error_vel(vn,wn,alfa1,alfa2,alfa3,alfa4); % error velocity values are calculated according to the error parameters.
   
   
   currentTime(i) = toc;
   dto = currentTime(i)-currentTime(i-1);
   
   [odom_x odom_y odom_teta]= odom_pose(vn,wn,odom_x,odom_y,odom_teta,dto);  % odometry localization
   
   
    [mekf_x_t mekf_y_t mekf_teta_t Error_pose_cov_t] = MEKF_Predict(vn, wn, mekf_x, mekf_y, mekf_teta, alfa1, alfa2, alfa3, alfa4, Error_pose_cov, dto); % State Prediction Part
    
      mekf_x = mekf_x_t;
      mekf_y = mekf_y_t;
      mekf_teta = mekf_teta_t;
      Error_pose_cov = Error_pose_cov_t;  
      
     
      
      
        if (key==1)
        
           key = 0;  
          
          [Robot_ranges ranges angles lxa lya] = lidarkalman_olcum(mekf_x, mekf_y, mekf_teta, map_robot, range_intervals,dmax,inc,angle_min,angle_max,kal_noise,Robot_ranges,measurement_noise);
 
          Kalman_ranges = ranges';
          Kalman_angles = angles';
          klx = lxa;
          kly = lya;
    
          %Measurement Update Part
          [mekf_x_t mekf_y_t mekf_teta_t Error_pose_cov_t] = MEKF_Measurement_Update(mekf_x, mekf_y, mekf_teta, Error_pose_cov, Robot_ranges, Robot_angles, Kalman_ranges, Kalman_angles, klx, kly, Qt,Th);
   
 
          mekf_x = mekf_x_t;
          mekf_y = mekf_y_t;
          mekf_teta = mekf_teta_t;
          Error_pose_cov = Error_pose_cov_t;
         
        end
    
 
     poseMsg = rosmessage(myPub2);
     
     poseMsg.Pose.Pose.Position.X = mekf_x;
     poseMsg.Pose.Pose.Position.Y = mekf_y;
     poseMsg.Pose.Pose.Position.Z = 0;
     
     my_orientation = eul2quat([(mekf_teta) 0 0], "ZYX");
     
     poseMsg.Pose.Pose.Orientation.X = my_orientation(2);
     poseMsg.Pose.Pose.Orientation.Y = my_orientation(3);
     poseMsg.Pose.Pose.Orientation.Z = my_orientation(4);
     poseMsg.Pose.Pose.Orientation.W = my_orientation(1);
     
     Covrc(1,1) = Error_pose_cov(1,1);
     Covrc(2,2) = Error_pose_cov(2,2);
     Covrc(6,6) = Error_pose_cov(3,3);
     
     poseMsg.Pose.Covariance = Covrc;
  
     poseMsg.Header.Stamp = rostime("now");
     send(myPub2, poseMsg);
   
    
     ROBOTx(i-1) = robot_x;
     ROBOTy(i-1) = robot_y;
     ROBOTteta(i-1) = robot_teta;
     
     MEKF_x(i-1) = mekf_x;
     MEKF_y(i-1) = mekf_y;
     MEKF_teta(i-1) = mekf_teta;
     
     Odom_x(i-1) = odom_x;
     Odom_y(i-1) = odom_y;
     Odom_teta(i-1) = odom_teta;
     
end


   rbx = d_robot*cos(robot_teta) + robot_x;
   rby = d_robot*sin(robot_teta) + robot_y;
   
   Lx = d_robot*cos(mekf_teta) + mekf_x;
   Ly = d_robot*sin(mekf_teta) + mekf_y;
   
   odx = d_robot*cos(odom_teta) + odom_x;
   ody = d_robot*sin(odom_teta) + odom_y;
   

figure(2)
clf
show(map_robot)
hold on
plot(ROBOTx,ROBOTy,'g--','Linewidth',3)
hold on
plot(Odom_x,Odom_y,'b-.','Linewidth',1.5)
hold on
plot(MEKF_x,MEKF_y,'r-','Linewidth',1.5)
hold on
plot(robot_x,robot_y,'g.','markersize',50);
hold on
line([robot_x rbx],[robot_y rby],'Color','k','Linewidth',1.5);
hold on
plot(odom_x,odom_y,'b.','markersize',50);
hold on
line([odom_x odx],[odom_y ody],'Color','k','Linewidth',1.5);
hold on
plot(mekf_x,mekf_y,'r.','markersize',50);
hold on
line([mekf_x Lx],[mekf_y Ly],'Color','k','Linewidth',1.5);
hold on
xlabel("X [meters]")
ylabel("Y [meters")
axis([Nx1 Nx2 Ny1 Ny2])


MEKF_2D_POSE_ERROR = sqrt((ROBOTx-MEKF_x).^2 + (ROBOTy-MEKF_y).^2);
ODOM_2D_POSE_ERROR = sqrt((ROBOTx-Odom_x).^2 + (ROBOTy-Odom_y).^2);

N = length(MEKF_teta);

 for i = 1:N

  error_angle = (MEKF_teta(i)-ROBOTteta(i))*(180/pi);
    
   
     if(error_angle>=360)
        error_angle = (error_angle-360);
        
    end
    if(error_angle<=-360)
        error_angle = (error_angle+360);
        
    end
    
     if(error_angle>=360)
        error_angle = (error_angle-360);
        
    end
    if(error_angle<=-360)
        error_angle = (error_angle+360);
        
    end
    
     
    if(error_angle>=180)
        error_angle = (error_angle-360);
        
    end
    
    if(error_angle<=-180)
         error_angle = (360 +error_angle);
        
    end
    
    MEKF_Angle_Error(i) = abs(error_angle);
    
 end

 for i = 1:N

  error_angle = (Odom_teta(i)-ROBOTteta(i))*(180/pi);
    
   
     if(error_angle>=360)
        error_angle = (error_angle-360);
        
    end
    if(error_angle<=-360)
        error_angle = (error_angle+360);
        
    end
    
     if(error_angle>=360)
        error_angle = (error_angle-360);
        
    end
    if(error_angle<=-360)
        error_angle = (error_angle+360);
        
    end
    
     
    if(error_angle>=180)
        error_angle = (error_angle-360);
        
    end
    
    if(error_angle<=-180)
         error_angle = (360 +error_angle);
        
    end
    
    Odom_Angle_Error(i) = abs(error_angle);
    
 end




MEKF_mean_Angle_Error = mean(MEKF_Angle_Error)
MEKF_Angle_std = sqrt(var(MEKF_Angle_Error))

MEKF_2D_mean_Error = mean(MEKF_2D_POSE_ERROR)
MEKF_std = sqrt(var(MEKF_2D_POSE_ERROR))

Odom_mean_Angle_Error = mean(Odom_Angle_Error)
Odom_Angle_std = sqrt(var(Odom_Angle_Error))

Oodm_2D_mean_Error = mean(ODOM_2D_POSE_ERROR)
Oodom_std = sqrt(var(ODOM_2D_POSE_ERROR))


Mt = length(currentTime);

for i = 1:Mt-1
    dst(i) = currentTime(i+1)-currentTime(i);
end

SamplingTime = mean(dst)
SamplingFreq = 1/SamplingTime




figure(3)
plot(currentTime(1:end-1),ODOM_2D_POSE_ERROR,'b-.','Linewidth',1);
hold on
plot(currentTime(1:end-1),MEKF_2D_POSE_ERROR,'r','Linewidth',1);
hold on
xlabel("Time [s]")
ylabel("2D Positioning Errors [meters]")

figure(4)
plot(currentTime(1:end-1),Odom_Angle_Error,'b-.','Linewidth',1);
hold on
plot(currentTime(1:end-1),MEKF_Angle_Error,'r-','Linewidth',1);
hold on
xlabel("Time [s]")
ylabel("Absolute Angle Errors [degrees]")


%-----Functions--------

function [vn wn] = error_vel(wr, wl,alfa1,alfa2,alfa3,alfa4)

  v= wr;
  w = wl;
  
G1 = zeros(1,12);
G2 = zeros(1,12);

b1= sqrt(alfa1*abs(v^2) + alfa2*abs(w^2));
b2= sqrt(alfa3*abs(v^2) + alfa4*abs(w^2));


   for j = 1:12
    
       G1(j) = 1-2*rand();
       G2(j) = 1-2*rand();
     
   end



A1 = (b1/6)*(sum(G1));
A2 = (b2/6)*(sum(G2));
   
v = v + A1;
w = w + A2;

    if w == 0
       w = pi*(10^(-8));
    end
vn = v;
wn = w;

end



function [odom_x odom_y odom_teta]= odom_pose(vn,wn,odom_x,odom_y,odom_teta,dt)  % odometry localization

if wn == 0
    wn = pi*(10^(-8));
end

odom_pose=[odom_x odom_y odom_teta]';
odo_pose = odom_pose + [-(vn/wn)*sin(odom_teta)+(vn/wn)*(sin(odom_teta + wn*dt));(vn/wn)*cos(odom_teta)-(vn/wn)*(cos(odom_teta + wn*dt));wn*dt];
odom_x = odo_pose(1,1);
odom_y= odo_pose(2,1);
odom_teta=odo_pose(3,1);

odom_teta = mod(odom_teta,2*pi);

if odom_teta<-pi
    odom_teta = 2*pi + odom_teta;
end

if odom_teta>pi
    odom_teta = odom_teta-2*pi;
end

end


function [mekf_x_t mekf_y_t mekf_teta_t Error_pose_cov_t] = MEKF_Predict(vn, wn, mekfx_t_1, mekfy_t_1, mekf_teta_t_1, alfa1, alfa2, alfa3, alfa4,  Error_pose_cov_t_1, dt)

    if wn==0
       wn = pi*(10^(-7));
    end

Gt = [1 0 -(vn/wn)*cos(mekf_teta_t_1)+(vn/wn)*cos(mekf_teta_t_1 + wn*dt);0 1 -(vn/wn)*sin(mekf_teta_t_1)+(vn/wn)*sin(mekf_teta_t_1 + wn*dt);0 0 1];

Vt = [(-sin(mekf_teta_t_1)+sin(mekf_teta_t_1 + wn*dt))/wn ((vn/(wn^2))*(sin(mekf_teta_t_1)-sin(mekf_teta_t_1 + wn*dt))+((dt*vn)/wn)*(cos(mekf_teta_t_1 + wn*dt)));(cos(mekf_teta_t_1)-cos(mekf_teta_t_1 + wn*dt))/wn ((vn/(wn^2))*(-cos(mekf_teta_t_1)+cos(mekf_teta_t_1 + wn*dt))+((dt*vn)/wn)*(sin(mekf_teta_t_1 + wn*dt)));0 dt];

mekf_t_1=[mekfx_t_1 mekfy_t_1 mekf_teta_t_1]';
mekf_t = mekf_t_1 + [-(vn/wn)*sin(mekf_teta_t_1)+(vn/wn)*(sin(mekf_teta_t_1 + wn*dt));(vn/wn)*cos(mekf_teta_t_1)-(vn/wn)*(cos(mekf_teta_t_1 + wn*dt));wn*dt];


Et = [alfa1*(vn^2)+alfa2*(wn^2) 0;0 alfa3*(vn^2)+alfa4*(wn^2)];


Error_pose_cov_t = Gt*Error_pose_cov_t_1*(Gt')+ Vt*Et*(Vt');

mekf_x_t = mekf_t(1,1);
mekf_y_t= mekf_t(2,1);
mekf_teta_t = mekf_t(3,1);


end



function [Robot_ranges ranges angles lxa lya] = lidarkalman_olcum(posex, posey, pteta, map3, range_intervals,dmax,inc,angle_min,angle_max,kal_noise,Robot_ranges,measurement_noise)

 

 lidar = rangeSensor;
 lidar.Range=range_intervals;
 lidar.RangeNoise = kal_noise;
 
 lidar.HorizontalAngleResolution = inc;
 lidar.HorizontalAngle = [angle_min angle_max];
 
 truePose = [posex posey pteta];

[ranges, angles] = lidar(truePose, map3);



l = ranges';
N = length(l);

l_robot = Robot_ranges;

lxa = zeros(1,N);
lya = zeros(1,N);


a = angles';



for j=1:N
    
   
     
    if num2str(l(j))== "NaN"
        l(j) = 0;
     end
    
    
    if l(j)>=dmax
        l(j)=0;
    end
    
     
    if num2str(l_robot(j))== "Inf"
        l_robot(j) = 0;
    end
     
     
    if l_robot(j)>=dmax
        l_robot(j)=0;
    end
      
    
    lxa(j) = posex + l(j)*cos(a(j)+pteta);
    lya(j) = posey + l(j)*sin(a(j)+pteta);
    
    
end

Robot_ranges = l_robot + measurement_noise*randn(1,N);

ranges = l';
angles = a';

end


function [m_x m_y m_teta Et] = MEKF_Measurement_Update(pose_x, pose_y, pose_teta, Error_cov_t, Robot_ranges, Robot_angles, Kalman_ranges, Kalman_angles, mlx, mly,Qt,Th)


I = [1 0 0;0 1 0;0 0 1];

M = 360; % Total number of measurements taken with LiDAR

Nl = 180; % Total number of VDPLs assigned to the map

nk = M/Nl;

Nm = 90; % The number of real measurements tasked with extracting the most appropriate virtual dynamic point landmarks (VDPL) assigned to the map.

nm = M/Nm;

n=4;


mu_ekf = [pose_x;pose_y;pose_teta];
Kw_ekf(1:2*n+1)=0;


kz=0;
   for il = 1:Nm

       kz = kz+nm;
    
       Bearing_ekf = Robot_angles(kz);


       if Bearing_ekf>=pi
          Bearing_ekf = Bearing_ekf-2*pi;
       end
 
       if Bearing_ekf<=-pi
          Bearing_ekf = Bearing_ekf+2*pi;
       end
 
     Zr = Robot_ranges(kz);

     zr =[Zr;0];

     w_max = 0;
     inx = 0;

     for l = 1:2*n+1
       
         kx = il*(nm) + (l-(n+1))*(nk);
    
        if(kx<=0)
        
            kx = M + kx ;
        end
    
        if(kx>M)
           kx = kx-M;
        end
    
    
        q = (mlx(kx)-pose_x)^2 + (mly(kx)-pose_y)^2; % mlx, the x position of the extracted point landmark on the map, mly, the y position of the extracted point landmark on the map, sqrt(q) = predicted measurement.

        Bteta_ekf = Kalman_angles(kx);

 
      if Bteta_ekf>=pi
          Bteta_ekf = Bteta_ekf-2*pi;
      end

      if Bteta_ekf <=-pi
          Bteta_ekf = Bteta_ekf+2*pi;
     end


     pz = Bearing_ekf-Bteta_ekf;

     zn =[sqrt(q);-pz];

     if (sqrt(q)>0)&&(Zr>0)

        Ht = [(-(mlx(kx)-pose_x)/sqrt(q)) (-(mly(kx)-pose_y)/sqrt(q)) 0;((mly(kx)-pose_y)/q) (-(mlx(kx)-pose_x)/q) -1];
        Hmt = [(-(mlx(kx)-pose_x)/sqrt(q)) (-(mly(kx)-pose_y)/sqrt(q));(-(mly(kx)-pose_y)/q) ((mlx(kx)-pose_x)/q)];


        St =  Ht*(Error_cov_t)*(Ht') + Qt + Hmt*[0.08^2 0;0 0.10^2]*(Hmt');



        Bs = isnan(St);

        bs = Bs(1,1)+Bs(1,2)+Bs(2,1)+Bs(2,2);

        if bs>=1
   

        St = (10^(-3))*eye(2,2);


       end

% This method is used to prevent the St matrix from being included in the calculation when it gets too close to the singular.
      D = eig(St);
      max_D = max(D);
      min_D = min(D);
      Rcond = min_D/max_D;

      if (abs(Rcond)>10^(-6))
    
    
         Kw_ekf(l) = (1/(sqrt(2*pi*det(St))))*exp(-0.5*((zr-zn)')*(inv(St))*(zr-zn));

        if Kw_ekf(l)>w_max
           w_max = Kw_ekf(l);
           inx = l;
           H_max = Ht;
           S_max = St;
           z_max = zn;
     
        end
 
      end


     end

     end
   
  
  if inx>0
   
   if(Kw_ekf(inx)>Th)
         
   Ht = H_max;
   St = S_max;
   zn = z_max;   
   
 Kt = (Error_cov_t)*(Ht')*(inv(St));

 mu_ekf = mu_ekf + Kt*(zr-zn);
 Error_cov_t = (I-(Kt*Ht))*Error_cov_t;
 
 pose_x = mu_ekf(1,1);
 pose_y = mu_ekf(2,1);
 
 pose_teta = mu_ekf(3,1);
    
   end
  end

    Kw_ekf = zeros(1,2*n+1);
   end


Et = Error_cov_t;
m_x = mu_ekf(1,1);
m_y = mu_ekf(2,1);
m_teta = mu_ekf(3,1);

end








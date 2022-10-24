clear all
clc
close all
load viconOdom
load rtabOdom

mat_idx = @(M,i,j) M(i,j);

%% Unpack data
% Script to generate a homogeneous transform to map the map frame into the
% vicon frame.

l1 = length(viconOdom.time);
l2 = length(rtabOdom.time);
pairIdx = 1;
for i=1:length(rtabOdom.time)
   id = find(viconOdom.time < rtabOdom.time(i) + 0.009e9 & viconOdom.time > rtabOdom.time(i) - 0.009e9);
   if ~isempty(id)
       
       pairs(pairIdx,:) = [i, id(1)]; % attempt at time syncing these.
                           % Really need to re-write the ROS node to only
                           % publish odometry when the rtabmap odom is
                           % published.   
      pairIdx = pairIdx + 1;
   end
end

% Raw data:
cameraVec = [rtabOdom.positionx(pairs(:,1)).';
             rtabOdom.positiony(pairs(:,1)).';
             rtabOdom.positionz(pairs(:,1)).'];
         
viconVec = [viconOdom.positionx(pairs(:,2)).';
            viconOdom.positiony(pairs(:,2)).';
            viconOdom.positionz(pairs(:,2)).'] .* 1e-3; % Convert to m
        
Rvn = quat2rotm([viconOdom.orientationw(pairs(:,2)),...
                viconOdom.orientationx(pairs(:,2)),...
                viconOdom.orientationy(pairs(:,2)),...
                viconOdom.orientationz(pairs(:,2))]); 
                     
Rcm = quat2rotm([rtabOdom.orientationw(pairs(:,1)),...
                rtabOdom.orientationx(pairs(:,1)),...
                rtabOdom.orientationy(pairs(:,1)),...
                rtabOdom.orientationz(pairs(:,1))]);
            
Rcm_eul_zyx = quat2eul([rtabOdom.orientationw(pairs(:,1)),...
                rtabOdom.orientationx(pairs(:,1)),...
                rtabOdom.orientationy(pairs(:,1)),...
                rtabOdom.orientationz(pairs(:,1))]);

% Shift datasets so that the initial points are at the origin 
viconVec_shift = viconVec - viconVec(:,1);
cameraVec_shift = cameraVec - cameraVec(:,1);

% scatter3(viconVec_shift(1,:),viconVec_shift(2,:),viconVec_shift(3,:))
% hold on
% scatter3(cameraVec(1,:),cameraVec(2,:),cameraVec(3,:))

%% Attempt to align paths
% Crop section of recording
cmp_start = 100;
cmpLength = 300;

% Define initial guess transformation of camera data. First components as z-y-x Euler
% angles, last components are translations
trans0 = [0.3373;2.8953;-2.4776;0;0;0];

% Define sum of squares cost between paths
pos_dif = @(trans) (rotatezyx(trans(1:3))*cameraVec_shift(:,cmp_start:cmpLength) + trans(4:6) - viconVec_shift(:,cmp_start:cmpLength)).';
cost = @(trans) sum(sum(pos_dif(trans).*pos_dif(trans)));

% Optimise transformation
trans_opt = fminsearch(cost,trans0);

% Apply optimal transformation to shifted camera deta
cameraVec_rot = rotatezyx(trans_opt(1:3))*cameraVec_shift + trans_opt(4:6);

% Display results
scatter3(viconVec_shift(1,cmp_start:cmpLength),viconVec_shift(2,cmp_start:cmpLength),viconVec_shift(3,cmp_start:cmpLength))
hold on
axis equal
scatter3(cameraVec_rot(1,cmp_start:cmpLength),cameraVec_rot(2,cmp_start:cmpLength),cameraVec_rot(3,cmp_start:cmpLength))


%% Attempt to align odometry
% Compute the odometry updates for each step for shifted data
viconVec_shift_odom = viconVec_shift(:,2:end) - viconVec_shift(:,1:end-1);
cameraVec_shift_odom = cameraVec_shift(:,2:end) - cameraVec_shift(:,1:end-1);

% Crop section of recording
cmp_start = 1;
cmpLength = length(viconVec_shift_odom);

% Define initial guess transformation of camera data. No translation
% component in this case
ang0 = [0.3373;2.8953;-2.4776];

% Define sum of squares cost between paths
odom_dif = @(ang) (rotatezyx(ang)*cameraVec_shift_odom(:,cmp_start:cmpLength) - viconVec_shift_odom(:,cmp_start:cmpLength)).';
cost = @(ang) sum(sum(odom_dif(ang).*odom_dif(ang)));

% Optimise transformation
ang_opt = fminsearch(cost,ang0);

% Compute odom errors in each coordinate
odom_dif_opt = odom_dif(ang_opt);
figure(1)
subplot(3,1,1)
histogram(odom_dif_opt(:,1))
xlabel('x odometry')
subplot(3,1,2)
histogram(odom_dif_opt(:,2))
xlabel('y odometry')
subplot(3,1,3)
histogram(odom_dif_opt(:,3))
xlabel('z odometry')

%% Functions

function R = rotatezyx(ang)

% X
c = cos(ang(1));
s = sin(ang(1));
Rx = [1  0   0;
      0  c  -s;
      0  s   c];

% Y
c = cos(ang(2));
s = sin(ang(2));
Ry = [ c  0   s;
       0  1   0;
      -s  0   c];

% Z
c = cos(ang(3));
s = sin(ang(3));
Rz = [ c  -s  0;
       s   c  0;
       0   0  1];

R = Rz*Ry*Rx;   

end



        

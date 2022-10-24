% IMU Calibration script 
% Was used to compute the rotation matrix that maps the LiDAR frame into 
% the IMU frame.


close all
% clear all
clc
set(groot,'DefaultAxesTickLabelInterpreter','latex');
% load IMUodom.mat
% load ICPodom.mat

load ICP_odom_better.mat
load IMUodombetter.mat

IMUodom = IMUodombetter;
ICPodom = ICPodombetter;

% Attempt at calibrating the IMU extrinsic parameters
time_imu = IMUodom.fieldheaderstamp;
imu_qx = IMUodom.fieldorientationx;
imu_qy = IMUodom.fieldorientationy;
imu_qz = IMUodom.fieldorientationz;
imu_qw = IMUodom.fieldorientationw;

imu_eul = quat2eul([imu_qw, imu_qx, imu_qy, imu_qz], "ZYX");

time_icp = ICPodom.fieldheaderstamp;
icp_qx = ICPodom.fieldposeposeorientationx;
icp_qy = ICPodom.fieldposeposeorientationy;
icp_qz = ICPodom.fieldposeposeorientationz;
icp_qw = ICPodom.fieldposeposeorientationw;

icp_eul = quat2eul([icp_qw, icp_qx, icp_qy, icp_qz], "ZYX");

% Need to align data in time.. Hard because of the massive discrepency in
% sample rates.
hz_icp = length(time_icp)/((time_icp(end) - time_icp(1))*1e-9); % About 10Hz
hz_imu = length(time_imu)/((time_icp(end) - time_icp(1))*1e-9); % About 100Hz

% Just take every 10th sample from IMU and pair it with IMU:
count= 0;
% for i=1:length(imu_eul)
%     if mod(i,10) == 0
%        count = count + 1;
%        sampled_IMU(count + 10,:) = imu_eul(i,:); 
%     end
% end

sampled_IMU = imu_eul;


modified(:,3) =  sampled_IMU(:,2);
modified(:,2) =  sampled_IMU(:,3);
modified(:,1) = -sampled_IMU(:,1);

figure
subplot(3,1,1)
plot(sampled_IMU(:,3))
hold on
plot(icp_eul(11:end,3))
% plot(modified(11:end,1))
legend("IMU", "ICP")

subplot(3,1,2) % Paired ICP z axis with IMU y axis
plot(sampled_IMU(:,2))
hold on
% plot(icp_eul(11:end,2))
plot(modified(11:end,2))
legend("IMU", "ICP")

subplot(3,1,3) % Paired ICP y axis with IMU z axis
plot(sampled_IMU(:,1))
hold on
plot(icp_eul(11:end,1))
% plot(modified(11:end,3) - 2.64)
legend("IMU", "ICP")

figure
subplot(3,1,1)
% plot(sampled_IMU(:,1))
hold on
plot(icp_eul(11:end,3))
plot(modified(11:end,3))
legend("ICP", "Modified IMU")

subplot(3,1,2) % Paired ICP z axis with IMU y axis
% plot(sampled_IMU(:,2))
hold on
plot(icp_eul(11:end,2))
plot(modified(11:end,2))
legend("ICP", "Modified IMU")

subplot(3,1,3) % Paired ICP y axis with IMU z axis
% plot(sampled_IMU(:,3))
hold on
plot(icp_eul(11:end,1))
plot(modified(11:end,1) - 2.64)
legend("ICP", "Modified IMU")

%% Transform IMU into ICP:
% rot_y(180)*rot_z(-90) Change this to be zyx

%% Look at angular velocities
imu_vx = IMUodom.fieldangular_velocityx;
imu_vy = IMUodom.fieldangular_velocityy;
imu_vz = IMUodom.fieldangular_velocityz;

imu_v = [imu_vx, imu_vy, imu_vz];

icp_vx = ICPodom.fieldtwisttwistangularx;
icp_vy = ICPodom.fieldtwisttwistangulary;
icp_vz = ICPodom.fieldtwisttwistangularz;

icp_v = [icp_vx, icp_vy, icp_vz];

% Just take every 10th sample from IMU and pair it with IMU:
count= 0;
% for i=1:length(imu_eul)
%     if mod(i,10) == 0
%        count = count + 1;
%        sampled_IMU_v(count + 10,:) = imu_v(i,:); 
%     end
% end

%% Optimise a transform between them:
velocity_diff = @(ang)  (rotatezyx(ang)*icp_v(11:end,:).').' - imu_v(11:end,:);
cost = @(ang) sum(sum(velocity_diff(ang).*velocity_diff(ang)));

ang0 = [-pi/2 -pi 0];
ang0 = [0 0 0];
ang_opt = fminsearch(cost, ang0)
R = rotatezyx(ang_opt);
%%
sampled_IMU_v = imu_v;

% modifiedv(:,1) =  sampled_IMU_v(:,2);
% modifiedv(:,2) =  sampled_IMU_v(:,1);
% modifiedv(:,3) = -sampled_IMU_v(:,3);

modifiedv = (R.'*sampled_IMU_v.').'; % Map imu data into ICP fame

figure
subplot(3,1,1)
title("\textbf{Angular Velocity - Rotation Adjusted}", Interpreter="latex")
% plot(sampled_IMU_v(:,1))
hold on
plot(icp_v(11:end,1), LineWidth=1.5)
plot(modifiedv(11:end,1), LineWidth=1.5)
legend("ICP", "IMU", Interpreter="latex")
xlabel("Sample number", Interpreter="latex")
ylabel("Roll rate (rad/s)", Interpreter="latex")
ax = gca;
ax.FontSize = 18;

subplot(3,1,2) % Paired ICP z axis with IMU y axis
% plot(sampled_IMU_v(:,2))
hold on
plot(icp_v(11:end,2), LineWidth=1.5)
plot(modifiedv(11:end,2), LineWidth=1.5)
legend("ICP", "IMU", Interpreter="latex")
xlabel("Sample number", Interpreter="latex")
ylabel("Pitch rate (rad/s)", Interpreter="latex")
ax = gca;
ax.FontSize = 18;

subplot(3,1,3) % Paired ICP y axis with IMU z axis
% plot(sampled_IMU_v(:,3))
hold on
plot(icp_v(11:end,3), LineWidth=1.5)
plot(modifiedv(11:end,3), LineWidth=1.5)
legend("ICP", "IMU", Interpreter="latex")
xlabel("Sample number", Interpreter="latex")
ylabel("Yaw rate (rad/s)", Interpreter="latex")
ax = gca;
ax.FontSize = 18;

figure
subplot(3,1,1)
title("\textbf{Angular Velocity - Raw Data}", Interpreter="latex")
hold on
plot(icp_v(11:end,1), LineWidth=1.5)
plot(sampled_IMU_v(11:end,1), LineWidth=1.5)
% plot(modifiedv(11:end,1))
legend("ICP", "IMU", Interpreter="latex")
xlabel("Sample number", Interpreter="latex")
ylabel("Roll rate (rad/s)", Interpreter="latex")
ax = gca;
ax.FontSize = 18;

subplot(3,1,2) % Paired ICP z axis with IMU y axis
plot(icp_v(11:end,2), LineWidth=1.5)
hold on
plot(sampled_IMU_v(11:end,2), LineWidth=1.5)
% plot(modifiedv(11:end,1))
legend("ICP", "IMU", Interpreter="latex")
xlabel("Sample number", Interpreter="latex")
ylabel("Pitch rate (rad/s)", Interpreter="latex")
ax = gca;
ax.FontSize = 18;

subplot(3,1,3) % Paired ICP y axis with IMU z axis
plot(icp_v(11:end,3), LineWidth=1.5)
hold on
plot(sampled_IMU_v(11:end,3), LineWidth=1.5)
% plot(modifiedv(11:end,1))
legend("ICP", "IMU", Interpreter="latex")
xlabel("Sample number", Interpreter="latex")
ylabel("Yaw rate (rad/s)", Interpreter="latex")
ax = gca;
ax.FontSize = 18;



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
close all
clear all
clc
set(groot,'DefaultAxesTickLabelInterpreter','latex');
%% Load data
FRAME_TO_FRAME = false; % Plot either Frame to Frame data, or Frame To Map

if FRAME_TO_FRAME == true
    load ekfOdom.mat
    load viconOdom.mat
    load icpOdom.mat
else
    load vicon.mat; load odomFTM.mat; load odomekfftm.mat
    viconOdom = vicon;
    icpOdom = odomFTM;
    ekfOdom = odomekfftm;
end

%% Try align EKF and Vicon data

for i=1:length(ekfOdom.time)

    timeDiff = abs(ekfOdom.time(i) - viconOdom.time);
    [~,idx] = min(timeDiff);

    rICP(i,:) = [ekfOdom.fieldposeposepositionx(i),...
                 ekfOdom.fieldposeposepositiony(i),...
                 ekfOdom.fieldposeposepositionz(i)];

    rVic(i,:) = [viconOdom.fieldposeposepositionx(idx),...
                 viconOdom.fieldposeposepositiony(idx),...
                 viconOdom.fieldposeposepositionz(idx)].*1e-3;

    timeDiff = abs(ekfOdom.time(i) - icpOdom.time);
    [~,idx] = min(timeDiff);

    rICP2(i,:) = [icpOdom.fieldposeposepositionx(idx),...
                 icpOdom.fieldposeposepositiony(idx),...
                 icpOdom.fieldposeposepositionz(idx)];

    t(i) = viconOdom.time(idx);

end
t = (t - t(1)).*1e-8;

% Adjust initial points
rVic = rVic - rVic(1,:);
rICP = rICP - rICP(1,:);

%% Plot un-aligned data
figure
subplot(3,1,1)
plot(t,rICP(:,1))
hold on
plot(t,rVic(:,1))
legend("ICP","Vicon")

subplot(3,1,2)
plot(t, rICP(:,2))
hold on
plot(t, rVic(:,2))
legend("ICP","Vicon")

subplot(3,1,3)
plot(t, rICP(:,3))
hold on
plot(t, rVic(:,3))
legend("ICP","Vicon")

%% Try align data:

% Create odometry stream
vicon_odom = rVic(2:end,:) - rVic(1:end-1,:);
ICP_odom = rICP(2:end,:) - rICP(1:end-1,:);

% plot(t(1:end-1), vicon_odom(:,1))

crpstart = 1000;
crpend = length(t)-1;
crpend = 2500;

% path_dif = @(trans) (rotatezyx(trans(4:6))*vicon_odom(crpstart:crpend,:).' + trans(1:3).' - ICP_odom(crpstart:crpend,:).');
% cost = @(trans) sum(sum(path_dif(trans).*path_dif(trans)));
% trans0 = [0 0 0 0 0 0];
% trans_opt = fminsearch(cost,trans0)


odom_dif = @(ang) (rotatezyx(ang)*ICP_odom(crpstart:crpend,:).' - vicon_odom(crpstart:crpend,:).');
cost = @(ang) sum(sum(odom_dif(ang).*odom_dif(ang)));

% Optimise transformation
% ang0 = [ -0.7663    4.1050   -1.6408];
ang0 = [0 0 0];
ang_opt = fminsearch(cost,ang0)





% Translation:
pos_dif = @(trans) (rotatezyx(ang_opt)*rICP(crpstart:crpend, :).' + trans.' - rVic(crpstart:crpend, :).').';
cost = @(trans) sum(sum(pos_dif(trans).*pos_dif(trans)));

tran0 = [0 0 0];
tran_opt = fminsearch(cost,tran0)

% Apply rotation, see if it worked
adjusted = (rotatezyx(ang_opt)*rVic.').' + 0*tran_opt;

%% Plot rotation adjusted data:
figure
subplot(3,1,1)
plot(t,adjusted(:,1), 'b', 'LineWidth',1.5)
hold on
title("Position vs Time")
plot(t,rICP(:,1), 'r', 'LineWidth',1.5)
plot(t, rICP2(:,1), 'g', 'LineWidth',1.5)
legend("Vicon","EKF", "ICP")
xlabel("time (s)", Interpreter="latex")
ylabel("$$x$$ (m)", Interpreter="latex")
ax = gca;
ax.FontSize = 18;

subplot(3,1,2)
plot(t, adjusted(:,2), 'b', 'LineWidth',1.5)
hold on
plot(t, rICP(:,2), 'r', 'LineWidth',1.5)
plot(t, rICP2(:,2), 'g', 'LineWidth',1.5)
legend("Vicon","EKF", "ICP")
xlabel("time (s)", Interpreter="latex")
ylabel("$$y$$ (m)", Interpreter="latex")
ax = gca;
ax.FontSize = 18;

subplot(3,1,3)
plot(t, adjusted(:,3), 'b', 'LineWidth',1.5)
hold on
plot(t, rICP(:,3), 'r', 'LineWidth',1.5)
plot(t, rICP2(:,3), 'g', 'LineWidth',1.5)
legend("Vicon","EKF", "ICP")
xlabel("time (s)", Interpreter="latex")
ylabel("$$z$$ (m)", Interpreter="latex")
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


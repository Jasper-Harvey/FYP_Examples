load icpOdom2.mat 
load ekfOdom2.mat 
load icpOdom4.mat
load viconOdom2.mat

ekfOdom = ekfOdom2;
icpOdom = icpOdom2;
viconOdom = viconOdom2;

icp2 = icpOdom4;


%% The "2" series of data is from a mapping session running the EKF. 10Hz lidar data. EKF at like 50Hz
% The "3" series of data is from the mapping session without the kalman
% filter running.
% The "4" series is ICP running witout kalman filter and also with frame to
% frame ICP

ekfTime = ekfOdom.time;
icpTime = icpOdom.time;
viTime = viconOdom.time;
icp2Time = icpOdom4.time;


ekfx = ekfOdom.fieldposeposepositionx;
ekfy = ekfOdom.fieldposeposepositiony;
ekfz = ekfOdom.fieldposeposepositionz;

icpx = icpOdom.fieldposeposepositionx;
icpy = icpOdom.fieldposeposepositiony;
icpz = icpOdom.fieldposeposepositionz;

vix = viconOdom.fieldposeposepositionx.*1e-3;
viy = viconOdom.fieldposeposepositiony.*1e-3;
viz = viconOdom.fieldposeposepositionz.*1e-3;

icp2x = icpOdom4.fieldposeposepositionx;
icp2y = icpOdom4.fieldposeposepositiony;
icp2z = icpOdom4.fieldposeposepositionz;

figure
subplot(3,1,1)
plot((ekfTime - ekfTime(1))*1e-9, ekfx); % EKF data
hold on
% plot(icpTime, icpx);
plot((icp2Time - icp2Time(1))*1e-9, icp2x); % Bad icp data
% plot((viTime - viTime(1))*1e-9, -vix + vix(1));
legend("EKF", "ICP","Vicon")
subplot(3,1,2)
plot((ekfTime - ekfTime(1))*1e-9, ekfy); % EKF data
hold on
% plot(icpTime, icpy);
plot((icp2Time - icp2Time(1))*1e-9, icp2y); % Bad icp data
% plot((viTime - viTime(1))*1e-9, -viy + viy(1));
legend("EKF", "ICP","Vicon")
subplot(3,1,3)
plot((ekfTime - ekfTime(1))*1e-9, ekfz);
hold on
% plot(icpTime, icpz);
plot((icp2Time - icp2Time(1))*1e-9, icp2z); % Bad icp data
% plot((viTime - viTime(1))*1e-9, -viz + viz(1));
legend("EKF", "ICP","Vicon")













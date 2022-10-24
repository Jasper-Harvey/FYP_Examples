clear all
close all
clc

pc = pcread("ES313_RGB_LIDAR_decimation3.ply");
pc1 = pcdownsample(pc, 'gridAverage', 0.1);
p1 = pc1.Location;
p2 = (eul2rotm([pi/10,0,pi/10])*pc1.Location.').' + [2,1,1];

% p3 = p2(p2(:,1) > 0,:);
p3 = p2;

tic
T = ICP_SVD_SE3(p1,p3, 200)
toc

x = T(1,4);
y = T(2,4);
z = T(3,4);
R = T(1:3,1:3);

% difference = [x;y;rad2deg(best_pose(3))];

transformedPts = R*p2.' + [x;y;z];

%%
figure
plot3(transformedPts(1,:), transformedPts(2,:), transformedPts(3,:), "*")
title("Aligned Point Clouds")
hold on
plot3(p1(:,1), p1(:,2), p1(:,3), ".")
axis("equal")
xlabel("x (m)")
ylabel("y (m)")
zlabel("z (m)")
xlim([-10 10])
ylim([-5 5])
zlim([-5 5])
ax = gca;
ax.FontSize = 18;

figure
plot3(p3(:,1), p3(:,2), p3(:,3), ".")
title("Misaligned Point Clouds")
hold on
plot3(p1(:,1), p1(:,2), p1(:,3), ".")
axis("equal")
xlim([-10 10])
ylim([-5 5])
zlim([-5 5])
xlabel("x (m)")
ylabel("y (m)")
zlabel("z (m)")
ax = gca;
ax.FontSize = 18;

% pcregistericp()





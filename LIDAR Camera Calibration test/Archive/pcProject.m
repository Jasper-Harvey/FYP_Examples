close all
clc
clear all

% load test2/newImages/goodTform.mat
load tran_opt.mat
load test2/cameraParams.mat

% t = tform2.Translation
% R = tform2.Rotation.';

% eulAng_zyx = rad2deg(rotm2eul(R))


% p = pcread("./newImages/1659659373814043.pcd");
% image = imread("./newImages/1659659373814043.png");
p = pcread("test2/newImages/1663630817293146.pcd");
image = imread("test2/newImages/1663630817293146.png");


% J2 = undistortImage(image,cameraParams);
% figure(10)
% imshow(J2)
% hold on
% plot(imPts(:,1),imPts(:,2),'.r','MarkerSize',10)

t = tran_opt(1:3)
% t(1) = t(1) - 0.02;
% t(2) = t(2) + 0.02;
R = rotatezyx(tran_opt(4:6))

pcTf = pointCloud((R*p.Location.' + t.').');
pcTf = pointCloud(pcTf.Location(vecnorm(pcTf.Location.') < 10,:));
pOut = fuseCameraToLidar(image,pcTf, cameraParams.Intrinsics);
pcshow(pOut)







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
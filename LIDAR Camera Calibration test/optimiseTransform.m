% Tries to find the optimal transform that maps checkerboard corners
% extracted from LiDAR point clouds into corners extracted from images.

% Pretty hacked together and messy..

clear all
close all
set(groot,'DefaultAxesTickLabelInterpreter','latex');


% load imgCnrs_goodfit
% load lidarCnrs_goodfit
load imgCnrs
load lidarCnrs

for i=1:size(imgCnrs,1)-1
   imgP(:,:,i) = squeeze(imgCnrs(i,:,:)).';
   lidarP(:,:,i) = squeeze(lidarCnrs(i,:,:)).';
end

%% Should be aiming for somewhere in the region of 150mm sideways. 0 up and down. 
        
%% Optimise for Rotation:

idz = 1:length(lidarP);
% idz = 1:5
% idz = [2 3 5 6 7 8 9 11 12 13]
% idz = randsample([1:length(lidarP)],7)
% idz = [5 6 7 1 2 13]
costF = @(ang) costRotation(ang, imgP(:,:,idz), lidarP(:,:,idz));
ang0 = [0,0,0];
% Optimise transformation
opts = optimset("MaxFunEvals", 1e30);
ang_opt = fminsearch(costF,ang0,opts)

%% Have a crack at translation:

costF = @(tran) costTran(tran, imgP(:,:,idz), lidarP(:,:,idz));
tran0 = [-0.4,0,-0.1, ang_opt];
% Optimise transformation
tran_opt = fminsearch(costF,tran0, opts)


save("tran_opt", "tran_opt")
%% Plot
R = rotatezyx(tran_opt(4:6));
% R = rotatezyx([pi/2 pi pi/2])
for j=1:length(lidarP)
    
    P1 = squeeze(lidarP(:,:,j));
    P2 = squeeze(imgP(:,:,j)).';
    tfPts = (rotatezyx(tran_opt(4:6))*P1 + tran_opt(1:3).').';
%     tfPts = (rotatezyx([1.2437,-1.5812,0.3122])*P1 + 0*tran_opt(1:3).').';
    
    % Histograms
    d(j,:) = mean(P2) - mean(tfPts);

    % Planes:
    fill3(P2(:,1),P2(:,2), P2(:,3),"c") % Image planes un-changed
    hold on
    plot3(P2(1,1),P2(1,2), P2(1,3), '.r', "MarkerSize", 20)
    plot3(P2(2,1),P2(2,2), P2(2,3), '.g', "MarkerSize", 20)
    plot3(P2(3,1),P2(3,2), P2(3,3), '.b', "MarkerSize", 20)
    plot3(P2(4,1),P2(4,2), P2(4,3), '.c', "MarkerSize", 20)
    fill3(tfPts(:,1), tfPts(:,2), tfPts(:,3),"r") % Lidar planes transformed
    plot3(tfPts(1,1),tfPts(1,2), tfPts(1,3), '.r', "MarkerSize", 20)
    plot3(tfPts(2,1),tfPts(2,2), tfPts(2,3), '.g', "MarkerSize", 20)
    plot3(tfPts(3,1),tfPts(3,2), tfPts(3,3), '.b', "MarkerSize", 20)
    plot3(tfPts(4,1),tfPts(4,2), tfPts(4,3), '.c', "MarkerSize", 20)
%     fill3(P1(1,:),P1(2,:), P1(3,:),"g") % LIDAR planes un-changed
    daspect([1 1 1])
end

% Rinv = rotatezyx(-tran_opt(4:6));
p0x = [0.2, 0, 0]; p0y = [0, 0.2, 0]; p0z = [0, 0, 0.2];
lidarOrigin = tran_opt(1:3).';
line1 = lidarOrigin + R*[0.2; 0; 0];
line2 = lidarOrigin + R*[0; 0.2; 0];
line3 = lidarOrigin + R*[0; 0; 0.2];

% Camera frame marker:
line([0;p0x(1)], [0;p0x(2)], [0;p0x(3)], "Color", "blue", "LineWidth", 2)
line([0;p0y(1)], [0;p0y(2)], [0;p0y(3)], "Color", "red", "LineWidth", 2)
line([0;p0z(1)], [0;p0z(2)], [0;p0z(3)], "Color", "green", "LineWidth", 2)
% LIDAR frame marker:
line([lidarOrigin(1);line1(1)], [lidarOrigin(2);line1(2)], [lidarOrigin(3);line1(3)], "Color", "blue", "LineWidth", 2)
line([lidarOrigin(1);line2(1)], [lidarOrigin(2);line2(2)], [lidarOrigin(3);line2(3)], "Color", "red", "LineWidth", 2)
line([lidarOrigin(1);line3(1)], [lidarOrigin(2);line3(2)], [lidarOrigin(3);line3(3)], "Color", "green", "LineWidth", 2)
xlabel("x")
ylabel("y")
zlabel("z")
daspect([1 1 1])


figure(2)
subplot(3,1,1)
histogram(d(:,1).*1e3, 5)
ylabel("x")
subplot(3,1,2)
histogram(d(:,2).*1e3, 5)
ylabel("y")
subplot(3,1,3)
histogram(d(:,3).*1e3, 5)
ylabel("z")


figure(3)
subplot(3,1,1)
bar(d(:,1).*1e3)
title("Error between point cloud and image plane centers") 
ylabel("$x$ (mm)", Interpreter="latex")
ax = gca; ax.FontSize = 18;
subplot(3,1,2)
bar(d(:,2).*1e3)
ylabel("$y$ (mm)", Interpreter="latex")
ax = gca; ax.FontSize = 18;
subplot(3,1,3)
bar(d(:,3).*1e3)
ylabel("$z$ (mm)", Interpreter="latex")
xlabel("Point cloud, image pair")
ax = gca; ax.FontSize = 18;


%% Project on to image
load test2/cameraParams.mat
imagePath = fullfile("test2/newImages");
imgs= dir(fullfile('test2/newImages', '*.png'));
pcds = dir(fullfile('test2/newImages', '*.pcd'));

image = imread(imagePath + "\" + imgs(1).name);
pointCloudin = pcread(imagePath + "\" + pcds(1).name);
pointCloudin = pointCloud(pointCloudin.Location(vecnorm(pointCloudin.Location.') < 10, :));
% transformedPts = rotatezyx(tran_opt(4:6))*pointCloud.Location.' + tran_opt(1:3).';
R = rotatezyx(tran_opt(4:6));
% R = rotatezyx(ang_opt)
% R = rotatezyx([pi/2, -pi/2, 0]);
t = tran_opt(1:3);
T = rigid3d(R,t);
Ti = invert(T);
% imPts = projectLidarPointsOnImage(pointCloud.Location,cameraParams,Ti);
imPts = projectLidarPointsOnImage( (R*pointCloudin.Location.' + t.').' ,cameraParams,rigid3d());

J2 = undistortImage(image,cameraParams);
figure(10)
imshow(J2)
hold on
plot(imPts(:,1),imPts(:,2),'.r','MarkerSize',10)

pcTf = pointCloud((R*pointCloudin.Location.' + t.').');
pOut = fuseCameraToLidar(image,pcTf, cameraParams.Intrinsics);

%% Funcs:
function c = costRotation(ang, imgP, lidarP)
    for i=1:size(imgP,3)
       e = rotatezyx(ang)*squeeze(lidarP(:,:,i)) - squeeze(imgP(:,:,i)); 
       e2(i) = sum(sum(e.*e));
    end
    c = sum(e2);
end

function c = costTran(tran, imgP, lidarP)
    for i=1:size(imgP,3)
       e = rotatezyx(tran(4:6))*squeeze(lidarP(:,:,i)) + tran(1:3).' - squeeze(imgP(:,:,i)) ; 
       e2(i) = sum(sum(e.*e));
    end
    c = sum(e2);
end


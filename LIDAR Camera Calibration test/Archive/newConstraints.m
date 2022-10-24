clear all
close all

load imgConstraints.mat
load lidarConstraints.mat
load imgCnrs9
load lidarCnrs9
for i=1:size(lidarCnrs,1)-1
   imgP(:,:,i) = squeeze(imgCnrs(i,:,:)).';
   lidarP(:,:,i) = squeeze(lidarCnrs(i,:,:)).';
end

%% Should be aiming for somewhere in the region of 150mm
        
%% Optimise for Rotation:

idz = 1:10
% idz = [5 6 7 1 2 13]
costF = @(ang) costRotation(ang, imgConstraints, lidarConstraints);
ang0 = [0, 0, 0.0];
% Optimise transformation
ang_opt = fminunc(costF,ang0)

%% Have a crack at translation:

% costF = @(tran) costTran(tran, imgP(:,:,idz), lidarP(:,:,idz));
% tran0 = [0,0.0,0.0, ang_opt];
% % Optimise transformation
% tran_opt = fminsearch(costF,tran0)
costF = @(tran_opt) costTranslation(tran_opt, imgConstraints, lidarConstraints)
tran_opt = fminunc(costF,[0 0 0 0 0 0])
% tran_opt = [0 0 0 0 0 0];

%% Plot
R = rotatezyx(ang_opt);
% R = rotatezyx([pi/2 pi pi/2])
for j=1:length(lidarP)
    
    P1 = squeeze(lidarP(:,:,j));
    P2 = squeeze(imgP(:,:,j)).';
    tfPts = (rotatezyx(ang_opt)*P1 + tran_opt(1:3).').';
    
    % Histograms
    d(j,:) = mean(P2) - mean(tfPts);

    % Planes:
    fill3(P2(:,1),P2(:,2), P2(:,3),"c") % Image planes un-changed
    hold on
    fill3(tfPts(:,1), tfPts(:,2), tfPts(:,3),"r") % Lidar planes transformed
%     fill3(P1(1,:),P1(2,:), P1(3,:),"g") % Image planes un-changed
    daspect([1 1 1])
end
p0x = [0.2, 0, 0]; p0y = [0, 0.2, 0]; p0z = [0, 0, 0.2];
lidarOrigin = tran_opt(1:3).';
line1 = lidarOrigin + R.'*[0.2; 0; 0];
line2 = lidarOrigin + R.'*[0; 0.2; 0];
line3 = lidarOrigin + R.'*[0; 0; 0.2];

% Camera frame marker:
line([0;p0x(1)], [0;p0x(2)], [0;p0x(3)], "Color", "blue", "LineWidth", 2)
line([0;p0y(1)], [0;p0y(2)], [0;p0y(3)], "Color", "red", "LineWidth", 2)
line([0;p0z(1)], [0;p0z(2)], [0;p0z(3)], "Color", "green", "LineWidth", 2)
% LIDAR frame marker:
line([lidarOrigin(1);line1(1)], [lidarOrigin(2);line1(2)], [lidarOrigin(3);line1(3)], "Color", "blue", "LineWidth", 2)
line([lidarOrigin(1);line2(1)], [lidarOrigin(2);line2(2)], [lidarOrigin(3);line2(3)], "Color", "red", "LineWidth", 2)
line([lidarOrigin(1);line3(1)], [lidarOrigin(2);line3(2)], [lidarOrigin(3);line3(3)], "Color", "green", "LineWidth", 2)
daspect([1 1 1])


figure(2)
subplot(3,1,1)
histogram(d(:,1).*1e3,5)
subplot(3,1,2)
histogram(d(:,2).*1e3,5)
subplot(3,1,3)
histogram(d(:,3),5)

figure(3)
subplot(3,1,1)
bar(d(:,1).*1e3)
subplot(3,1,2)
bar(d(:,2).*1e3)
subplot(3,1,3)
bar(d(:,3))


%% Project on to image
load cameraParams.mat
imagePath = fullfile("newImages");
imgs= dir(fullfile('newImages', '*.png'));
pcds = dir(fullfile('newImages', '*.pcd'));

image = imread(imagePath + "\" + imgs(1).name);
pointCloudin = pcread(imagePath + "\" + pcds(1).name);
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
function c = costRotation(ang, imgConstraints, lidarConstraints)
    R = rotatezyx(ang);
    for i=1:length(lidarConstraints.normal)
       c1 = dot(R*lidarConstraints.normal(i,:).', imgConstraints.normal(i,:).') - 1;
       c2 = dot(R*lidarConstraints.direction(i,:).', imgConstraints.direction(i,:).') - 1;
       c5 = vecnorm(R*lidarConstraints.mean(i,:).' - imgConstraints.mean(i,:).');
       c3(i) = abs(c1) + abs(c2) + c5;
    end
    c = abs(sum(c3))/length(lidarConstraints.normal)
    
end

function c = costTranslation(tf, imgConstraints, lidarConstraints)
    R = rotatezyx(tf(4:6));
    for i=1:length(lidarConstraints.normal)
       c1 = dot(R*lidarConstraints.normal(i,:).', imgConstraints.normal(i,:).') - 1;
       c2 = dot(R*lidarConstraints.direction(i,:).', imgConstraints.direction(i,:).') - 1;
       c5 = vecnorm(R*lidarConstraints.mean(i,:).' + tf(1:3).' - imgConstraints.mean(i,:).');
       c3(i) = abs(c1) + abs(c2) + 50*c5;
    end
    c = abs(sum(c3))/length(lidarConstraints.normal)
    
end


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
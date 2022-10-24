% This script extracts data from images and point clouds taken of a
% checkerboard in an environent.
% It extracts the lidar and image corners and saves them for post
% processing in optimiseTransform.m


clc
clear all
close all
set(groot,'DefaultAxesTickLabelInterpreter','latex');

SAVE_DATA = false; % Flag to save .mat files or not. This will overwrite any files named imgCnrs.mat and lidarCnrs.mat

test = "test2"
load test2/cameraParams.mat
% Extracts checkerboard planes from point clouds, and matches them with the
% plane extracted in the image. 

% Point cloud plane extraction can be a little fiddly, and always gives a
% larger plane than the true dimensions of the checkerboard. 

% To improve this pipeline, need to look at the point cloud extraction.

%% Get all the image and point cloud files:
imagePath = fullfile(test + "/newImages");
imgs= dir(fullfile(test + '/newImages', '*.png'));
pcds = dir(fullfile(test + '/newImages', '*.pcd'));

imgCnrs = nan(46,4,3);
lidarCnrs = nan(46,4,3);

for i=1:length(imgs)
   imPath =  imagePath + "\" + imgs(i).name;
   
   % Get checkerboard points:
   [imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imPath);
   if ~isnan(imagePoints)
       image = imread(imPath);
%        J = insertText(image,imagePoints,1:size(imagePoints,1));
%        J = insertMarker(J,imagePoints,'o','Color','red','Size',5);
       
       % Gets corners of checkerboard: 
       [corners, dims] = estimateCheckerboardCorners3d(image, cameraParams.Intrinsics, 65, 'MinCornerMetric',0.15);
       if (dims(1) ~= 390 || dims(2) ~= 585)
          continue % Incorrect detection
       end
       
       avPt = mean(corners);
       
       adjust = corners - avPt;
       mask = adjust(:,1) < 0 & adjust(:,2) < 0; % Look for top left pt
       tL = corners(mask,:);
       mask = adjust(:,1) > 0 & adjust(:,2) < 0; % Look for top right pt
       tR = corners(mask,:);
       mask = adjust(:,1) > 0 & adjust(:,2) > 0; % Look for botoom right pt
       bR = corners(mask,:);
       mask = adjust(:,1) < 0 & adjust(:,2) > 0; % Look for botoom left pt
       bL = corners(mask,:);
       
       imgCnrs(i,:,:) = [tL;tR;bR;bL];
        
       imPts = projectLidarPointsOnImage([tL;tR;bR;bL;avPt],cameraParams,rigid3d());
       J2 = undistortImage(image,cameraParams);
       figure(1)
       subplot(1,4,1:3)
       imshow(J2)
       hold on
       plot(imPts(1,1),imPts(1,2),'.r','MarkerSize',40)
       plot(imPts(2,1),imPts(2,2),'.g','MarkerSize',40)
       plot(imPts(3,1),imPts(3,2),'.b','MarkerSize',40)
       plot(imPts(4,1),imPts(4,2),'.c','MarkerSize',40)
       plot(imPts(5,1),imPts(5,2),'.y','MarkerSize',40)
       legend("Top left", "Top right", "Bottom left", "Bottom right", "mean")
%        title('Detected Checkerboard Corners')
%        title(sprintf('Detected a %d x %d Checkerboard',boardSize));
       hold off
        
       % Plot planes of checkerboard in 3D:
       subplot(1,4,4)
       p1 = corners(1,:); p2 = corners(2,:); p3 = corners(3,:); p4 = corners(4,:);
       p = [p1; p2; p3; p4; p1];
       plot3(tL(1), tL(2), tL(3), ".r", "MarkerSize", 40)
       hold on
       plot3(tR(1), tR(2), tR(3), ".g", "MarkerSize", 40)
       plot3(bR(1), bR(2), bR(3), ".b", "MarkerSize", 40)
       plot3(bL(1), bL(2), bL(3), ".c", "MarkerSize", 40)
       fill3(p(:,1), p(:,2), p(:,3),1, "DisplayName", "")
        
       p0x = [0.2, 0, 0]; p0y = [0, 0.2, 0]; p0z = [0, 0, 0.2];
       line([0;p0x(1)], [0;p0x(2)], [0;p0x(3)], "Color", "blue", "LineWidth", 2)
       line([0;p0y(1)], [0;p0y(2)], [0;p0y(3)], "Color", "red", "LineWidth", 2)
       line([0;p0z(1)], [0;p0z(2)], [0;p0z(3)], "Color", "green", "LineWidth", 2)
       legend('','','','','','x-axis', 'y-axis', 'z-axis')
       xlabel("x")
       ylabel("y")
       zlabel("z")
       daspect([1 1 1])
   

        drawnow
%% Point cloud segmentation and plane extraction:

        p = pcread(imagePath + "\" + pcds(i).name);
        disp("*** New file read ***")
        [labels,numClusters] = pcsegdist(p,0.5); % Segment point cloud
        for j=1:numClusters
            if size(p.Location(labels == j,:),1) > 500
                pseg = pointCloud(p.Location(labels==j, :));

                % Fit plane using RANSAC: 
                [model, inlierIDX, outlierIDX] = pcfitplane(pseg, 0.03);

                if length(inlierIDX) > 200
                    plane = select(pseg, inlierIDX);

                    % Re-fit plane to better represent most likely plane
                    % fit:
                    planeFit = @(params) planefunc(params(1), params(2), params(3),params(4), plane.Location);
                    params0 = double(model.Parameters);
                    res = fminsearch(planeFit, params0);
%                     idx = getInliers(res, 0.02, plane.Location);

                    % Alternative method to re-fit plane to data:
                    [~,~,V] = svd(plane.Location - mean(plane.Location));
                    svdModel = nan(1,4);
                    svdModel(1:3) = V(:,3);
                    svdModel(4) = -mean(plane.Location)*V(:,3);
                    idx = getInliers(svdModel, 0.02, plane.Location);

                    % Re-define the plane points from better fit:
                    plane = select(plane, idx);

                    % Get minimum bounding box that includes all points in
                    % the plane fit:
                    x = double(plane.Location(:,1));
                    y = double(plane.Location(:,2));
                    z = double(plane.Location(:,3));
                    [rotmat,cnrPts,volume,surface] = minboundbox(x,y,z,'v',3);
                   
                    % Dimension check
                    length_dim = sqrt((cnrPts(1,1) - cnrPts(2,1))^2 + (cnrPts(1,2) - cnrPts(2,2))^2 + (cnrPts(1,3) - cnrPts(2,3))^2);
                    width_dim = sqrt((cnrPts(1,1) - cnrPts(4,1))^2 + (cnrPts(1,2) - cnrPts(4,2))^2 + (cnrPts(1,3) - cnrPts(4,3))^2);

                    if abs(length_dim - 0.6) < 0.07 && abs(width_dim - 0.4) < 0.07 
                        [model.Parameters;res;svdModel] % To show the differences between different plane fits...
                        disp("Found plane!")
                        disp("Length: " + length_dim + ", Width: " + width_dim)
                       

                        avPt1 = mean(cnrPts(1:4,:));
                        avPt2 = mean(cnrPts(5:8,:));
                       
                        [mindepth,idx] = min([avPt1(1), avPt2(1)]);
                        avPt = avPt1*(idx == 1) + avPt2*(idx==2);
                        selectPts = cnrPts(1:4,:)*(idx==1) + cnrPts(5:8,:)*(idx==2);
                      

                        selectPts(:,:) = (cnrPts(1:4,:) + cnrPts(5:8,:))./2; 
       
                        adjust = selectPts(1:4,:) - avPt;
                        mask = adjust(:,2) > 0 & adjust(:,3) > 0; % Look for top left pt
                        tL = selectPts(mask,:);
                        mask = adjust(:,2) < 0 & adjust(:,3) > 0; % Look for top right pt
                        tR = selectPts(mask,:);
                        mask = adjust(:,2) < 0 & adjust(:,3) < 0; % Look for botoom right pt
                        bR = selectPts(mask,:);
                        mask = adjust(:,2) > 0 & adjust(:,3) < 0; % Look for botoom left pt
                        bL = selectPts(mask,:);
                       
                        % Reduce size of bounding box to match checkerboard
                        % dimensions:
                        reducedCnrs = reduceCorners([tL;tR;bR;bL]);
                        tL = reducedCnrs(1,:);
                        tR = reducedCnrs(2,:);
                        bR = reducedCnrs(3,:);
                        bL = reducedCnrs(4,:);
                       
                        % Plotting
                        figure(3)
                        plot3(p.Location(labels==j, 1), p.Location(labels==j, 2), p.Location(labels==j, 3), '.r', MarkerSize=4)
                        hold on
%                         plot3(avPt(1), avPt(2), avPt(3), '.y', "MarkerSize", 20)
                        plot3(tL(1),tL(2),tL(3), '.r', "MarkerSize", 20)
                        plot3(tR(1),tR(2),tR(3), '.g', "MarkerSize", 20)
                        plot3(bR(1),bR(2),bR(3), '.b', "MarkerSize", 20)
                        plot3(bL(1),bL(2),bL(3), '.c', "MarkerSize", 20)
                        fill3([tL(1),tR(1),bR(1),bL(1)],[tL(2),tR(2),bR(2),bL(2)],[tL(3),tR(3),bR(3),bL(3)],"cyan", FaceAlpha=1)
                        fill3(cnrPts(1:4,1), cnrPts(1:4,2), cnrPts(1:4,3),"red", FaceAlpha=0.4)
                        fill3(cnrPts(5:end,1), cnrPts(5:end,2), cnrPts(5:end,3),"blue", FaceAlpha=0.4)
                        xlabel("x")
                        ylabel("y")
                        zlabel("z")
                        axis equal
                       legend("","","", "", "", "Estimated checkerboard plane", "Furthest extreme", "Nearest extreme")
                        hold off
                        drawnow
                       
                        lidarCnrs(i,:,:) = [tL;tR;bR;bL];
                    else
%                        disp("Failed dimension check") 
                    end
                else
                    disp("Failed inlier check")
                end

            else
%                disp("Failed cluster size check") 
            end
        end
    end
end
% Remove nan values:
filter = ~isnan(lidarCnrs);
imgCnrs = imgCnrs(filter(:,1,1),:,:);
lidarCnrs = lidarCnrs(filter(:,1,1),:,:);

if SAVE_DATA == true
    % Save the data: 
    save("imgCnrs", "imgCnrs")
    save("lidarCnrs", "lidarCnrs")
end


function cnrs = reduceCorners(pts)
    tL = pts(1,:);
    tR = pts(2,:);
    bR = pts(3,:);
    bL = pts(4,:);
    % Unit vector from tL to tR
    u1 = (tR - tL)/vecnorm(tR - tL);
    dist = (vecnorm(tR - tL) - 0.585)./2;
    cnrs(1,:) = tL + dist*u1;
    cnrs(2,:) = tR - dist*u1;
    cnrs(3,:) = bR - dist*u1;
    cnrs(4,:) = bL + dist*u1;
    
    % Unit vector from tL to bL
    u2 = (bL - tL)/vecnorm(bL - tL);
    dist = (vecnorm(bL - tL) - 0.390)/2;
    cnrs(1,:) = cnrs(1,:) + dist*u2;
    cnrs(2,:) = cnrs(2,:) + dist*u2;
    cnrs(3,:) = cnrs(3,:) - dist*u2;
    cnrs(4,:) = cnrs(4,:) - dist*u2;

end


% Plane equation: ax + by + cz + d = 0;
function cost = planefunc(a,b,c,d, points)
    cost = abs(sum(sum(d + a*points(:,1) + b*points(:,2) + c*points(:,3))));
end

% Get inliers from a plane equation:
function inlierIDX = getInliers(plane, Maxdist, points)
    % Get inliers of the plane based off distance from the plane.
    distance = abs(points * plane(1:3)' + plane(4));
    inlierIDX = distance <= Maxdist;
end



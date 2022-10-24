clear all;close all;clc
addpath("..\helperFuncs")
% Another crack a grid mapping. Just testing new functions, new ways of
% mapping before implementing SLAM

map.x_size = 100; % m (absolute width)
map.y_size = 100; % m (absolute width)
map.xlim = [0 100]; % Bounds
map.ylim = [0 100]; % Bounds
map.resolution = 1; % x y resolution m
map.occ_map = single(log(1)*ones(map.x_size/map.resolution,map.y_size/map.resolution)); % Initialise some data for a map. Prior = 0.5.


load('state_meas_data.mat')

figure
for t=1:length(X) % Main loop.
    
    robo_pose = X(:,t); % Known pose
    z_t = reshape(z(:,:,t), [2,11]); % Current measurement
    z_t = z_t(:,(~isnan(z_t(1,:)) | ~isnan(z_t(2,:))));
    
    
    [x_zt,y_zt] = measurementToCartesian(z_t,robo_pose);
    [xz_ind,yz_ind] = measurementToGrid(x_zt,y_zt,map);
    [xr_ind,yr_ind] = measurementToGrid(robo_pose(1),robo_pose(2),map);
    
    for beam=1:size(z_t,2)
        [x_hit, y_hit] = bresenham(xr_ind,yr_ind, xz_ind(beam), yz_ind(beam)); % Get cell indices between robot and end of beam.
        [x,y] = cellsToCartesian(x_hit,y_hit,map);        
        cells = [x;y];
        probs = inverseSensorModel(cells,robo_pose,z_t(:,beam),map.resolution);

        ind = sub2ind(size(map.occ_map), y_hit, x_hit);
        map.occ_map(ind) = map.occ_map(ind) + probs;
    end
   

    p = exp(map.occ_map)./(1 + exp(map.occ_map));
    if nonzeros(isnan(p))
       disp("something wrong... Got nan in my map")
       pause 
    end
    p(p < 0.1) = 0.1;
    p(p > 0.99) = 0.99;
    
    s = occupancyMap(p,1/map.resolution);
    show(s)
    hold on
    plot(robo_pose(1),robo_pose(2), 'bo')
    plot(x_zt,y_zt, 'r*')
    for i = 1:size(z_t,2)
        plot([robo_pose(1),x_zt(i)],...
            [robo_pose(2),y_zt(i)],'-b') % Plot intersecting rays
    end
    hold off
    pause(0.05)
end

figure
% Threshhold the map and save it:
thresholdMap = p;
thresholdMap(thresholdMap < 0.5) = 0;
thresholdMap(thresholdMap >= 0.5) = 1;
finalMap = occupancyMap(thresholdMap,1/map.resolution);
show(finalMap);

save("map", "thresholdMap");






function coordinate = rayCast(pose,beam_angles,maxrange,map)
% Pose - robot pose
% Angle - beam angle relative to robot
% Maxrange - maximum range of beam
% Map - occupancy grid

[x_zt,y_zt] = measurementToCartesian([maxrange*ones(1,size(beam_angles,2));beam_angles], pose); % get end points of ray
[xz_ind,yz_ind] = measurementToGrid(x_zt, y_zt, map); % Be careful here cause it will truncate and just "hit" on the boundary of the map...
[xr_ind,yr_ind] = measurementToGrid(pose(1), pose(2), map);

if map.occ_map(yr_ind,xr_ind) > 0.5
    coordinate(:,beam) = [0;0];
end


coordinate = zeros(2,size(beam_angles,2));
for beam=1:size(beam_angles,2)
    [x_hit, y_hit] = bresenham(xr_ind,yr_ind, xz_ind(beam), yz_ind(beam)); % Get cell indices between robot and end of beam.
    x_hit_reduced = x_hit( y_hit <= size(map.occ_map,1) & y_hit > 0 & x_hit <= size(map.occ_map,1) & x_hit > 0);
    y_hit_reduced = y_hit( y_hit <= size(map.occ_map,1) & y_hit > 0 & x_hit <= size(map.occ_map,1) & x_hit > 0);
    ind = sub2ind(size(map.occ_map), y_hit_reduced, x_hit_reduced);
    k = find(map.occ_map(ind) > 0.7);
    if isempty(k)
        coordinate(:,beam) = [x_zt(beam);y_zt(beam)];
    else
        indx = x_hit_reduced(k); 
        indy = y_hit_reduced(k);
        [x,y] = cellsToCartesian(indx,indy, map); % Get x and y pos of all occupied cells that the ray passed through.
        
        % Find the argmin of the range:
        r = sqrt( (pose(1) - x.').^2 + (pose(2) - y.').^2  );
        [~,ind] = min(r);
        
        %coordinate(:,beam) = [x(ind);y(ind)];
        
        cell_coordinate(:,beam) = [x(ind);y(ind)];
        
        
        x1 = cell_coordinate(1,beam) - map.resolution/2;
        y1 = cell_coordinate(2,beam) - map.resolution/2;
        x2 = cell_coordinate(1,beam) + map.resolution/2;
        y2 = cell_coordinate(2,beam) + map.resolution/2;
        
        
        a = occupancyMap(map.occ_map,1);
        show(a)
        hold on
        plot(pose(1),pose(2))
        plot([x_zt(beam),pose(1)],[y_zt(beam),pose(2)])
        xline(x1)
        xline(x2)
        yline(y1)
        yline(y2)
        
        [xt,yt] =cellsToCartesian(x_hit_reduced,y_hit_reduced,map)
        plot(xt,yt,'r+')
        hold off
       
        theta = pose(3) + beam_angles(beam);
        m = tan(theta);
        b = pose(2) - m*pose(1);
        
        ray_points = [...
            x1 m*x1 + b;
            (y1-b)/m y1;
            x2 m*x2 + b;
            (y2-b)/m y2];
        
        index = ray_points(:,1) >= x1 & ray_points(:,1) <= x2 & ray_points(:,2) >= y1 & ray_points(:,2) <= y2;
        reduced_ray_points = ray_points(index,:);
        
        r = sqrt( (ray_points(:,1).' - pose(1)).^2 + (ray_points(:,2).' - pose(2)).^2   );
        [~,ind] = min(r(index));
        
        coordinate(:,beam) = reduced_ray_points(ind,:).';
        
    end
end
end


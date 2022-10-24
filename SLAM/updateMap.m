function updated_map = updateMap(z_t, pose, map, map_params)

    current_map.x_size = map_params.x_size;
    current_map.y_size = map_params.y_size;
    current_map.xlim = map_params.xlim;
    current_map.ylim = map_params.ylim ;
    current_map.resolution = map_params.resolution; 
    current_map.occ_map = map;

    [x_zt,y_zt] = measurementToCartesian(z_t(:,~isnan(z_t(1,:))),pose);
    [xz_ind,yz_ind] = measurementToGrid(x_zt,y_zt,current_map);
    [xr_ind,yr_ind] = measurementToGrid(pose(1),pose(2),current_map);
    
    for beam=1:size(z_t(:,~isnan(z_t(1,:))),2)
        [x_hit, y_hit] = bresenham(xr_ind,yr_ind, xz_ind(beam), yz_ind(beam)); % Get cell indices between robot and end of beam.
        [x,y] = cellsToCartesian(x_hit,y_hit,current_map);        
        cells = [x;y];
        probs = inverseSensorModel(cells,pose,z_t(:,beam),current_map.resolution);

        ind = sub2ind(size(current_map.occ_map), y_hit, x_hit);
        if isnan(ind) | isempty(ind)
           disp("We have issues") 
        end
        
        current_map.occ_map(ind) = current_map.occ_map(ind) + probs;
    end
    
    updated_map = current_map.occ_map;
end


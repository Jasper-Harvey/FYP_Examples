function [x_ind, y_ind] = measurementToGrid(x,y, map)
    % Convert given x and y coordinates to grid cell indices.
    % Cell 1,1 should be top left of grid (xlim(2), ylim(2))
    % Cell (end,1) should be bottom left (xlim(1), ylim(1))
    
    cell_num = [map.x_size/map.resolution;map.y_size/map.resolution];
    
    x_ind = ceil(x/map.resolution);
    y_ind = cell_num(2) + 1 - ceil(y/map.resolution);
    
    % Some checks: (This breaks ray casting)
    y_ind(y_ind > cell_num(2)) = cell_num(2);
    y_ind(y_ind < 1) = 1;
    
    x_ind(x_ind < 1) = 1;
    x_ind(x_ind > cell_num(1)) = cell_num(1);
end
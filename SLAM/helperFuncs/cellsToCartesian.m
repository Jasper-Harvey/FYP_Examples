function [x,y] = cellsToCartesian(x_ind,y_ind,map)
    % Computes the centre of mass of a cell 
    % (1,1) = 0 + 0.5 , 99.5
    % (1,1) = x_ind*resolution, y_ind*resolution 
    y = map.y_size - (y_ind - 1)*map.resolution - map.resolution/2;
    x = (x_ind - 1)*map.resolution + map.resolution/2;
end
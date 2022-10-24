function logProb = inverseSensorModel(cells,pose, z, cell_size)
%INVERSESENSORMODEL
% Input: 
% cells = the relevent cells to check for the beam. (center of mass of the cell)
% pose = current pose of the robot
% z = range and angle of the beam/beams
% cell_size = x and y width of the cell. (grid resolution)

% Returns log probability of the given cells

l_occ = log(0.70/0.30);
l_free = log(0.30/0.70);
alpha = cell_size; % Thickness of obstacles (m). Should be kinda related to the grid size too.
beta = 1*pi/180;
zmax = 100;

% Compute range to each of the cells

r_cells = sqrt((cells(1,:) - pose(1)).^2 + (cells(2,:) - pose(2)).^2);

beam_angles = pose(3) + z(2,:); % Adjust beam angles to map coordinates.
endpt_x = pose(1) + z(1,:).*cos(beam_angles);
endpt_y = pose(2) + z(1,:).*sin(beam_angles);

% Generate mask based on logic for occupied or not occupied cells:
%occ_mask = (z(1) < zmax) & ((r_cells - z(1)) <= alpha/2.0 & (r_cells - z(1)) > 0);
%free_mask = r_cells <= (z(1) - alpha/2.0);

% New logic. Not sure if its good???
occ_mask = (z(1,:) < zmax) & endpt_x >= (cells(1,:) - cell_size/2) & endpt_x <= (cells(1,:) + cell_size/2) & endpt_y >= (cells(2,:) - cell_size/2) & endpt_y <= (cells(2,:) + cell_size/2); 
free_mask = ~occ_mask; % This only works if we know we wont have ranges greater than the max range and every cell that we are cheking can be viewed by the sensor...
                       % Just need to make sure we always give this function the right data. 


logProb = zeros(1,length(cells));
logProb(occ_mask) = l_occ;
logProb(free_mask) = l_free;
end


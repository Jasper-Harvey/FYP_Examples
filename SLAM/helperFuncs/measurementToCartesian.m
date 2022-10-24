function [x, y] = measurementToCartesian(z,pose)
    % Convert range measurement\s to x,y coords
    beam_angles = pose(3) + z(2,:);
    x = pose(1) + z(1,:).*cos(beam_angles);
    y = pose(2) + z(1,:).*sin(beam_angles);
end


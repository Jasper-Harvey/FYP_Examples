clear all
clc

load("map.mat"); 
map.occ_map = thresholdMap; 


robo_pose = [5;6;2*pi/180]; % x,y,theta

T = 340; % Final time
map_display = occupancyMap(map.occ_map,1);
delta_t = 5; % made this bigger cause my inputs are tiny....
for t=1:T  
    % New pose
    robo_pose = stateTransition(robo_pose, robotInputs(t), delta_t);   

    hold off
    map_display = occupancyMap(map.occ_map,1);
    show(map_display);
    hold on
    plot(robo_pose(1),robo_pose(2), 'bo')
    plot([robo_pose(1), robo_pose(1) + 2*cos(robo_pose(3))],[robo_pose(2), robo_pose(2) + 2*sin(robo_pose(3))] )
    
   
    pause(0.01)

    end
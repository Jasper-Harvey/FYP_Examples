clear all
close all
clc
addpath(".\helperFuncs");
set(groot,'DefaultAxesTickLabelInterpreter','latex');

% Particle filter localisation

load("map.mat"); 
map.x_size = 100; % m (absolute width)
map.y_size = 100; % m (absolute width)
map.xlim = [0 100]; % Bounds
map.ylim = [0 100]; % Bounds
map.resolution = 1; % x y resolution m
map.occ_map = thresholdMap; 

delta_t = 0.5; % Made this large cause robot wasnt moving fast enough.

% Generate some random particles to put around the place:
n_p = 300; % Number of particles

% particles(:,:) = [randsample([3:1:80],n_p,true);randsample([3:1:80],n_p,true);randsample([2*(-pi/180):0.01:2*(pi/180)],n_p,true)]; % each row is a random pose
% particles(:,:) = [5;6;2*pi/180]; % each row is a random pose
particles(:,:) = [randsample([3:1:97],n_p,true);randsample([3:1:97],n_p,true);zeros(1,n_p)];

beam_angles = [-pi/2:pi/10:pi/2];
maxrange = 200;
robo_pose = [5;6;2*pi/180]; % actual robot position to be estimated.

load inputs.mat;
%% Localisation loop:
T = 340; % Final time
map_display = occupancyMap(map.occ_map,1);
probs = zeros(1,n_p);
c =1; 
for t=1:0.1:length(inputs)  
    tic
    % New pose
%     if mod(t,1) == 0
%         input = robotInputs(t) + [0.01; 0.01;0.003].*randn(3,1);
%     end
%     robo_pose = stateTransition(robo_pose, input, delta_t);
    robo_pose = stateTran(robo_pose, inputs(round(t/0.1 - 9,0),:), 0.5);
    
    robot_trajectory(:,round(t/0.1 - 9,0)) = robo_pose;
    
    % Propagate particles through state transition matrix:
    q = 1*eye(3,3);
    q(3,3) = q(3,3)*0.3; % Reduce the noise in the angle 
    particles(:,:) = stateTransition(particles(:,:), zeros(3,n_p), delta_t) + q*randn(3,size(particles,2));
    
    % Get measurements from the robot:
    good_robo_pts = rayIntersection(map_display,robo_pose,beam_angles,maxrange).';
    good_robo_ranges = sqrt((good_robo_pts(1,:) - robo_pose(1)).^2 + (good_robo_pts(2,:) - robo_pose(2)).^2) + 0.1*eye(1,length(beam_angles))*randn(length(beam_angles),length(beam_angles)); 
    coords = good_robo_pts;

    % Loop through particles:
    for ppp=1:n_p
           good_particle_pts = rayIntersection(map_display,particles(:,ppp),beam_angles,maxrange).';
           good_particle_ranges = sqrt((good_particle_pts(1,:) - particles(1,ppp)).^2 + (good_particle_pts(2,:) - particles(2,ppp)).^2); % (ranges of all particles)
           
           
%            probs(ppp) = measurementModel(good_robo_ranges.',good_particle_ranges.');
           logP(ppp) = logmeasurementModel(good_robo_ranges.',good_particle_ranges.');
           old_pts(:,:,ppp) = good_particle_pts;
    end
    
     % Generate normalised likelihoods (weights) from meaasurement 
     if t > 0 % t > 1 because I want a nice plot.
        w = probs/sum(probs);
        lw = logP - max(logP) - log(sum(exp(logP - max(logP))));
        % resample
%         ii = randsample(n_p,n_p,true,w);
        ii = randsample(n_p,n_p,true,exp(lw));
        particles = particles(:,ii);

        % Best particle
        [~,idx] = max(exp(lw));
        pbest(c,:) = particles(:,idx).';
        c = c + 1;
     end
    toc


   
    if mod(t,1) == 0
    hold off
    map_display = occupancyMap(map.occ_map,1);
    show(map_display);
    hold on
    plot(robo_pose(1),robo_pose(2), 'bo', "MarkerSize",10, "LineWidth",2)
    plot([robo_pose(1), (robo_pose(1) + 2*cos(robo_pose(3)))], [robo_pose(2), (robo_pose(2) + 2*sin(robo_pose(3)))], 'b', "LineWidth", 2)
    
%     plot(coords(1,:),coords(2,:), 'r*')
%     for i = 1:size(coords,2)
%         plot([robo_pose(1),coords(1,i)],...
%             [robo_pose(2),coords(2,i)],'-b') % Plot intersecting rays
%     end
% 
%     plot(pbest(:,1),pbest(:,2));
%     plot(robot_trajectory(1,:),robot_trajectory(2,:), 'r', "LineWidth",1);
    % Plot particles:
    title("Particle Filter Localisation")
    for p=1:n_p
        plot(particles(1,p),particles(2,p),'r.', "MarkerSize",20)
    end
    end
    % Some pausing stuff to give me time to see first resample step
    if t > 1
        pause(0.01)
    else
        pause(3)
    end
 
end
%%
tvec = [0:0.1:50];
figure
subplot(3,1,1)
plot(tvec, pbest(1:length(tvec), 1), 'b', 'LineWidth',0.75)
title("Particle Filter State Estimation", 'FontSize',20, Interpreter='latex')
hold on
plot(tvec, robot_trajectory(1, 1:length(tvec)), 'r', 'LineWidth',1.5)
ax = gca
ax.FontSize = 14
ylabel("$$x$$ (m)", "FontSize",20, Interpreter='latex')
xlabel("time (sec)", "FontSize",20, Interpreter='latex')
legend("Highest likelihood particle", "Ground truth", "FontSize", 14)

subplot(3,1,2)
plot(tvec, pbest(1:length(tvec), 2), 'b', 'LineWidth',0.75)
hold on
plot(tvec, robot_trajectory(2, 1:length(tvec)), 'r', 'LineWidth',1.5)
ax = gca
ax.FontSize = 14
ylabel("$$y$$ (m)", "FontSize",20, Interpreter='latex')
xlabel("time (sec)", "FontSize",20, Interpreter='latex')

subplot(3,1,3)
plot(tvec, pbest(1:length(tvec), 3), 'b', 'LineWidth',0.75)
hold on
plot(tvec, robot_trajectory(3, 1:length(tvec)), 'r', 'LineWidth',1.5)
ax = gca
ax.FontSize = 14
ylabel("$$\theta$$ (rad)", "FontSize",20, Interpreter='latex')
xlabel("time (sec)", "FontSize",20, Interpreter='latex')



function nextState = stateTran(x, u, dt)
    nextState(1) = x(1) + dt*(u(1) + u(2))*sin(x(3));
    nextState(2) = x(2) + dt*(u(1) + u(2))*cos(x(3));
    nextState(3) = x(3) + dt*(u(1) - u(2));
end

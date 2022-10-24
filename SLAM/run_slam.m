clear all; close all; clc;
    
load('map.mat'); % This is the "real" environment.
real_map = occupancyMap(thresholdMap,1);
addpath(".\helperFuncs")


map.x_size = 200; % m (absolute width)
map.y_size = 200; % m (absolute width)
map.xlim = [0 200]; % Bounds
map.ylim = [0 200]; % Bounds
map.resolution = 1; % x y resolution m

params.x_size = map.x_size;
params.y_size = map.y_size;
params.xlim = map.xlim;
params.ylim = map.ylim ;
params.resolution = map.resolution; 

% Generate some random particles to put around the place:
n_p = 500; % Number of particles
particles = zeros(3,n_p);
for i=1:n_p
    particles(:,i) = [randsample([40:0.1:60],1),randsample([40:0.1:60],1),randsample([0,0,0],1)]; % each row is a random pose
end

mean_map = single(log(1)*ones(map.x_size/map.resolution,map.y_size/map.resolution));

% Initalise the real position of the robot (not known):
robo_pose = [5;5;2*pi/180];
beam_angles = [-pi/2:pi/60:pi/2]; % LIDAR beam angles
maxrange = 200; % Max range of range finder 
delta_t = 5; % For state transition func. 

figure(1)
pind = 1;
probs = zeros(1,n_p);
T = 340;
for t=1:T % Main loop.
    %% This is the stuff we dont know
    % New pose
    if mod(t,1) == 0
       input =  robotInputs(t);
    end
    
    robo_pose = stateTransition(robo_pose, input, delta_t);
    rp(t,:) = robo_pose;
    % Generate some robot measurements (from the 'real' position)
    robo_pts = rayIntersection(real_map,robo_pose,beam_angles,maxrange).'; % Points of intersection on the map 
    robo_ranges = sqrt((robo_pts(1,:) - robo_pose(1)).^2 + (robo_pts(2,:) - robo_pose(2)).^2) + 0.1*eye(1,length(beam_angles))*randn(length(beam_angles),length(beam_angles)); % Converted to ranges.. (the measurements we will get)
    
    tic
    %% Try to estimate it here :O
    % Propagate particles through state transition matrix:
    q = 1.5*eye(3,3); 
    q(3,3) = q(3,3)*0.07; % Reduce the noise in the angle 
    particles(:,:) = stateTransition(particles(:,:), input, delta_t) + q*randn(3,n_p);
    
    % Loop through all particles
    for j=1:n_p
        if t > 1
              
           mProb = exp(mean_map)./(1 + exp(mean_map)); % This is for the mapping library I am using
           m = occupancyMap(mProb,1); % This is for the mapping library I am using
           particle_pts = rayIntersection(m,particles(:,j),beam_angles,maxrange).';
           particle_ranges = sqrt((particle_pts(1,:) - particles(1,j)).^2 + (particle_pts(2,:) - particles(2,j)).^2); % (ranges of all particles)

%            probs(j) = measurementModel(robo_ranges.',particle_ranges.');
           logP(j) = logmeasurementModel(robo_ranges.',particle_ranges.');
           
           old_pts(:,:,j) = particle_pts;
        end
    end
    
    % Generate normalised likelihoods (weights) from meaasurement 
     % resample
     if t > 1 
%         w = probs/sum(probs);
        lw = logP - max(logP) - log(sum(exp(logP - max(logP))));
        ii = randsample(n_p,n_p,true,exp(lw));
        particles = particles(:,ii);
        [~,pind] = min(lw); % Get index of the best particle
        disp('Resampled!')
        if t > 300
           bp = 1; 
        end
     end
     
     % Weighed mean of particles:
     mean_part(:,t) = mean(particles,2);
     mean_map = updateMap([robo_ranges;beam_angles], mean_part(:,t),mean_map, params);
     toc

     
     
%     disp("Error:")
%     disp(mean_part - robo_pose)
    %% Plotting stuff:
    p = exp(mean_map)./(1 + exp(mean_map));
    if nonzeros(isnan(p))
       disp("something wrong... Got NaN in my map")
       pause 
    end
    p(p < 0.1) = 0.1;
    p(p > 0.99) = 0.99;
    
    s = occupancyMap(p,1/map.resolution);
    show(s)
    hold on

    poseDiff = mean_part(:,1) - [5;5;2*pi/180]; 
    plot(rp(:,1) + poseDiff(1), rp(:,2) + poseDiff(2), 'r')
%     plot(mean_part(1,1:1:end),mean_part(2,1:1:end), 'b')
    %plot(x_zt,y_zt, 'r*')
    for i = 1:n_p
        plot(particles(1,i),particles(2,i), 'r.', "MarkerSize",10)
    end
   xlim([25 175]);
   ylim([25 175]);
   ax = gca;
   ax.FontSize = 18;
   hold off
   pause(0.001)
    
    if mod(t,1) == 0
%         Frames(t) = getframe(gcf);
    end
    drawnow

    if t==1
        disp("Paused.....")
%         pause
    end
end
    
writerObj = VideoWriter('myVideo.avi');
vidlen = 10;
writerObj.FrameRate = T/vidlen;
% set the seconds per image
  
% open the video writer
open(writerObj);
% write the frames to the video
for i=1:length(Frames)
    % convert the image to a frame
    frame = Frames(i) ;    
    writeVideo(writerObj, frame);
end
% close the writer object
close(writerObj);
    
    
    
    
    
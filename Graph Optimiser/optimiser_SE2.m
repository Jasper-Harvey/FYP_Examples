% Pose graph optimisation implementation from 
% http://rvsn.csail.mit.edu/graphoptim/eolson-graphoptim2006.pdf

% Imports information from .g2o files. Must be 2D graphs.

close all
clear all
clc

% filename = 'input_M3500a_g2o.g2o';
% filename = 'input_M3500c_g2o.g2o';
% filename = 'input_M3500_g2o.g2o';
% filename = 'input_MITb_g2o.g2o';
filename = 'input_INTEL_g2o.g2o';
[nodes, nodePairs, links, informationM] = importG2O(filename);

% idx of nodes == nodeID 
% idx of nodePairs == linkID

% Access constraint information from nodePair n:
% links(n)

% Access node from node Pairs:
% nodes(nodePair(n,1)) for node a
% nodes(nodePair(n,1)) for node b
initNodes = nodes; % Un-optimised nodes
figure
% plot(nodes(:,1), nodes(:,2))
hold on
plot(nodes(:,1), nodes(:,2), ".", 'markersize', 5)
title("Trajectory - 150 Iterations", 'fontSize', 20)
set(gca,'xtick',[])
set(gca,'ytick',[])
for z = 1:length(nodePairs)
   m = nodePairs(z,1);
   n = nodePairs(z,2);
   xdata = [nodes(m,1), nodes(n,1)];
   ydata = [nodes(m,2), nodes(n,2)];
   if n == m+1
       plot(xdata,ydata, 'b', 'linewidth', 3);
   else
       plot(xdata,ydata, 'r--', 'linewidth', 1.75);
   end
end

%%
% Made some minor modifications to use the information matrix instead of
% the covariance matrix.
tempa = -999;
maxIter = 1000;
Nll = zeros(maxIter,1);
for iters=[1:maxIter]     
    subplot(1,2,1)
    plot(nodes(:,1), nodes(:,2))
    subplot(1,2,2)
    plot(1:iters,Nll(1:iters))
    
    
    pause(0.5)
    iters
    y = [inf inf inf].';
    M = zeros(length(nodes),3);
    % for all constraints:
    for i=1:length(links)
        % Retrive link pose
        infMat = informationM(i, :);
        sig = [infMat(1), infMat(2), infMat(3);
               infMat(2), infMat(4), infMat(5);
               infMat(3), infMat(5), infMat(6)]; 
          
        a = nodePairs(i,1);
        b = nodePairs(i,2);
        if a > b
           sig = sig.';
           tempa = a;
           a = b;
           b = tempa;
        end
        % Retrieve point a
        pa = nodes(a,:).';
        Ra = [cos(pa(3)), -sin(pa(3)), 0;
             sin(pa(3)), cos(pa(3)), 0;
             0, 0, 1]; % Rotation matrix
        

         % R*infM*R.' is information matrix in global frame
        W = Ra*sig*Ra.'; % Different form of W


        ind = [a+1:b]; 
        for j = ind
            M(j,:) = M(j,:) + diag(W).';
%             y = min(y,diag(W));
            D = diag(W);
            y(1) = min(y(1),D(1));
            y(2) = min(y(2),D(2));
            y(3) = min(y(3),D(3));
        end

    end
    % Modified Stochastic Gradient Descent
    idx = randsample(length(links),length(links), 'false').';
    for i=1:length(links)
        % Retrive link pose
        TF = links(i, :);
        infMat = informationM(i, :);
        sig = [infMat(1), infMat(2), infMat(3);
               infMat(2), infMat(4), infMat(5);
               infMat(3), infMat(5), infMat(6)];

        a = nodePairs(i,1);
        b = nodePairs(i,2);
        if a > b
           sig = sig.';
           tempa = a;
           a = b;
           b = tempa;
        end
        % Retrieve point a
        pa = nodes(a,:).';
        Ra = [cos(pa(3)), -sin(pa(3)), 0;
             sin(pa(3)), cos(pa(3)),   0;
             0,            0,          1]; % Rotation matrix
        Pa = [Ra(1:2,1:2), pa(1:2);
                0,0,1];

        % Retrieve point b
        pb = nodes(b,:).';
        Rb = [cos(pb(3)), -sin(pb(3)), 0;
             sin(pb(3)), cos(pb(3)), 0;
             0, 0, 1]; % Rotation matrix
        Pb = [Rb(1:2,1:2), pb(1:2);
                0,0,1];

        Rab = [cos(TF(3)), -sin(TF(3)), 0;
              sin(TF(3)), cos(TF(3)), 0;
              0, 0, 1]; % Rotation matrix
        if tempa == b
           Rab = Rab.';
           TF(1:2) = -TF(1:2);
        end
        Tab = [Rab(1:2,1:2), TF(1:2).';
                0,0,1];

        Pb_prime = Pa*Tab;
        rotation_component = [Pb_prime(1,1:2), 0;
                              Pb_prime(2,1:2),0;
                              0,0,1];
        rot = rotm2eul(rotation_component);
        pb_prime = [Pb_prime(1:2,3); rot(1)];
        
        uw = unwrap([rot(1), pb(3)]);
        pb_prime(3) = uw(1);
        pb(3) = uw(2);
        
        r = pb_prime - pb;
%         r(3) = mod(r(3), 2*pi);

        Nll(iters) = Nll(iters) + r.'*sig*r; % Compute negative log likelihood
        d = 2*(Ra.'*sig*Ra)*r;
        

        ind = [a+1:b]; 
        
        for k = 1:3
           alpha = 1/(y(k)*iters);
           total_weight = sumM(M,a,b,k);
           beta = (b - a)*d(k)*alpha; % This almost never gets used
           
           if abs(beta) > abs(r(k))
               beta = r(k);
%                  beta;
           else
               beta;
           end
           
           dpose = 0;
           lower = (a+1)*(a < b) + (b+1)*(a > b);
           for m = [lower:length(nodes)]
               if sum(m == ind) >= 1
                   dpose = dpose + beta/M(m,k)/total_weight; % Total_weight is broken
               end 
               nodes(m,k) = nodes(m,k) + dpose;
           end
           dpose;
        end
    end
end



function total = sumM(M,l,u,j)
    t = 0;
    vec = l+1:u;
    for z=vec
        t = t + 1/M(z,j);
    end
    total = t;
end

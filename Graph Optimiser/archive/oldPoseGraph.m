close all
clear all
clc

load intel-2d-posegraph.mat pg
% load grid-2d-posegraph pg

% pg = optimizePoseGraph(pg);

nodePairs = edgeNodePairs(pg); % Each row is a pair of nodes that form an edge
[links, informationM] = edgeConstraints(pg);
nodes = nodeEstimates(pg);


% idx of nodes == nodeID
% idx of nodePairs == linkID

% Access constraint information from nodePair n:
% links(n)

% Access node from node Pairs:
% nodes(nodePair(n,1)) for node a
% nodes(nodePair(n,1)) for node b
initNodes = nodes;
figure
plot(nodes(:,1), nodes(:,2))

%% Attempt at algo from 
% http://rvsn.csail.mit.edu/graphoptim/eolson-graphoptim2006.pdf
maxIter = 500;

for iters=[1:maxIter]       
    plot(nodes(:,1), nodes(:,2))
%     hold on
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
               infMat(3), infMat(5), infMat(6)]\eye(3); % Invert information matrix (bad I know)

        a = nodePairs(i,1);
        b = nodePairs(i,2);
        % Retrieve point a
        pa = nodes(a,:).';
        Ra = [cos(pa(3)), -sin(pa(3)), 0;
             sin(pa(3)), cos(pa(3)), 0;
             0, 0, 1]; % Rotation matrix

         % R*infM*R.' is information matrix in global frame
        W = (Ra*sig*Ra.')\eye(3);

        ind = [a+1:b]; 
        for j = ind
            M(j,:) = M(j,:) + diag(W).';
            y = min(y,diag(W));
        end
        
    end
    
    % Modified Stochastic Gradient Descent
    for i=1:length(links)
        % Retrive link pose
        TF = links(i, :);
        infMat = informationM(i, :);
        sig = [infMat(1), infMat(2), infMat(3);
               infMat(2), infMat(4), infMat(5);
               infMat(3), infMat(5), infMat(6)]\eye(3); % Invert information matrix (bad I know)

        a = nodePairs(i,1);
        b = nodePairs(i,2);
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
        Tab = [Rab(1:2,1:2), TF(1:2).';
                0,0,1];

        
        Pb_prime = Pa*Tab;
        rotation_component = [Pb_prime(1,1:2), 0;
                              Pb_prime(2,1:2),0;
                              0,0,1];
        rot = rotm2eul(rotation_component);
        pb_prime = [Pb_prime(1:2,3); rot(1)];
        r = pb_prime - pb;
%         r(3) = mod(r(3), 2*pi);

        d = 2*((Ra.'*sig*Ra)\eye(3))*r;
        
        ind = [a+1:b]; 
        
        for k = 1:3
           alpha = 1/(y(k)*iters);
           total_weight = sumM(M,a,b,k);
           beta = (b - a)*d(k)*alpha;
           
           if abs(beta) > abs(r(k))
               beta = r(k);
           end
           
           dpose = 0;
           for m = [a+1:length(nodes)]
               if sum(m == ind) >= 1
                   dpose = dpose + beta/M(m,k)/total_weight; % Total_weight is broken
%                    dpose =  dpose + beta/M(m,k)/total_weight;
               end          
               nodes(m,k) = nodes(m,k) + dpose;
           end
           
        end
    end
end



function total = sumM(M,l,u,j)
    t = 0;
    for z=l+1:u
        t = t + 1/M(z,j);
    end
    total = t;
end

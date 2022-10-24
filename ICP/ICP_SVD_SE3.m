function best_pose = ICP_SVD_SE3(P1, P2, iterations)
% Basic implementation of ICP
% Uses SVD to calculate the rotation matrix

% P1 static points
% P2 moving points

T = eye(4); % Initial homogeneous transform
iters = 200;
min_err = 1e-4;
min_iters = 5;
err = 0;

movingP = [P2, ones(size(P2,1),1)];
staticP = [P1, ones(size(P1,1),1)];

for i=1:iters
    if i == iterations
       break 
    end
    
    last_err = err;
    
    P1_knn = staticP - mean(staticP,1);
    P2_knn = movingP - mean(movingP,1);
    
%     [idx, distance] = knnsearch(movingP(:,1:3), staticP(:,1:3), "Distance", "euclidean"); % Search for P2 in P1
    [idx, distance] = knnsearch(P2_knn(:,1:2), P1_knn(:,1:2)); % Search for P2 in P1
    
    % Remove duplicate indicies:
    unique = false;
    while ~ unique
        unique = true;
        for j = 1:length(idx)
            if idx(j) == -1
                continue
            end
            for k = j+1:length(idx)
                if idx(j) == idx(k)
                    if distance(j) < distance(k)
                        idx(k) = -1;
                    else
                        idx(j) = -1;
                        break
                    end
                end
            end
        end
    end % End while 
    
%     for j = 1:length(idx)
%        if distance(j) > 10
%           idx(j) = -1;
%           disp("rejected data")
%        end
%     end
    
%     pair_static = staticP(idx ~= -1, 1:3);
%     idx = idx(idx ~= -1);
%     pair_moving = movingP(idx, 1:3);

    pair_static = P1_knn(idx ~= -1, 1:3);
    idx = idx(idx ~= -1);
    pair_moving = P2_knn(idx, 1:3);
    
%     hold off
%     plot(pair_moving(:,1), pair_moving(:,2), "b*")
%     hold on
%     plot(pair_static(:,1), pair_static(:,2), "r*")
%     for z=1:size(pair_moving,1)
%        plot([pair_moving(z,1), pair_static(z,1)], [pair_moving(z,2), pair_static(z,2)], 'b')
%     end
    
    
%     static_cent = pair_static - mean(pair_static,1);
%     moving_cent = pair_moving - mean(pair_moving,1);
    
    static_cent = pair_static;
    moving_cent = pair_moving;
    
    W = moving_cent.'*static_cent;
    [U,~,V] = svd(W);
    
    tempR = (V*diag([ones(1,2) sign(det(U*V'))])*U.') ;% Handle reflection
    
%     tempR = (U*V).';
%     tempt = mean(pair_static,1).' - (tempR*mean(pair_moving,1).');
    tempt = mean(staticP(:,1:3),1).' - (tempR*mean(movingP(:,1:3),1).');
%     tempt = mean(pair_moving).' - (tempR*mean(pair_static).');

    newT = eye(4);
    newT(1:3,1:3) = tempR(1:3,1:3);
    newT(1:3,4) = tempt;
    
    T = T*newT;
    
    best_pose = T;
    
    movingP = (T*[P2, ones(size(P2,1),1)].').';
    
    q = (T*[pair_moving, ones(size(pair_moving,1),1)].').';
    diff = pair_static - q(:,1:3);
    
    err = sum(sum(diff.'*diff))/length(pair_moving)
    if (abs(last_err - err) < min_err) && (i >= min_iters)
       disp("Exiting.. error tolerance has been reached after " + i + " iterations")
       break 
    end
    
end % end iters
    disp("Exiting.. after " + i + " iterations")
    
end % End function
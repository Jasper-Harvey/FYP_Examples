function new_pose = stateTransition(pose_old, input,delta_t)
% Very shitty state transition matrix.

x_new = pose_old(1,:) + input(1)*delta_t;
y_new = pose_old(2,:) + input(2)*delta_t;
theta_new = pose_old(3,:) + input(3)*delta_t;

new_pose = [x_new;y_new;theta_new];

end



x(1,:) = [0,0,0];
u = [0.1;0];
for k=[1:400]
    x(k+1,:) = stateTran(x(k,:), u, 0.1);
    if k > 200
       u = [0;0.1]; 
    end
   
end


plot(x(:,1),x(:,2))
ylim([-2 2])
xlim([-2 2])

function nextState = stateTran(x, u, dt)
    nextState(1) = x(1) + dt*(u(1) + u(2))*sin(x(3));
    nextState(2) = x(2) + dt*(u(1) + u(2))*cos(x(3));
    nextState(3) = x(3) + dt*(u(1) - u(2));
end
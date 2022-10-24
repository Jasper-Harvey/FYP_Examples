function input = robotInputs(t)

if t <= 20
    x_input = 0.1;
    y_input = 0.1;
    theta_input = 0;
elseif t <= 40
    x_input = 0;
    y_input = 0; 
    theta_input = 0.003;
elseif t <= 60
    x_input = 0.1;
    y_input = 0;
    theta_input = 0;
elseif t <= 80
    x_input = 0.1;
    y_input = 0;
    theta_input = 0;
elseif t <= 100
    x_input = 0;
    y_input = 0;
    theta_input = 0.001;
elseif t <= 120
    x_input = 0.1;
    y_input = 0.1;
    theta_input = 0;
elseif t <= 140
    x_input = 0;
    y_input = 0;
    theta_input = 0.0006;
elseif t <= 180
    x_input = 0;
    y_input = 0;
    theta_input = 0.006;
elseif t <= 200
    x_input = 0.1;
    y_input = 0.1;
    theta_input = 0;
elseif t <= 280
    x_input = 0;
    y_input = 0.1;
    theta_input = 0;
elseif t <= 300
    x_input = 0;
    y_input = 0;
    theta_input = 0.02;
elseif t <= 330
    x_input = 0;
    y_input = 0.002;
    theta_input = -0.02;
elseif t <= 340
    x_input = 0.1;
    y_input = 0.1;
    theta_input = 0;
    
    
    
    
else
    x_input = 0;
    y_input = 0;
    theta_input = 0;
    
end
    

input = [x_input;y_input;theta_input];
end


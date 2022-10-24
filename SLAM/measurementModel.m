function prob = measurementModel(measurement, particle_measurement)
    
    error = measurement(~isnan(particle_measurement)) - particle_measurement(~isnan(particle_measurement));
    if isempty(error)
        prob = 0;
        return
    end
    Q = 0.1*eye(length(error));
    prob = (1/sqrt(2*pi*norm(Q)))*exp(-0.5*(error.')* Q * error ); 
    
    prob(isnan(prob)) = 0; 

end


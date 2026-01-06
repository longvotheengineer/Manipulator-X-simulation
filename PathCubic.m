function trajPVA = PathCubic(theta_A, theta_B, T, N)
    v10 = 0;
    v20 = 0;
    v30 = 0;
    v40 = 0;
    
    theta0 = [theta_A(1) theta_A(2) theta_A(3) theta_A(4)];
    thetaf = [theta_B(1) theta_B(2) theta_B(3) theta_B(4)];

    t = linspace(0, T, N)';      
    trajPVA.t = t;

    theta0 = theta0(:)';
    thetaf = thetaf(:)';

    dtheta = thetaf - theta0;    

    % s(tau) = a0 + a1*tau + a2*tau^2 + a3*tau^3 
    % where tau = t/T; s(0) = 0; s(1) = 1
    % s_dot(0) = 0; s_dot(1) = 1
    a0 = theta0;                        
    a1 = [v10 v20 v30 v40];                        
    a2 = 3*dtheta / T^2;                     
    a3 = -2*dtheta / T^3;                   

    trajPVA.theta = a0 + a1.*t + a2.*(t.^2) + a3.*(t.^3);
    trajPVA.v     = a1 + 2*a2.*t + 3*a3.*(t.^2);
    trajPVA.a     = 2*a2 + 6*a3.*t;
end

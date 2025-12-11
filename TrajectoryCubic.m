function trajPVA = TrajectoryCubic(px, py, pz, T, N, robot_config)
    vx0 = robot_config.vx0;
    vy0 = robot_config.vy0;
    vz0 = robot_config.vz0;
    % vx0 = 0;
    % vy0 = 0;
    % vz0 = 0;

    p0 = [px(1) py(1) pz(1)];
    pf = [px(2) py(2) pz(2)];

    t = linspace(0, T, N)';      
    trajPVA.t = t;

    p0 = p0(:)';     
    pf = pf(:)';

    dp = pf - p0;    

    a0 = p0;                        
    a1 = [vx0 vy0 vz0];                     
    a2 = 3*dp / T^2;                     
    a3 = -2*dp / T^3;                   

    trajPVA.p = a0 + a1.*t + a2.*(t.^2) + a3.*(t.^3);
    trajPVA.v = a1 + 2*a2.*t + 3*a3.*(t.^2);
    trajPVA.a = 2*a2 + 6*a3.*t;

    dist_abs = abs(dp); 
    p_max_val = max(abs([p0; pf])); 
    v_max_val = 1.5 * dist_abs / T;
    a_max_val = 6 * dist_abs / (T^2);
    
    trajPVA.p_max = p_max_val;
    trajPVA.v_max = v_max_val; 
    trajPVA.a_max = a_max_val; 
end
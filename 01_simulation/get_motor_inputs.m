function motor_speeds = get_motor_inputs(t, x, param)
    %% ====================================================================
    % 1. Direct User Input Section (Tuning Device)
    % ====================================================================
    base_w = zeros(4, 1);
    
    if t < 1.0
        % 0 to 1 second: Standby
        base_w(1) = 0; base_w(2) = 0; base_w(3) = 0; base_w(4) = 0;
        
    elseif t >= 1.0 && t < 3.0
        % 1-3 seconds: Vertical rise (all four motors strong!)
        base_w(1) = 250; 
        base_w(2) = 250; 
        base_w(3) = 250; 
        base_w(4) = 250; 
        
    elseif t >= 3.0 && t < 4.0
        % 3-5 seconds: Forward (slow forward, fast backward!)
        base_w(1) = 220; % LF
        base_w(2) = 220; % RF
        base_w(3) = 250; % RB
        base_w(4) = 250; % LB
        
    elseif t >= 4.0 && t < 5.0
        % 5-7 seconds: Backward
        base_w(1) = 250; 
        base_w(2) = 250; 
        base_w(3) = 220; 
        base_w(4) = 220; 

    elseif t >= 5.0 && t < 7.0
        % 5-7 seconds: Rising
        base_w(1) = 250; 
        base_w(2) = 250; 
        base_w(3) = 250; 
        base_w(4) = 250; 
        
    else
        % 7 seconds later: Start off in the air (free fall)
        base_w(1) = 0; base_w(2) = 0; base_w(3) = 0; base_w(4) = 0;
    end

    %% ====================================================================
    % Controller (Stabilization Unit)
    % ====================================================================
    phi_curr   = x(7);  
    p_curr = x(10); 
    theta_curr = x(8);  
    q_curr = x(11); 
    r_curr     = x(12);                 
    
    Kp = 40.0;
    Kd = 15.0; 
    
    roll_corr  = Kp * phi_curr   + Kd * p_curr;
    pitch_corr = Kp * theta_curr + Kd * q_curr;
    yaw_corr   =                       Kd * r_curr;
    
    %% ====================================================================
    % Final Output (user input + controller calibration)
    % ====================================================================
    motor_speeds = zeros(4, 1);
    motor_speeds(1) = base_w(1) - roll_corr - pitch_corr - yaw_corr;
    motor_speeds(2) = base_w(2) + roll_corr - pitch_corr + yaw_corr;
    motor_speeds(3) = base_w(3) + roll_corr + pitch_corr - yaw_corr;
    motor_speeds(4) = base_w(4) - roll_corr + pitch_corr + yaw_corr;
    
    motor_speeds = max(0, motor_speeds);
end

    % % ====================================================================
    % % Movement number
    % % ====================================================================
    % 
    % % Backward
    % base_w(1) = 250; 
    % base_w(2) = 250; 
    % base_w(3) = 220; 
    % base_w(4) = 220;
    % 
    % % Forward
    % base_w(1) = 220; 
    % base_w(2) = 220; 
    % base_w(3) = 250; 
    % base_w(4) = 250;
    % 
    % % Rising
    % base_w(1) = 250; 
    % base_w(2) = 250; 
    % base_w(3) = 250; 
    % base_w(4) = 250;
    % 
    % % Left
    % base_w(1) = 220; 
    % base_w(2) = 250; 
    % base_w(3) = 250; 
    % base_w(4) = 220;
    % 
    % % Right
    % base_w(1) = 250; 
    % base_w(2) = 220; 
    % base_w(3) = 220; 
    % base_w(4) = 250;
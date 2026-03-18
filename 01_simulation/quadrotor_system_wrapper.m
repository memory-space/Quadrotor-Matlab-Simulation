function x_dot = quadrotor_system_wrapper(t, x, param)
    % Call the function created by 'get_motor_inputs' to get the motor speed for the current time (t).
    u_motor = get_motor_inputs(t, x, param);
    
    % Call 'quadrotor_dynamics'.
    % x_dot (state change rate) is calculated.
    x_dot = quadrotor_dynamics(t, x, u_motor, param);
    
    % [Constraint on the ground] It prevents the quadrotor from going down through the floor.
    % NED coordinate system: z>= 0 (ground), x_dot (6) > 0 (downward acceleration)
    if x(3) >= 0 && x_dot(6) > 0
        x_dot(6) = 0; % Vertical Acceleration Blocking
        x_dot(3) = 0; % Vertical Speed Shutdown
    end
end




% Author: Lim Minseok (minseoklim@postech.ac.kr)
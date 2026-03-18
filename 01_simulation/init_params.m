function [param] = init_params()
    %  This function initializes all physical parameters for the quadrotor.
    %  All variable names follow full-name conventions for clarity.
    
    %% Quadrotor Physical Specifications:Crazyflie 2.0 Nano Quadrocopter
    % Defined based on user input: 9.2 x 9.2 x 2.9 (cm)
    param.quadrotor_total_mass = 0.027; % [kg] Total mass of the quadrotor (approx. 27g)
    param.quadrotor_width_x    = 0.092; % [m] Width along the x-axis (92mm)
    param.quadrotor_length_y   = 0.092; % [m] Length along the y-axis(92mm)
    param.quadrotor_height_z   = 0.029; % [m] Height along the z-axis(29mm)
    
    % Distance from center to each motor (Arm length)
    % Assuming a standard X-configuration
    param.arm_length_from_center = 0.046; % [m]
    
    %% Inertia Matrix (J): The mass of the rotating world
    % Moments of inertia defined in MIT Lecture Table 6.1
    % For a basic model, we assume a symmetric rigid body.
    % These values should be updated if a Actual Measurements is available.
    % inertia_moment_x, y: Resistance to the action of tilting the mass quadrotor
    %                      of the rotating world to both sides (Roll) or bending back and forth (Pitch)
    param.inertia_moment_x = 1.395e-5; % [kg*m^2] Moment of inertia about x-axis
    param.inertia_moment_y = 1.436e-5; % [kg*m^2] Moment of inertia about y-axis

    % inertia_moment_z: Resistance to the action of spinning (Yaw) the quadrotor in place
    param.inertia_moment_z = 2.173e-5; % [kg*m^2] Moment of inertia about z-axis

    % Symmetric Structure
    param.inertia_moment_xz = 0;
    param.inertia_moment_xy = 0;
    param.inertia_moment_yz = 0;

    % Full Inertia Matrix J (3x3)
    param.inertia_matrix_J = [param.inertia_moment_x,  -param.inertia_moment_xy, -param.inertia_moment_xz; ...
                             -param.inertia_moment_xy,  param.inertia_moment_y,  -param.inertia_moment_yz; ...
                             -param.inertia_moment_xz, -param.inertia_moment_yz,  param.inertia_moment_z];

    %% Motor and Aerodynamic Constants
    % (rad/s) = (rpm) * (2*pi/60)
    param.thrust_coefficient_cf = 1.28e-8 / ( (2*pi/60)^2 ); % [N/(rad/s)^2] Thrust coefficient (cf)
    param.torque_to_thrust_ratio = 0.005964552; % [m]
    param.torque_drag_coefficient_cd = param.thrust_coefficient_cf * param.torque_to_thrust_ratio; % [Nm/(rad/s)^2] Drag torque coefficient (cd)
    
    %% Environmental Constants
    % Gravity vector in world frame (z-axis points downward)
    param.gravity_acceleration = 9.81; % [m/s^2] Standard gravity
    param.gravity_vector_world = [0; 
                                  0; 
                                 param.gravity_acceleration]; 
    
    %% Simulation World Boundaries
    % Defined space: Width 10m, Depth 5m, Height 15m
    param.world_limit_x = [-5, 5];     % [m] x-range
    param.world_limit_y = [-2.5, 2.5]; % [m] y-range
    param.world_limit_z = [0, 15];     % [m] z-range (Height)
    
    %% Simulation Time Settings
    % Users can adjust the total simulation time here
    param.simulation_start_time = 0;   % [sec]
    param.simulation_end_time = 10;    % [sec] (Adjustable)
    param.simulation_step_size = 0.01; % [sec] 
end




% ========================================================================
% Reference: 
%            Beard, Randal W., and Timothy W. McLain. Small unmanned aircraft: Theory and practice. Princeton university press, 2012.
%            Förster, Julian. "System identification of the crazyflie 2.0 nano quadrocopter." (2015).
%            Luca Carlone. Visual Navigation for Autonomous Vehicles (VNAV): Lecture 6: Quadrotor Dynamics
% Author: Lim Minseok (minseoklim@postech.ac.kr)
% ========================================================================
function x_dot = quadrotor_dynamics(t, x, motor_speeds, param)
    % State vector x: [p_w; v_w; Phi; omega_b] (12x1)
    % p_w: World position [pn, pe, pd]
    % v_w: World velocity [vn, ve, vd] 
    % Phi: Euler angles [phi, theta, psi]
    % omega_b: Body angular rate [p, q, r]
    
    % motor_speeds: [w1, w2, w3, w4]' (4x1) - Angular velocities of rotors [rad/s]
    % w1: Front-Left  (FL) rotor angular velocity (Clockwise)
    % w2: Front-Right (FR) rotor angular velocity (Counter-Clockwise)
    % w3: Back-Right  (BR) rotor angular velocity (Clockwise)
    % w4: Back-Left   (BL) rotor angular velocity (Counter-Clockwise)
    
    %% State Extraction
    velocity_world = x(4:6);      % v^w
    phi = x(7);                   % Euler angle
    theta = x(8); 
    psi = x(9); 
    omega_b = x(10:12);           % w^B

    %% Force and Torque Calculation
    % Extract individual motor speeds [rad/s]
    % Motor numbering per: 1(FL), 2(FR), 3(BR), 4(BL)
    w1 = motor_speeds(1); 
    w2 = motor_speeds(2);
    w3 = motor_speeds(3);
    w4 = motor_speeds(4);

    cf = param.thrust_coefficient_cf;
    cd = param.torque_drag_coefficient_cd;
    L  = param.arm_length_from_center;

    % L_comp (Component Length): Vertical distance when the arm is projected on x and y axes
    L_comp = L * cosd(45); % 45-degree offset components

    % 2.1 Individual Thrust Forces
    % Thrust_f = cf * wi * |wi|
    Thrust_f1 = cf * w1 * abs(w1);
    Thrust_f2 = cf * w2 * abs(w2);
    Thrust_f3 = cf * w3 * abs(w3);
    Thrust_f4 = cf * w4 * abs(w4);

    % u1: Total Thrust Magnitude
    u1 = Thrust_f1 + Thrust_f2 + Thrust_f3 + Thrust_f4;

    % 2.2 Individual Drag Torques
    % tau_drag = cd * wi * |wi|
    % Individual torque considering the direction acting on the gas:
    % Positive torque = 1, Negative torque: -1
    % Clockwise (w1, w3): Positive torque
    % Couter-Clockwise (w2, w4): Negative torque
    tau_drag1 = -cd * w1 * abs(w1);
    tau_drag2 =  cd * w2 * abs(w2);
    tau_drag3 = -cd * w3 * abs(w3);
    tau_drag4 =  cd * w4 * abs(w4);

    % 2.3 Body Torques
    % Roll(Mx): Left-right slope
    %        eq: L_comp*(Left forces) - (Right forces) 
    moment_x = L_comp * ((Thrust_f1 + Thrust_f4) - (Thrust_f2 + Thrust_f3));
    
    % Pitch(My): Back-and-forth slope
    %         eq: L_comp*(Front forces) - (Back forces)
    moment_y = L_comp * ((Thrust_f1 + Thrust_f2) - (Thrust_f3 + Thrust_f4));
    
    % Yaw (Mz): Rotation in place
    %       eq: Sum of aerodynamic drag torques
    moment_z = tau_drag1 + tau_drag2 + tau_drag3 + tau_drag4;

    tau_b = [moment_x; 
             moment_y; 
             moment_z];

    %% Rotation Matrix (R_B^w)
    % Z-Y-X Euler Representation    
    % R_b_w: Rotation matrix from Body frame to World frame
    % R_b_w = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
    %          cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
    %              -sin(theta),                          sin(phi)*cos(theta),                              cos(phi)*cos(theta)];
    R_b_w = body2world_xyz(phi, theta, psi);

    %% Newton-Euler Equation in Matrix Form
    % m: quadrotor total mass
    % J: inertia matrix
    m = param.quadrotor_total_mass;
    J = param.inertia_matrix_J;
   
    % 4.1  External and torque vectors
    % f_w = m * g_w + R * f_b
    f_w = m * param.gravity_vector_world + R_b_w * [0; 0; -u1];
    % Wrench = A six-dimensional vector that combines 'Force' and 'Torque' into one
    Wrench = [ f_w; 
              tau_b];

    % 4.2 Mass and inertia matrix
    M = [m*eye(3),  zeros(3,3);
         zeros(3,3),    J    ];

    % 4.3 Coriolis/Gyroscopic Term
    G = [        zeros(3,1);
         cross(omega_b, J*omega_b) ];

    % 4.4 Solve the determinant: 
    % Equation of Motion: Wrench = M*accels + G
    % We want accels use inv
    accels = inv(M)*(Wrench - G);

    % 4.5 Extract linear acceleration(1:3), angular acceleration components(4:6)
    % v_dot_world: linear acceleration
    % w_dot_body: angular acceleration 
    v_dot_world = accels(1:3); % World linear acceleration (a^w)
    w_dot_body  = accels(4:6); % Body angular acceleration (alpha^B)

    %% Kinematics (Position & Attitude)
    % 5.1 Translational Kinematics(World Frame Position Change Rate): p_dot^w = v^w
    % Change of position in the World Frame = Speed in the world Frame
    p_dot_world = velocity_world;
    
    % 5.2 Rotational Kinematics(Calculating the rate of change in posture): Mapping Euler rates (Phi_dot) to Body rates (omega_b)
    % induction principle: omega_b = W * Phi_dot
    % 1) Roll rate (phi_dot) is already aligned to the x-axis of the gas: [1; 0; 0]
    % 2) Pitch rate (theta_dot) is projected to the gas axis via a roll rotation (phi)
    % 3) Yaw rate (psi_dot) is projected through both pitch (theta) and roll (phi) rotation

    % Defining Rotation Matrix
    Rx = [1,      0,       0;
          0,  cos(phi), sin(phi); 
          0, -sin(phi), cos(phi)]; % Roll rotation
    Ry = [cos(theta), 0, -sin(theta); 
              0,      1,      0; 
          sin(theta), 0,  cos(theta)]; % Pitch rotation

    % Mapping Matrix W Configuration
    % omega_b = col1*phi_dot + col2*theta_dot + col3*psi_dot
    col1 = [1; 0; 0];           % Roll axis
    col2 = Rx * [0; 1; 0];      % Pitch axis by Roll
    col3 = Rx * Ry * [0; 0; 1]; % Yaw axis by Pitch, Roll
    
    % W is 3*3 matrix ex)W = [~, ~, ~];
    W = [col1, col2, col3];          % Forward Mapping (Phi_dot -> omega_b)
    
    % We need Phi_dot : omega_b = W * Phi_dot
    % Use inv
    Phi_dot = inv(W)*omega_b;

    %% Assemble x_dot
    x_dot = [p_dot_world; 
             v_dot_world; 
                 Phi_dot; 
              w_dot_body];
end


% ========================================================================
% [Object of Function] Calculating Quadrotor State Differential Equations
% Based on the Newton-Euler equation, this function is a 'physical engine' 
% that calculates how the quadrotor will change in the next moment (x_dot) 
% given the current state (x) and motor_speeds of the quadrotor.

% Specifically, the physical quantity you want to obtain:
% 1. v_dot_world: Linear acceleration based on the world coordinate system (f = ma)
% 2. w_dot_body : angular acceleration based on the gas coordinate system (tau = J * alpha)
% 3. p_dot_world: Rate of change in location (speed)
% 4. Phi_dot: Rate of change in posture (oiler angle)

% The final result, x_dot (12x1), is delivered to a numerical integrator (Solver) 
% such as ode45, used to determine the drone status (position, speed, posture, etc.) of the next time step.

% Reference: 
%            Beard, Randal W., and Timothy W. McLain. Small unmanned aircraft: Theory and practice. Princeton university press, 2012.
%            Förster, Julian. "System identification of the crazyflie 2.0 nano quadrocopter." (2015).
%            Luca Carlone. Visual Navigation for Autonomous Vehicles (VNAV): Lecture 6: Quadrotor Dynamics
% Author: Lim Minseok (minseoklim@postech.ac.kr)
% ========================================================================
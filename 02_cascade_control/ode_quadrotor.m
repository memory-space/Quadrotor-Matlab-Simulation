function x_dot = ode_quadrotor(t, x, param)
% ========================================================================
% [ODE Function] Quadrotor Cascade Control
%
% Enter:
%   t - Current time [s]
%   x - Status vectors (12×1)
%             x(1:3)   - Position         [North; East; Down]
%             x(4:6)   - Speed            [vN; vE; vD]
%             x(7:9)   - Position angle   [phi; theta; psi]
%             x(10:12) - Angular velocity [p;q;r]
%   param - drone parameter structure
%
% Output:
%   x_dot - state vector differentiation (12×1)
%
% Controller internal status (managed by persistent variables):
%   step           - definitive step counter
%   v_setpoint     - pos_control output target speed       
%   phi_setpoint   - accel_to_att Output Target Roll Angle    
%   theta_setpoint - accel_to_att output target pitch angle    
%   T_setpoint     - accel_to_att output target thrust       
%   omega_setpoint - att_control output target angular velocity     
%   v_I            - vel_control integral term (paragraph I)
%   rate_I         - rate_control integral term (paragraph I)
%   v_prev         - vel_control Previous error (for D)
%
% Loop structure:
%   50  Hz (every 20 steps) : pos_control,  vel_control, accel_to_att
%   250 Hz (every 4 steps)  : att_control
%   1   kHz (every steps)   : rate_control, mixer, dynamics
%
% Frequency separation method:
%   Calculate the determined step number by round(t / dt_fast)
%   Update controller only when current_step > step
%   → code45 Prevent duplicate runs on internal navigation calls
%
% Reference: 
%            Carvalho, Estéban. Neural learning for efficient quadrotor flight control. Diss. Université Grenoble Alpes [2020-....], 2023.
% Author: Lim Minseok (minseoklim@postech.ac.kr)
% ========================================================================

%% persistent declaration
persistent step
persistent v_setpoint phi_sp theta_sp T_sp omega_sp
persistent v_I rate_I v_prev
persistent dbg_t

%% Initialization
if isempty(step)
    step     = 0;
    v_setpoint     = zeros(3,1);
    phi_sp   = 0;
    theta_sp = 0;
    T_sp     = param.quadrotor_total_mass * param.gravity_acceleration;
    omega_sp = zeros(3,1);
    v_I    = zeros(3,1);
    rate_I = zeros(3,1);
    v_prev   = zeros(3,1);
    dbg_t    = -1;
end

%% State extraction
pos     = x(1:3);
vel     = x(4:6);
phi     = x(7);
theta   = x(8);
psi     = x(9);
omega_b = x(10:12);
g       = param.gravity_acceleration;
m       = param.quadrotor_total_mass;
J = param.inertia_matrix_J;
p_ref   = param.p_ref;
psi_ref = param.psi_ref;

%% Check steps
current_step = round(t / param.dt_fast);

if current_step > step
    step = current_step;

    %% [50Hz] pos_control + vel_control + accel_to_att
    if mod(step, 20) == 0

        %% pos_control
        err_p = p_ref - pos;
        v_setpoint  = param.Kp_pos * err_p;

        %% vel_control
        err_v = v_setpoint - vel;
        
        % I Term
        v_I = v_I + err_v * param.dt_slow;
        
        % D Term
        if step <= 20   % Skip term D for the first 50Hz period
            v_der = zeros(3,1);
        else
            v_der = (err_v - v_prev) / param.dt_slow;
        end
        v_prev = err_v;
        
        % PID summation
        a_sp = param.Kp_vel*err_v + param.Ki_vel*v_I + param.Kd_vel*v_der;
        
        % saturate
        a_sp = saturate(a_sp, -5, 5);

        %% accel_to_att
        c_psi    = cos(psi);
        s_psi    = sin(psi);

        % theta_sp = -atan((a_sp(1)*c_psi + a_sp(2)*s_psi) / (g - a_sp(3)));
        % phi_sp   =  atan(cos(theta_sp) * (a_sp(2)*c_psi - a_sp(1)*s_psi) / (g - a_sp(3)));
        % T_sp     = m*(g - a_sp(3)) / (cos(phi_sp)*cos(theta_sp));
        
        phi_sp = -(a_sp(1)*s_psi - a_sp(2)*c_psi)/(g - a_sp(3));
        theta_sp = -(a_sp(1)*c_psi + a_sp(2)*s_psi)/(g - a_sp(3));
        T_sp = m * (g - a_sp(3));

        T_sp     = max(0, T_sp);
    end

    %% [250Hz] att_control
    if mod(step, 4) == 0
        R_sp     = body2world_xyz(phi_sp, theta_sp, psi_ref);
        R_curr   = body2world_xyz(phi, theta, psi);
        R_err    = R_curr' * R_sp;
        ex = (R_err(3,2) - R_err(2,3)) / 2; % ex → Roll  Direction error
        ey = (R_err(1,3) - R_err(3,1)) / 2; % ey → Pitch Direction error
        ez = (R_err(2,1) - R_err(1,2)) / 2; % ez → Yaw   Direction error
        omega_sp = param.Kp_att * [ex; ey; ez];
        omega_sp = max(-5, min(5, omega_sp));
    end

    %% [1kHz] rate_int 
    err_omega = omega_sp - omega_b;
    rate_I  = rate_I + err_omega * param.dt_fast;

end

err_omega = omega_sp - omega_b;
omega_dot_sp    = param.Kp_rate*err_omega + param.Ki_rate*rate_I;
omega_dot_sp = saturate(omega_dot_sp, -4, 4);

%% mixer + dynamics
tau_sp=J*omega_dot_sp + cross(omega_b,J*omega_b);
cf     = param.thrust_coefficient_cf;
cd     = param.torque_drag_coefficient_cd;
L_comp = param.arm_length_from_center * cosd(45);
M_mix  = [cf,          cf,          cf,          cf;
          L_comp*cf,  -L_comp*cf,  -L_comp*cf,   L_comp*cf;
          L_comp*cf,   L_comp*cf,  -L_comp*cf,  -L_comp*cf;
         -cd,          cd,         -cd,           cd];
w_sq         = M_mix \ [T_sp; tau_sp];
motor_speeds = sqrt(max(0, w_sq));

x_dot = quadrotor_dynamics_solution(t, x, motor_speeds, param);

%% Debug output
if t - dbg_t >= 0.5
    fprintf('\n[t=%.2f | step=%d] ─────────────────────────────────\n', t, step);
    fprintf('  pos    : N=%7.3f  E=%7.3f  Alt=%7.3f m\n', pos(1), pos(2), -pos(3));
    fprintf('  vel    : vN=%6.3f  vE=%6.3f  vD=%6.3f m/s\n', vel(1), vel(2), vel(3));
    fprintf('  att    : phi=%6.1f  theta=%6.1f  psi=%6.1f deg\n', rad2deg(phi), rad2deg(theta), rad2deg(psi));
    fprintf('  att_sp : phi=%6.1f  theta=%6.1f deg\n', rad2deg(phi_sp), rad2deg(theta_sp));
    fprintf('  T_sp   : %.4f N\n', T_sp);
    fprintf('  v_sp   : [%.2f  %.2f  %.2f] m/s\n', v_setpoint(1), v_setpoint(2), v_setpoint(3));
    fprintf('  omega_sp: [%.3f  %.3f  %.3f] rad/s\n', omega_sp(1), omega_sp(2), omega_sp(3));
    fprintf('  motor  : [%.1f  %.1f  %.1f  %.1f] rad/s\n', motor_speeds(1), motor_speeds(2), motor_speeds(3), motor_speeds(4));
    fprintf('  v_int  : [%.3f  %.3f  %.3f]\n', v_I(1), v_I(2), v_I(3));
    fprintf('  v_prev : [%.3f  %.3f  %.3f]\n', v_prev(1), v_prev(2), v_prev(3));
    fprintf('  50Hz  실행 횟수 : %d회\n', floor(step/20));
    fprintf('  250Hz 실행 횟수 : %d회\n', floor(step/4));
    fprintf('  1kHz  실행 횟수 : %d회\n', step);

    flags = {};
    if any(isnan(x)) || any(isinf(x))
        flags{end+1} = '!!! NaN/Inf 발산';
    end
    if any(abs(pos) > 100)
        flags{end+1} = sprintf('!!! 위치 발산 (max=%.1f m)', max(abs(pos)));
    end
    if any(motor_speeds > 5000)
        flags{end+1} = sprintf('!!! 모터 폭발 (max=%.1f)', max(motor_speeds));
    end
    if T_sp > 10
        flags{end+1} = sprintf('!!! T_sp 폭발 (%.2f N)', T_sp);
    end
    if max(abs([rad2deg(phi), rad2deg(theta)])) > 60
        flags{end+1} = sprintf('!!! 자세 과다 (phi=%.1f theta=%.1f)', ...
                                rad2deg(phi), rad2deg(theta));
    end
    if any(abs(v_I) > 75)
        flags{end+1} = sprintf('!!! v_int 폭발 (max=%.1f)', max(abs(v_I)));
    end

    if isempty(flags)
        fprintf('  [정상]\n');
    else
        for k = 1:length(flags)
            fprintf('  %s\n', flags{k});
        end
    end

    dbg_t = t;
end

end
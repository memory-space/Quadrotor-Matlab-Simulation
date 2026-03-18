% ========================================================================
% [Main] Quadrotor Cascade Control - Single ode45 Call (Persistent State)
%
% Status Vector 12 Dimensions:
%   x(1:3) - Position [North; East; Down]
%   x(4:6) - Speed [vN; vE; vD]
%   x(7:9) - Position angle [phi; theta; psi]
%   x(10:12) - Angular velocity [p;q;r]
%
% Controller internal status (managed by persistent variables):
%   v_int - vel_control integral term
%   rate_int - rate_control integral term
%   v_prev - vel_control Previous error (for D)
%
% Loop structure:
%   50  Hz : pos_control + vel_control + accel_to_att
%   250 Hz : att_control
%   1   kHz: rate_control + mixer
%
% Author: Lim Minseok (minseoklim@postech.ac.kr)
% ========================================================================
clear all; clc; close all;

%% ── Parameter ─────────────────────────────────────────────────────────
param = init_params();

p_ref   = [5; 5; -5];
psi_ref = deg2rad(90);

%% ── Gain ───────────────────────────────────────────────
param.Kp_pos  = 0.65;
param.Kp_vel  = 1.3;  param.Ki_vel  = 0.002;  param.Kd_vel  = 0.068;
param.Kp_att  = 5;
param.Kp_rate = 6;    param.Ki_rate = 0.9;
param.p_ref   = p_ref;
param.psi_ref = psi_ref;

%% ── Timing ───────────────────────────────────────────────────────────
param.dt_slow = 0.020;
param.dt_mid  = 0.004;
param.dt_fast = 0.001;

%% ── Initial state (12th Dimension) ───────────────────────────────────────────────
x0 = zeros(12, 1);

%% ── ode45 call ──────────────────────────────────────────────
t_span   = 0 : param.dt_fast : 10.0;
ode_opts = odeset('MaxStep', param.dt_fast, 'RelTol', 1e-4, 'AbsTol', 1e-6);

fprintf('Start simulation...\n');
[t_out, x_out] = ode45(@(t,x) ode_quadrotor(t, x, param), t_span, x0, ode_opts);
fprintf('Simulation complete!\n');

%% ── Result plot  ──────────────────────────────────────
figure('Name', 'Position', 'Color', 'w');
subplot(3,1,1); plot(t_out, x_out(:,1)); ylabel('North (m)'); grid on; title('Position');
yline(p_ref(1), 'r--', 'Target North');
subplot(3,1,2); plot(t_out, x_out(:,2)); ylabel('East (m)');  grid on;
yline(p_ref(2), 'r--', 'Target East');
subplot(3,1,3); plot(t_out, -x_out(:,3)); ylabel('Altitude (m)'); xlabel('Time (s)'); grid on;
yline(-p_ref(3), 'r--', 'Target Altitude');

figure('Name', 'Attitude', 'Color', 'w');
subplot(3,1,1); plot(t_out, rad2deg(x_out(:,7))); ylabel('Roll (deg)');  grid on; title('Attitude');
subplot(3,1,2); plot(t_out, rad2deg(x_out(:,8))); ylabel('Pitch (deg)'); grid on;
subplot(3,1,3); plot(t_out, rad2deg(x_out(:,9))); ylabel('Yaw (deg)');   xlabel('Time (s)'); grid on;
yline(rad2deg(psi_ref), 'r--', 'Target Yaw');

%% ── Animation ───────────────────────────────────────────────────────
animate(t_out, x_out(:,1:12), param);
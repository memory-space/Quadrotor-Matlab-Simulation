clear; clc;
close all;

%Environmental settings and parameters load
param = init_params(); % init_params.m must be in the same folder

% Set simulation time and initial state
tspan = 0:0.001:10;  % 0.01 second interval from 0 to 10 seconds
x0 = zeros(12, 1);  % [p;v; Phi; omega] All 0 (completely stopped)

% Numerical integration (ODE45)
% Combine the input with the physical engine via 'quadrotor_system_wrapper'.
[t_out, x_out] = ode45(@(t, x) quadrotor_system_wrapper(t, x, param), tspan, x0);

% animation
animate(t_out, x_out, param);




% Author: Lim Minseok (minseoklim@postech.ac.kr)
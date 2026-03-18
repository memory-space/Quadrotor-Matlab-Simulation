# 02 — Quadrotor Cascade PID Control
> Full cascade control: position → velocity → attitude → rate.  
> Frequency-separated multi-loop architecture with persistent state management.

---

## Overview

This module extends the physics engine from `01_simulation/` with a full cascade PID controller. Rather than manually commanding motor speeds, the drone autonomously tracks a 3D position setpoint using a four-loop cascade architecture.

The controller runs at three distinct frequencies within a single `ode45` call, managed via persistent variables and step counters.

---

## File Structure

| File | Description |
|---|---|
| `main.m` | Entry point — sets gains, reference, runs ODE45, plots results |
| `init_params.m` | Physical parameters (same as 01) |
| `ode_quadrotor.m` | ODE function — cascade controller + dynamics |
| `quadrotor_dynamics.m` | Newton-Euler physics engine |
| `body2world_xyz.m` | ZYX Euler rotation matrix (body → world) |
| `saturate.m` | Element-wise saturation utility |
| `animate.m` | 3D real-time animation |

---

## How to Run

```matlab
>> main
```

Flies to position `[N=5, E=5, Alt=5m]` with yaw target `90°`. Plots position and attitude time histories, then opens 3D animation.

---

## Cascade Control Architecture

```
p_ref
  │
  ▼
[pos_control]   50 Hz    Kp_pos
  │  v_setpoint
  ▼
[vel_control]   50 Hz    Kp_vel, Ki_vel, Kd_vel
  │  a_setpoint
  ▼
[accel_to_att]  50 Hz    geometry
  │  phi_sp, theta_sp, T_sp
  ▼
[att_control]   250 Hz   Kp_att  (SO(3) error)
  │  omega_sp
  ▼
[rate_control]  1 kHz    Kp_rate, Ki_rate
  │  tau_sp
  ▼
[mixer]         1 kHz    allocation matrix
  │  motor_speeds
  ▼
[dynamics]      1 kHz    Newton-Euler
```

---

## Control Frequencies

| Loop | Frequency | Period |
|---|---|---|
| pos_control + vel_control + accel_to_att | 50 Hz | 20 ms |
| att_control | 250 Hz | 4 ms |
| rate_control + mixer + dynamics | 1 kHz | 1 ms |

Frequency separation is implemented using `round(t / dt_fast)` step counting inside the ODE function with persistent variables.

---

## Attitude Error (SO(3))

Attitude control uses a rotation matrix error formulation:

```matlab
R_err = R_curr' * R_sp;
ex = (R_err(3,2) - R_err(2,3)) / 2;   % Roll error
ey = (R_err(1,3) - R_err(3,1)) / 2;   % Pitch error
ez = (R_err(2,1) - R_err(1,2)) / 2;   % Yaw error
omega_sp = Kp_att * [ex; ey; ez];
```

---

## Default Gains

| Gain | Value |
|---|---|
| Kp_pos | 0.65 |
| Kp_vel / Ki_vel / Kd_vel | 1.3 / 0.002 / 0.068 |
| Kp_att | 5 |
| Kp_rate / Ki_rate | 6 / 0.9 |

---

## Reference

- Carvalho, E. (2023). *Neural learning for efficient quadrotor flight control*. Université Grenoble Alpes.

---

## Author

**Lim Minseok** (임민석)  
M.S. Student, Mechanical Engineering, POSTECH  
minseoklim@postech.ac.kr

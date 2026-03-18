# 01 — Quadrotor 6-DOF Dynamics Simulation
> Open-loop motor commands with PD attitude stabilization.  
> Physics engine based on Newton-Euler equations with NED coordinate convention.

---

## Overview

This module implements the core quadrotor dynamics from scratch. Motor commands are defined as time-segmented open-loop step inputs, with a simple PD controller layered on top for attitude stabilization (roll, pitch, yaw).

The goal of this module is to verify the physical model and understand how motor mixing maps to translational and rotational motion.

---

## File Structure

| File | Description |
|---|---|
| `main.m` | Entry point — runs ODE45 and launches animation |
| `init_params.m` | Physical parameters (mass, inertia, motor coefficients) |
| `quadrotor_system_wrapper.m` | Wraps dynamics + motor inputs; enforces ground constraint |
| `quadrotor_dynamics.m` | Newton-Euler physics engine |
| `get_motor_inputs.m` | Open-loop motor commands + PD attitude stabilizer |
| `body2world_xyz.m` | ZYX Euler rotation matrix (body → world) |
| `animate.m` | 3D real-time animation |

---

## How to Run

```matlab
>> main
```

Runs a 10-second simulation and opens the 3D animation window.

---

## State Vector

```
x = [pn, pe, pd,        % NED position [m]
     vn, ve, vd,        % NED velocity [m/s]
     phi, theta, psi,   % Roll, Pitch, Yaw [rad]
     p, q, r]           % Body angular rates [rad/s]
```

---

## Motor Layout (X-Configuration)

```
     Front
  1(FL)  2(FR)
      \/
      /\
  4(BL)  3(BR)
     Back
```

| Motor | Spin | Role |
|---|---|---|
| 1 FL | CW | +Roll, +Pitch, -Yaw |
| 2 FR | CCW | -Roll, +Pitch, +Yaw |
| 3 BR | CW | -Roll, -Pitch, -Yaw |
| 4 BL | CCW | +Roll, -Pitch, +Yaw |

---

## Controller

PD attitude stabilizer in `get_motor_inputs.m`:

```
roll_corr  = Kp * phi   + Kd * p
pitch_corr = Kp * theta + Kd * q
yaw_corr   =              Kd * r
```

Altitude and position are controlled via open-loop motor speed commands defined per time segment.

---

## Author

**Lim Minseok** (임민석)  
M.S. Student, Mechanical Engineering, POSTECH  
minseoklim@postech.ac.kr

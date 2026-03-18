# Quadrotor MATLAB Simulation Series
> MATLAB-based 6-DOF quadrotor dynamics simulation series, progressively building from open-loop control to full cascade PID control.  
> Based on the Crazyflie 2.0 nano quadrotor with NED coordinate convention and ZYX Euler angles.

---

## Repository Structure

```
quadrotor-matlab-simulation/
│
├── 01_simulation/           # Open-loop motor commands + PD attitude stabilizer
├── 02_cascade_control/      # Full cascade PID (pos → vel → att → rate)
└── 03_rotation_matrix/      # ZYX Euler rotation matrix reference implementation
```

---

## Progression

| Step | Folder | Control Method | Description |
|---|---|---|---|
| 1 | `01_simulation/` | Open-loop + PD | Manual motor commands with attitude stabilization |
| 2 | `02_cascade_control/` | Cascade PID | Full position-to-rate cascade controller |
| — | `03_rotation_matrix/` | — | Rotation matrix utility (body ↔ world frame) |

---

## Common Specifications (Crazyflie 2.0)

| Parameter | Value |
|---|---|
| Total mass | 27 g |
| Arm length | 46 mm |
| Coordinate system | NED (North-East-Down) |
| Euler convention | ZYX (Yaw → Pitch → Roll) |

---

## Requirements

- MATLAB R2020a or later
- No additional toolboxes required

---

## References

- Beard, R. W., & McLain, T. W. (2012). *Small Unmanned Aircraft: Theory and Practice*. Princeton University Press.
- Förster, J. (2015). *System identification of the Crazyflie 2.0 nano quadrocopter*.
- Carlone, L. *Visual Navigation for Autonomous Vehicles (VNAV): Lecture 6 — Quadrotor Dynamics*. MIT.
- Carvalho, E. (2023). *Neural learning for efficient quadrotor flight control*. Université Grenoble Alpes.

---

## Author

**Lim Minseok** (임민석)  
M.S. Student, Mechanical Engineering, POSTECH  
minseoklim@postech.ac.kr

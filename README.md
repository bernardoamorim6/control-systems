# Control Systems Projects

A collection of academic projects exploring control theory, from basic PID
feedback to state-space control with state estimation. Built during my
Electronics and Computer Engineering studies at FEUP / KU Leuven.

| Project | Control approach | Sensors / Actuation | Tested on |
|---|---|---|---|
| [Wall-Follower Robot](./wall-follower-robot) | PID + finite-state machine | Ultrasonic distance sensors, DC motors | Real hardware |
| [Rotary Inverted Pendulum](./rotary-inverted-pendulum) | LQR (state-space) | Angle sensors, DC motor | Real hardware, real-time DAQ |
| [Quadcopter Flight Control](./quadcopter-lqr-kalman) | LQR, LQR + integral action, LQG (Kalman filter), pole placement | 12-state nonlinear model | Simulation (Simulink) |

---

## Wall-Follower Robot

Autonomous robot that follows a wall at a set distance using three
ultrasonic sensors and a PID controller, with a finite-state machine
handling mode switching (following left wall, following right wall,
turning). Written in C for Arduino.

- `main.cpp` — main control loop, FSM, PID logic, motor driver interface

## Rotary Inverted Pendulum

Design and real-time implementation of an LQR controller to balance a
rotary inverted pendulum (arm + rod) at its unstable equilibrium point.
Includes derivation of the linear state-space model, controllability/
observability analysis, Q/R tuning, and a filtered derivative estimator
for the angle measurements. Implemented in Simulink and run on a real-time
data-acquisition board at 200 Hz; validated with setpoint-tracking and
disturbance-rejection tests on physical hardware.

- `report.pdf` — full write-up: modeling, controller design, and
  experimental results
- Simulink model files

Team project with Wiktor Prządka.

## Quadcopter Flight Control

Coursework project modeling and controlling a quadcopter's nonlinear
12-state dynamics around hover. Covers linearization, discretization,
and three control strategies: LQR, LQR with integral action (for
zero steady-state error), and LQG control using a Kalman filter for
state estimation from noisy measurements, plus pole placement via
Sylvester's equation for comparison. Robustness tested by adding a
0.1 kg payload disturbance.

**Note:** this project was simulation-only (Simulink); it was not
flown on real hardware.

- `report.pdf` — full write-up: modeling, all four control strategies,
  and simulated results
- Simulink model files

Team project with Wiktor Prządka.

---

## Videos
Video of the robot wall follower: 
https://github.com/user-attachments/assets/f3426d07-0943-4877-a8b9-4efd76e99657

Video of the inverted pendulum:
https://github.com/user-attachments/assets/4ed23280-cda6-41cb-a38d-dce45fdcdf94


### About the reports

Each subfolder's `report.pdf` was originally written for university
coursework and is included as-is for reference. Code and model files
reflect what was actually built and tested at the time.

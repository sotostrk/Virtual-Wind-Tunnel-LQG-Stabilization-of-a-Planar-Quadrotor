# Virtual Wind-Tunnel LQG Stabilization of a Planar Quadrotor

**Discrete-time LQG control of a planar quadrotor in a virtual wind-tunnel environment, featuring Kalman state estimation, LQR feedback, full MATLAB simulation, and a C++ embedded-ready implementation with CSV output.**

---

##  Project Overview

This project simulates the dynamics of a planar quadrotor hovering in the presence of lateral wind disturbances. Using control theory methods we design a **Linear Quadratic Gaussian (LQG)** controller to stabilize the system despite partial noisy measurements and persistent wind input.

The final system uses:
- A discrete-time **LQR** controller to generate optimal thrust and torque commands.
- A **Kalman filter** to estimate the full system state from only partial measurements.
- A **wind disturbance model** simulating sinusoidal gusts from a virtual wind tunnel.
- A full MATLAB simulation, and a matching **C++ implementation** that simulates embedded deployment with CSV export.

---

##  Physics and Modeling

###  Planar Quadrotor System

We consider a 2D quadrotor constrained to planar motion. The system has:

- **6 states**:
  - `x` and `z`: horizontal and vertical positions  
  - `θ`: pitch angle  
  - `ẋ`, `ż`, `θ̇`: corresponding velocities

- **2 control inputs**:
  - `ΔF`: thrust deviation from hover (controls vertical acceleration)  
  - `τ`: torque (controls angular acceleration)

###  Dynamics

Using Newton-Euler mechanics, the nonlinear equations of motion are derived as:
ẍ = ((F₀ + ΔF)/m)·sin(θ) + a_wind(t)
z̈ = ((F₀ + ΔF)/m)·cos(θ) − g
θ̈ = τ/I

 Where:
- `F₀ = m * g` is the hover thrust
- `a_wind(t)` is a time-varying lateral wind disturbance
- `I` is the moment of inertia
These dynamics are converted into first-order ODEs and linearized around hover.

---

##  Methodology

1. **Nonlinear Modeling**  
   - Derive full 6-state dynamics including thrust, gravity, and wind.  
   - Wind acts as an external disturbance on `ẋ`.

2. **Open-Loop Simulation**  
   - Simulate the uncontrolled system.  
   - Result: pitch and position drift; confirms instability.

3. **Linearization**  
   - Around `θ = 0`, `ΔF = 0`, `τ = 0`.  
   - Use Jacobian to obtain `A`, `B`, and `C` matrices.

4. **Discretization**  
   - Use `c2d()` with sampling time `Ts = 0.02s`.  
   - Discrete-time matrices `Ad`, `Bd`, `Cd` now represent the system.

5. **LQR Controller Design**  
   - Design `K` using `dlqr()` with tuned `Q` and `R` weights.  
   - Objective: minimize state deviations and control energy.

6. **Kalman Filter Design**  
   - Design `L` using `dlqe()` assuming process noise `W` and measurement noise `V`.  
   - Estimate `ẋ`, `ż`, and `θ̇` using only noisy measurements of `x`, `z`, and `θ`.

7. **Closed-Loop Simulation**  
   - Run full discrete-time LQG loop in MATLAB.  
   - Inject wind and measurement noise each time step.

8. **C++ Port**  
   - Export all discrete matrices (`Ad`, `Bd`, `Cd`, `Kd`, `Ld`) to `main.cpp`.  
   - Run C++ loop mimicking MATLAB logic.  
   - Log simulation to `data.csv`, plot in MATLAB.

---

##  Open-Loop Behavior

![Open Loop](https://raw.githubusercontent.com/sotostrk/Virtual-Wind-Tunnel-LQG-Stabilization-of-a-Planar-Quadrotor/main/Planar_Quad_Hover_LQG/Figs/Open_Loop_Cont.png)

Without feedback, the quadrotor drifts indefinitely due to:
- Gravity pulling it downward.
- Sinusoidal wind accelerating it laterally.
- Accumulated pitch instability due to unbalanced torques.

This demonstrates that the system is **open-loop unstable** and justifies the need for LQG control.

---

##  Closed-Loop (LQG in MATLAB)

![Closed Loop States](https://raw.githubusercontent.com/sotostrk/Virtual-Wind-Tunnel-LQG-Stabilization-of-a-Planar-Quadrotor/main/Planar_Quad_Hover_LQG/Figs/Closed_Loop_disc_states.png)
![Closed Loop Control](https://raw.githubusercontent.com/sotostrk/Virtual-Wind-Tunnel-LQG-Stabilization-of-a-Planar-Quadrotor/main/Planar_Quad_Hover_LQG/Figs/Closed_Loop_disc_contrl.png)

- The LQR controller stabilizes all 6 states to hover.  
- Kalman estimates (dashed) track true states (solid) with minimal lag.  
- Control inputs remain smooth and bounded.  
- Disturbances are rejected effectively, proving the estimator + controller combination works.

---

##  C++ Port Results

![C++ Results](https://raw.githubusercontent.com/sotostrk/Virtual-Wind-Tunnel-LQG-Stabilization-of-a-Planar-Quadrotor/main/Planar_Quad_Hover_LQG/Figs/CL_cpp_results.png)

- The C++ simulation replicates MATLAB behavior with high fidelity.  
- True and estimated states are nearly indistinguishable.  
- Demonstrates that the LQG algorithm is ready for real-time deployment.

---

## How to Run This Project

### MATLAB

cd MATLAB
run('run_closed_loop_lqg_quad_discrete.m')

### C++
cd cpp
g++ -std=c++17 main.cpp -o quad_lqg_sim
./quad_lqg_sim

## Future Extensions

- Add actuator dynamics and delays in Simulink

- Extend to 3D quadrotor simulation

- Use Extended Kalman Filter (EKF) for nonlinear estimation

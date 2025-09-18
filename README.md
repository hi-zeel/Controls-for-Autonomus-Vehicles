# 🚗 Control Systems for Autonomous Driving

This repository showcases my **learning journey in control systems** applied to **Autonomous Vehicles (ADAS)**.  
I started from **basic control theory** and advanced step by step to implement **PID, MPC, and Kalman filter-based strategies**, eventually integrating them into **dual-axle steering and lane-keeping tasks in CARLA**.

---

## 📌 Motivation
Autonomous driving requires **robust and adaptive control strategies**.  
Through this repository, I demonstrate how I:
- Learned control fundamentals 📘
- Applied them in simulation (CARLA, Python, Jupyter) ⚙️
- Built and compared real control strategies (PID vs MPC, Kalman filter integration) 🏎️
- Moved towards **full ADAS integration**.

---

## 🚀 Repository Structure

### 1. Foundations
- `Control_Systems.ipynb` → basics of control theory.
- `KalmanFilter_SideslipEstimation.ipynb` → estimation of vehicle sideslip using KF.

### 2. PID Control
- `Dual-Axle PID.py` → PID controller for dual-axle steering.  
- `Dual-Axle PID with Speed-Based Strategy in CARLA.py` → adaptive PID with speed-based adjustments in CARLA.  
- `Lane-Keeping PID.py` → lane-keeping using PID.  
- **Results:**  
  ![PID Lane-Keeping](control/PID Lane-Keeping Control.png)

### 3. Model Predictive Control (MPC)
- `MPC_Basics.ipynb` → introduction to MPC.  
- `MPC_double_integrator.ipynb` → simple double-integrator MPC.  
- `MPC_Applications.ipynb` → applying MPC to vehicle control.  
- `MPC_Advanced.ipynb` → advanced scenarios and tuning.  
- `PID_vs_MPC_LaneKeeping.ipynb` → performance comparison of PID vs MPC.

### 4. Kalman Filter + Control Fusion
- `KF_DualAxle_PID.ipynb` → combining Kalman filter with dual-axle PID.  
- `KF_DualAxle_PID_HUD.ipynb` → estimation-driven control with HUD visualizations.  

### 5. Full ADAS Integration
- `ADAS_Full_Integrated_Notebook.ipynb` → end-to-end integration.  
- `ADAS_Interactive_SuperLab.ipynb` → interactive lab for experimenting with ADAS strategies.  
- `Extended Code for Comparison.py` → comparing different approaches.  
- `ADAS_Guide.txt` → personal notes/guide.

---

## 📈 Skills Demonstrated
- Control theory (PID, MPC, state estimation).
- Implementation in Python, Jupyter, CARLA.
- Dual-axle steering and lane-keeping.  
- Kalman filter integration for robustness.  
- Performance evaluation and comparison (PID vs MPC).  
- Full ADAS integration workflow.

---

## 🛠️ Tech Stack
- **Python** (NumPy, SciPy, control libraries)
- **CARLA Simulator**
- **Jupyter Notebooks**
- **Matplotlib / Plotly** for visualization

---

## 🌟 Why This Matters
This work demonstrates my **progression from fundamentals to advanced control systems** in the context of **autonomous driving**.  
It highlights not just theoretical learning, but also **hands-on implementation, simulation, and integration** — key skills for modern ADAS and self-driving vehicle development.

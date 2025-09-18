import numpy as np
import matplotlib.pyplot as plt

# Simulation parameters
dt = 0.1
time = np.arange(0, 20, dt)

# Vehicle parameters
Lf, Lr = 1.2, 1.3
L = Lf + Lr
v = 10.0  # m/s (try changing this to 5 for low speed)

# PID gains for front control
Kp_f, Ki_f, Kd_f = 0.2, 0.01, 0.4

# State variables
y1, yaw1 = 2.0, 0.0   # Front-only
y2, yaw2 = 2.0, 0.0   # Dual-axle
cte1, cte2 = y1, y2
int1, int2 = 0, 0
prev1, prev2 = cte1, cte2

history_cte_front, history_cte_dual = [], []

for t in time:
    # --- Front-only PID ---
    int1 += cte1 * dt
    der1 = (cte1 - prev1) / dt
    delta_f1 = Kp_f*cte1 + Ki_f*int1 + Kd_f*der1
    delta_r1 = 0

    yaw_rate1 = (v/L) * (np.tan(delta_f1) - np.tan(delta_r1))
    yaw1 += yaw_rate1 * dt
    y1 += v * np.sin(yaw1) * dt

    prev1 = cte1
    cte1 = y1
    history_cte_front.append(cte1)

    # --- Dual-Axle PID with speed-based δr strategy ---
    int2 += cte2 * dt
    der2 = (cte2 - prev2) / dt
    delta_f2 = Kp_f*cte2 + Ki_f*int2 + Kd_f*der2

    # Speed-based rear steering strategy
    if v < 8:  
        k = -0.5   # low speed → rear steers opposite
    else:        
        k = +0.3   # high speed → rear steers same direction
    delta_r2 = k * delta_f2

    yaw_rate2 = (v/L) * (np.tan(delta_f2) - np.tan(delta_r2))
    yaw2 += yaw_rate2 * dt
    y2 += v * np.sin(yaw2) * dt

    prev2 = cte2
    cte2 = y2
    history_cte_dual.append(cte2)

# Plot comparison
plt.figure(figsize=(10,5))
plt.plot(time, history_cte_front, label="Front-only PID")
plt.plot(time, history_cte_dual, label="Dual-Axle PID (with speed strategy)")
plt.xlabel("Time (s)")
plt.ylabel("Cross-Track Error (m)")
plt.title("Improved Dual-Axle PID: Speed-based δr Strategy")
plt.legend()
plt.grid(True)
plt.show()

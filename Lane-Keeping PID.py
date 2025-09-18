import numpy as np
import matplotlib.pyplot as plt

# Simulation parameters
dt = 0.1  # time step
time = np.arange(0, 20, dt)

# Vehicle parameters
Lf, Lr = 1.2, 1.3
L = Lf + Lr
v = 10.0  # m/s

# PID gains
Kp, Ki, Kd = 0.2, 0.01, 0.4

# State variables
y = 2.0       # initial lateral offset (meters)
yaw = 0.0     # initial heading (rad)
cte = y       # cross-track error
integral = 0  # integral of error
prev_cte = cte

history_y, history_cte = [], []

for t in time:
    # PID control
    integral += cte * dt
    derivative = (cte - prev_cte) / dt
    delta_f = Kp*cte + Ki*integral + Kd*derivative
    
    # Bicycle model update
    yaw_rate = (v/L) * np.tan(delta_f)
    yaw += yaw_rate * dt
    y += v * np.sin(yaw) * dt
    
    prev_cte = cte
    cte = y  # desired path is y=0
    
    # Save history
    history_y.append(y)
    history_cte.append(cte)

# Plot results
plt.figure(figsize=(10,5))
plt.plot(time, history_cte, label="Cross-Track Error")
plt.xlabel("Time (s)")
plt.ylabel("Error (m)")
plt.title("PID Lane-Keeping Control")
plt.grid(True)
plt.legend()
plt.show()

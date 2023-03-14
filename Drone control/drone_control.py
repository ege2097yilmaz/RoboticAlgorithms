import time
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# PID controller gains
kp = 0.8
ki = 0.05
kd = 0.0012

# Setpoint values
setpoint_x = 0.0
setpoint_y = 0.0
setpoint_z = 10.0

# Initial drone position
position_x = 0.0
position_y = 0.0
position_z = 0.0

# PID error terms
error_x = 0.0
error_y = 0.0
error_z = 0.0
error_sum_x = 0.0
error_sum_y = 0.0
error_sum_z = 0.0
error_diff_x = 0.0
error_diff_y = 0.0
error_diff_z = 0.0

# Time variables
t0 = time.time()
dt = 0.01
times = []
positions = []

# Simulation parameters
velocity_x = 1.0
velocity_y = -0.5
velocity_z = 2.0
noise_sigma = 0.1

prev_error_x = velocity_x
prev_error_y = velocity_y
prev_error_z = velocity_z

# Simulate sensor data
position_x += velocity_x * dt + np.random.normal(scale=noise_sigma)
position_y += velocity_y * dt + np.random.normal(scale=noise_sigma)
position_z += velocity_z * dt + np.random.normal(scale=noise_sigma)

while True:
    sensor_data = (position_x, position_y, position_z)

    # Calculate errors
    error_x = setpoint_x - position_x
    error_y = setpoint_y - position_y
    error_z = setpoint_z - position_z
    error_sum_x += error_x * dt
    error_sum_y += error_y * dt
    error_sum_z += error_z * dt
    error_diff_x = (error_x - prev_error_x) / dt
    error_diff_y = (error_y - prev_error_y) / dt
    error_diff_z = (error_z - prev_error_z) / dt

    # Calculate control signals using PID controller
    control_signal_x = kp * error_x + ki * error_sum_x + kd * error_diff_x
    control_signal_y = kp * error_y + ki * error_sum_y + kd * error_diff_y
    control_signal_z = kp * error_z + ki * error_sum_z + kd * error_diff_z

    # Simulate drone motor outputs
    motor_output_x = control_signal_x / 2.0
    motor_output_y = control_signal_y / 2.0
    motor_output_z = control_signal_z / 2.0

    # Update drone position based on motor outputs
    position_x += motor_output_x * dt + np.random.normal(scale=noise_sigma)
    position_y += motor_output_y * dt + np.random.normal(scale=noise_sigma)
    position_z += motor_output_z * dt + np.random.normal(scale=noise_sigma)

    # Update previous errors
    prev_error_x = error_x
    prev_error_y = error_y
    prev_error_z = error_z

    # Record time and position data
    times.append(time.time() - t0)
    positions.append((position_x, position_y, position_z))

    # Pause for specified time interval
    time.sleep(dt)

    # Check if simulation has ended
    if times[-1] >= 15.0:
        break

print(positions[-1])


# Plot 3D trajectory of drone's position
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_zlim(0, 20)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.plot([p[0] for p in positions], [p[1] for p in positions], [p[2] for p in positions])
plt.show()

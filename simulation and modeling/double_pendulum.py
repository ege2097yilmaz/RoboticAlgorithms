import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Define the parameters of the double pendulum
l1 = 1.0  # length of the first pendulum arm
l2 = 1.0  # length of the second pendulum arm
m1 = 1.0  # mass of the first pendulum
m2 = 1.0  # mass of the second pendulum
g = 9.81  # gravitational acceleration

# Define the initial conditions
theta1 = np.pi/2  # initial angle of the first pendulum
theta2 = np.pi/4  # initial angle of the second pendulum
omega1 = 0.0  # initial angular velocity of the first pendulum
omega2 = 0.0  # initial angular velocity of the second pendulum

# Define the time step and the number of iterations
dt = 0.01  # time step
nsteps = 10000  # number of iterations

# Define the arrays to store the motion
t = np.zeros(nsteps)  # time array
theta1_array = np.zeros(nsteps)  # array for the angle of the first pendulum
theta2_array = np.zeros(nsteps)  # array for the angle of the second pendulum

omega1_array = np.zeros(nsteps)  # array for the angular velocity of the first pendulum
omega2_array = np.zeros(nsteps)  # array for the angular velocity of the second pendulum
x1_array = np.zeros(nsteps)  # array for the x-coordinate of the first pendulum
y1_array = np.zeros(nsteps)  # array for the y-coordinate of the first pendulum
x2_array = np.zeros(nsteps)  # array for the x-coordinate of the second pendulum
y2_array = np.zeros(nsteps)  # array for the y-coordinate of the second pendulum

# Perform the simulation using the Euler method
for i in range(nsteps):
    # Compute the accelerations
    alpha1 = -g*(2*m1 + m2)*np.sin(theta1) - m2*g*np.sin(theta1 - 2*theta2) - 2*np.sin(theta1 - theta2)*m2*(omega2**2*l2 + omega1**2*l1*np.cos(theta1 - theta2))
    alpha1 /= l1*(2*m1 + m2 - m2*np.cos(2*theta1 - 2*theta2))
    
    alpha2 = 2*np.sin(theta1 - theta2)*(omega1**2*l1*(m1 + m2) + g*(m1 + m2)*np.cos(theta1) + omega2**2*l2*m2*np.cos(theta1 - theta2))
    alpha2 /= l2*(2*m1 + m2 - m2*np.cos(2*theta1 - 2*theta2))
    
    # Update the velocities
    omega1 += alpha1*dt
    omega2 += alpha2*dt
    
    # Update the angles
    theta1 += omega1*dt
    theta2 += omega2*dt
    
    # Compute the coordinates of the pendulums
    x1 = l1*np.sin(theta1)
    y1 = -l1*np.cos(theta1)
    
    x2 = x1 + l2*np.sin(theta2)
    y2 = y1 - l2*np.cos(theta2)
    
    # Store the motion
    t[i] = i*dt
    theta1_array[i] = theta1
    theta2_array[i] = theta2
    x1_array[i] = x1
    y1_array[i] = y1
    x2_array[i] = x2
    y2_array[i] = y2


fig, axs = plt.subplots(2, 2, figsize=(10, 10))

fig.suptitle('Double Pendulum Motion')

axs[0, 0].plot(t, theta1_array, label='Pendulum 1')
axs[0, 0].plot(t, theta2_array, label='Pendulum 2')
axs[0, 0].set_xlabel('Time (s)')
axs[0, 0].set_ylabel('Angle (rad)')
axs[0, 0].legend()

axs[0, 1].plot(theta1_array, omega1_array, label='Pendulum 1')
axs[0, 1].plot(theta2_array, omega2_array, label='Pendulum 2')
axs[0, 1].set_xlabel('Angle (rad)')
axs[0, 1].set_ylabel('Angular Velocity (rad/s)')
axs[0, 1].legend()

axs[1, 0].plot(x1_array, y1_array, label='Pendulum 1')
axs[1, 0].plot(x2_array, y2_array, label='Pendulum 2')
axs[1, 0].set_xlabel('x (m)')
axs[1, 0].set_ylabel('y (m)')
axs[1, 0].legend()

axs[1, 1].set_xlim(-2, 2)
axs[1, 1].set_ylim(-2, 2)

line1, = axs[1, 1].plot([], [], 'o-', lw=2)
line2, = axs[1, 1].plot([], [], 'o-', lw=2)

def init():
    line1.set_data([], [])
    line2.set_data([], [])
    return line1, line2

def animate(i):
    xdata = [0, x1_array[i], x2_array[i]]
    ydata = [0, y1_array[i], y2_array[i]]
    line1.set_data([xdata[0], xdata[1]], [ydata[0], ydata[1]])
    line2.set_data([xdata[1], xdata[2]], [ydata[1], ydata[2]])
    return line1, line2

anim = animation.FuncAnimation(fig, animate, init_func=init, frames=nsteps, interval=dt*1000, blit=True, repeat=False)

plt.show()
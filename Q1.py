import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Link lengths
L1 = L2 = L3 = 1.0 

radius = 1.5  
center_x = center_y = 0 

# Desired circle trajectory
t = np.linspace(0, 2 * np.pi, 500)  
circle_x = center_x + radius * np.cos(t)
circle_y = center_y + radius * np.sin(t)

# Forward kinematics function
def forward_kinematics(theta1, theta2, theta3):
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2) + L3 * np.cos(theta1 + theta2 + theta3)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2) + L3 * np.sin(theta1 + theta2 + theta3)
    return x, y

# Numerical inverse kinematics function (simple iterative method)
def numerical_inverse_kinematics(x, y, theta1=0.1, theta2=0.1, theta3=0.1, iterations=1000, tolerance=1e-6):
    for _ in range(iterations):
        x_calc, y_calc = forward_kinematics(theta1, theta2, theta3)
        error_x = x - x_calc
        error_y = y - y_calc

        if abs(error_x) < tolerance and abs(error_y) < tolerance:
            break

        J11 = -L1 * np.sin(theta1) - L2 * np.sin(theta1 + theta2) - L3 * np.sin(theta1 + theta2 + theta3)
        J12 = -L2 * np.sin(theta1 + theta2) - L3 * np.sin(theta1 + theta2 + theta3)
        J13 = -L3 * np.sin(theta1 + theta2 + theta3)
        J21 = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2) + L3 * np.cos(theta1 + theta2 + theta3)
        J22 = L2 * np.cos(theta1 + theta2) + L3 * np.cos(theta1 + theta2 + theta3)
        J23 = L3 * np.cos(theta1 + theta2 + theta3)

        det_J = J11 * J22 - J12 * J21
        if abs(det_J) < 1e-10:  # Checking for singularity
            break
        
        J = np.array([[J11, J12, J13], [J21, J22, J23]])
        delta_theta = np.linalg.pinv(J) @ np.array([error_x, error_y])  

        theta1 += delta_theta[0]
        theta2 += delta_theta[1]
        theta3 += delta_theta[2]

    return theta1, theta2, theta3


fig, ax = plt.subplots()
line, = ax.plot([], [], 'bo-')
link1, = ax.plot([], [], 'r-')
link2, = ax.plot([], [], 'g-')
ax.set_xlim(-3, 3) 
ax.set_ylim(-3, 3) 

def update(frame):
    x_target, y_target = circle_x[frame], circle_y[frame]
    theta1, theta2, theta3 = numerical_inverse_kinematics(x_target, y_target)
    x, y = forward_kinematics(theta1, theta2, theta3)
    line.set_data([0, L1 * np.cos(theta1), L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2), x],
                  [0, L1 * np.sin(theta1), L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2), y])
    link1.set_data([0, L1 * np.cos(theta1)], [0, L1 * np.sin(theta1)])
    link2.set_data([L1 * np.cos(theta1), L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)],
                   [L1 * np.sin(theta1), L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)])
    return line, link1, link2

# Create animation
ani = FuncAnimation(fig, update, frames=len(circle_x), interval=50, blit=True)
plt.title('3R Planar Manipulator Tracing a Circle with Numerical Inverse Kinematics')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.grid(True)

# Display the animation
plt.show()

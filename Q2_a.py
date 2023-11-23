import numpy as np
import matplotlib.pyplot as plt


# Lengths of the robot arm links
L1 = 0.25  # Length of link 1
L2 = 0.25 # Length of link 2
L3 = 0.25  # Length of link 3

# Function for inverse kinematics

'''def inverse_kinematics(x, y, z):
    s23 = np.cos(theta3)*np.sin(theta2) + np.cos(theta2)*np.sin(theta3)
    c23 = np.cos(theta2)*np.cos(theta3) - np.sin(theta2)*np.sin(theta3)
    
    c1= x/(-L3*s23 -L2*np.sin(theta2))
    s1= y/(-L3*s23 -L2*np.sin(theta2))
    theta1 = np.arctan2(np.sin(theta2), np.cos(theta2))

    x_dot = -L3*s23 -L2*np.sin(theta2)
    y_dot = -L3*s23 -L2*np.sin(theta2)
    z_dot = L3*c23 + L2*np.cos(theta2)

    c3 = (y_dot**2 +z_dot**2 - L3**2 - L2**2)/(2*L3*L2)
    s3 = np.sqrt(1-c3**2)
    theta3 = np.arctan2(s3, c3)
    
    A = np.array([[-L3*np.sin(theta1)*np.cos(theta3)-L2*np.sin(theta1), -L3*np.sin(theta1)*np.sin(theta3)], [-L3*np.sin(theta1)*np.cos(theta3) -L2*np.sin(theta1),-L3*np.sin(theta1)*np.sin(theta3)]])
    b = np.array([y, x])
    sol1,sol2 = np.linalg.solve(A, b)
    theta2 = np.arctan2(sol1, sol2)
                
    return theta1, theta2, theta3'''

def inverse_kinematics(x, y, z):
    theta1 = np.arctan2(y, x)
    r = np.sqrt(x**2 + y**2)
    D = np.sqrt((z-L1)**2 + r**2)
    theta3 = np.arccos((-L2**2 -L3**2 +D**2)/(2*L2*L3))
    theta2 = np.arctan2(r,z-L1) - np.arctan2(L2+L3*np.cos(theta3), L3*np.sin(theta3))
    return theta1, theta2, theta3

# Function for forward kinematics
def forward_kinematics(theta1, theta2, theta3): #from kinematic analysis of 3R articulated bot
    
    x = (L2*np.cos(theta2)+L3*np.cos(theta2+theta3))*np.sin(theta1)
    y = (L2*np.cos(theta2)+L3*np.cos(theta2+theta3))*np.cos(theta1)
    z = L2*np.sin(theta2)+L3*np.sin(theta2+theta3) +L1
    return x, y, z

# Example usage for forward kinematics
angles = np.degrees([9.46, 34.46,32.33])  # Joint angles in degrees
x, y, z = forward_kinematics(*angles)
print(f"Forward Kinematics Result - End-effector Position: X={x:.2f}m, Y={y:.2f}m, Z={z:.2f}m")


# Example usage for inverse kinematics
target_position = [0.26, 0.02, 0.58]  # Target end-effector position [X, Y, Z]
theta1, theta2, theta3 = inverse_kinematics(*target_position)
print(f"Inverse Kinematics Result - Joint Angles: Theta1={np.degrees(theta1):.2f}°, Theta2={np.degrees(theta2):.2f}°, Theta3={np.degrees(theta3):.2f}°")

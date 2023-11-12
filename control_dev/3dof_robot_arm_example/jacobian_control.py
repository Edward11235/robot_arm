import numpy as np

# Arm segment lengths
L1 = 1.0  # Length from base to first joint
L2 = 1.0  # Length from first joint to second joint
L3 = 1.0  # Length from second joint to end-effector (gripper)

def forward_kinematics(theta1, theta2, theta3):
    """
    Calculate the end-effector position for given joint angles in 3D.
    """
    # Compute the position of the end-effector
    x = (L1 + L2 * np.cos(theta2) + L3 * np.cos(theta2 + theta3)) * np.cos(theta1)
    y = (L1 + L2 * np.cos(theta2) + L3 * np.cos(theta2 + theta3)) * np.sin(theta1)
    z = L2 * np.sin(theta2) + L3 * np.sin(theta2 + theta3)
    return np.array([x, y, z])

def jacobian(theta1, theta2, theta3):
    """
    Compute the Jacobian for the robot arm in 3D.
    """
    J = np.array([
        [-(L2 * np.cos(theta2) + L3 * np.cos(theta2 + theta3)) * np.sin(theta1), -L2 * np.sin(theta2) * np.cos(theta1) - L3 * np.sin(theta2 + theta3) * np.cos(theta1), -L3 * np.sin(theta2 + theta3) * np.cos(theta1)],
        [(L2 * np.cos(theta2) + L3 * np.cos(theta2 + theta3)) * np.cos(theta1), -L2 * np.sin(theta2) * np.sin(theta1) - L3 * np.sin(theta2 + theta3) * np.sin(theta1), -L3 * np.sin(theta2 + theta3) * np.sin(theta1)],
        [0, L2 * np.cos(theta2) + L3 * np.cos(theta2 + theta3), L3 * np.cos(theta2 + theta3)]
    ])
    return J

def inverse_kinematics(target_pos, initial_theta):
    """
    Use the Jacobian inverse to iteratively find joint angles that achieve the target position in 3D.
    """
    theta = np.array(initial_theta)
    for i in range(100):  # Number of iterations can be adjusted
        pos = forward_kinematics(theta[0], theta[1], theta[2])
        error = target_pos - pos
        if np.linalg.norm(error) < 0.001:  # Convergence threshold
            break
        
        J = jacobian(theta[0], theta[1], theta[2])
        J_inv = np.linalg.pinv(J)  # Compute the pseudo-inverse of the Jacobian
        dtheta = J_inv.dot(error)  # Calculate change in joint angles
        
        theta += dtheta  # Update joint angles
    
    return theta

# Example usage:
target_position = np.array([0.5, 0.5, 0.5])  # Replace with your target position in (x, y, z)
initial_angles = np.array([0.0, 0.0, 0.0])  # Replace with your initial joint angles
angles = inverse_kinematics(target_position, initial_angles)
print(f"Angles to reach target (in radian): {angles}")
# Convert radians to degrees
angles_degrees = np.degrees(angles)
# Normalize angles to range [0, 360)
normalized_angles = np.mod(angles_degrees, 360)
print(f"Angles to reach target (in degree): {normalized_angles}")
final_position = forward_kinematics(angles[0], angles[1], angles[2])
print(f"Final position is: {final_position}")
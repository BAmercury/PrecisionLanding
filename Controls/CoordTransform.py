import numpy as np
import math

# Give euler angles as a python list and return numpy matrix for the rotation matrix
def generate_rotation_matrix(euler_angles):
    # Using 3-2-1 Euler (Yaw, Pitch, Roll)
    sine_pitch = math.sin(euler_angles[0])
    cos_pitch = math.cos(euler_angles[0])
    sine_roll = math.sin(euler_angles[1])
    cos_roll = math.cos(euler_angles[1])
    sine_yaw = math.sin(euler_angles[2])
    cos_yaw = math.cos(euler_angles[2])

    # Form the sequential matrices
    L1 = np.array([
        [1, 0, 0],
        [0, cos_pitch, sine_pitch],
        [0, -sine_pitch, cos_pitch]
    ])
    L2 = np.array([
        [cos_roll, 0, -sine_roll],
        [0, 1, 0],
        [sine_roll, 0, cos_roll]
    ])
    L3 = np.array([
        [cos_yaw, sine_yaw, 0],
        [-sine_yaw, cos_yaw, 0],
        [0, 0, 1]
    ])

    M = L1 * L2 * L3
    return M

if __name__ == "__main__":
    orient = [0, 0, 3.14159] # Radians, new vehicle orientation from origin
    M = generate_rotation_matrix(orient)
    print(M)

    vel = [5, 0, 0]
    velArray = np.asarray(vel)
    velB = M.dot(velArray)

    print(velB)

import numpy as np
import math


def generate_rotation_matrix(yaw_angle):
    sine_yaw = math.sin(yaw_angle)
    cos_yaw = math.cos(yaw_angle)

    # Form the sequential matrices
    M = np.array([
        [cos_yaw, sine_yaw],
        [-sine_yaw, cos_yaw]
    ])
    return M
if __name__ == "__main__":
    orient = 3.14159 # Radians, new vehicle orientation from origin
    M = generate_rotation_matrix(orient)
    print(M)

    vel = [5, 0] # Body Frame
    M_inv = np.linalg.inv(M)
    velR = M_inv.dot(vel)
    print("Original Body Vector: %s" % vel)
    print("From body to Reference: %s" % np.round(velR))
    # Now convert back to original
    velB = M.dot(velR)
    print("Reference back to body: %s" % np.round(velB))

import control
import numpy as np

# Open loop model
Alat = np.array([
    [-0.1180, 32.033, 0, 0],
    [0, -6.411, 1, 0],
    [0, -20.639, 0, 1],
    [0, -1.235, 0, 0]
])

Blat = np.array([
        [0], 
        [0.0039974],
        [0.016275],
        [0.0040072]
])

Clat = np.array([0.3048, 0, 0, 0])
Dlat = 0

sys_lat = control.ss(Alat, Blat, Clat, Dlat)
print(sys_lat)
# Test if system is controllable
ContMatrix = control.ctrb(Alat, Blat)
print("Rank of Controllability Matrix: %s" % np.linalg.matrix_rank(ContMatrix))
# Test if system is observable
ObsMatrix = control.obsv(Alat, Clat)
print("Rank of Observability Matrix: %s" % np.linalg.matrix_rank(ObsMatrix))
# Fully controllable, fully observable. So we can employ state feedback
K = control.acker(Alat, Blat, [-10])
print(K)
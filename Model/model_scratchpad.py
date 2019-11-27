import control
import numpy as np
from matplotlib import pyplot as plt

# Open loop model
# Quadrotor with aerodynamic properties and ACAH
# Yande Lui formulation:
# https://etda.libraries.psu.edu/catalog/13071yxl5197
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


sys_tf_lat = control.ss2tf(sys_lat)
lat_delay = 0.156
delay_NUM, delay_DEN = control.pade(lat_delay, 4)
sys_tf_lat_delay = control.tf(delay_NUM, delay_DEN)
sys_tf_lat = sys_tf_lat *sys_tf_lat_delay 


T, yout = control.step_response(sys_tf_lat*17.3333)
plt.plot(T, yout)
plt.show()





# Fully controllable, fully observable. So we can employ state feedback
# = control.acker(Alat, Blat, [-10])
#print(K)
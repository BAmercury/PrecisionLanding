# Testing command filter out to see if it works as expected
import time
import numpy as np
from matplotlib import pyplot as plt
# Declare Variables
update_rate = 0.2
start_time = 0
forward_f = 0.0
RC = 0.7


# Generate a step fnctoin
time_vec = np.arange(start=0, stop=10, step=update_rate)
step_vec = np.arange(start=-5, stop=5, step=update_rate)
step = np.heaviside(step_vec, 3)
filt_command = np.zeros(step.shape[0])
accel_filt_command = np.zeros(step.shape[0])


i = 0
for time_step in time_vec:
    # Get the input
    pilot_input = step[i]
    # Low pass filter
    dt = time_step - start_time
    start_time = time_step
    a = dt/(RC+dt)
    pilot_input_forward_f = (a * forward_f) + ((1-a) * float(pilot_input))
    print(pilot_input_forward_f)
    # Differentiate
    fwd_accel = (pilot_input_forward_f - forward_f) / dt
    forward_f = pilot_input_forward_f

    # Save output
    filt_command[i] = pilot_input_forward_f
    accel_filt_command[i] = fwd_accel
    i = i + 1

fig1, ax1 = plt.subplots()
ax1.plot(step)
ax1.plot(filt_command)
ax1.set_title("Filtered Velocity Command")

fig2, ax2 = plt.subplots() # two axes on figure
ax2.plot(accel_filt_command)
ax2.set_title("Filtered Velocity Integrated Command")
plt.show()

    




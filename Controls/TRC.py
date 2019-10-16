# Translational Rate Control

# Using Dronekit to send commands to drone
from dronekit import connect, VehicleMode
import pygame
import time
import sys
import math
import json
from simple_pid import PID
import numpy as np
from CoordTransform import generate_rotation_matrix
# Declare Variables
pilot_joy_enable = False
joystick_inputs = [0, 0, 1500, 1500]
btn_inputs = [1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]
# Declare boolean for Arm toggle and TRC Toggle
toggle_trc = 0
toggle_arm = 0
# TRC Global variables
start_time = 0 # seconds
forward_f = 0
lateral_f = 0
RC = 0.7 # Low pass filter RC time constant
Kp_lat = 1.351
Ki_lat = 0.0095
Kp_long = -1.957
Ki_long = -0.138
forward_integral = 0
lateral_integral = 0
g = 32.033
Kpwm = 16

# PID object
pi_controller_f = PID(Kp_long, Ki_long, 0, sample_time=0.05)
pi_controller_lat = PID(Kp_lat, Ki_lat, 0, sample_time=0.05)
# Function takes in joystick value and maps it to a desired speed command in m/s
def map2speed(JoystickVal, mapping):
    InMin = float(mapping['JoystickMin'])
    InMax = float(mapping['JoystickMax'])
    OutMax = float(mapping['MaxSpeed'])
    OutMin = float(mapping['MinSpeed'])
    val = (JoystickVal - InMin) * (OutMax - OutMin) / (InMax - InMin) + OutMin
    return float(val)
# Function to map joystick input to PWM
# PWM range is 1900 to 1100 and joystick range is 1 to -1
def map2pwm(x):
    return int( (x - -1) * (1900 - 1100) / (1 - -1) + 1100)
# Returns a list of joystick commands (Throttle, Yaw, Roll, Pitch) from user
# mapped to PWM values
def getJoystickUpdates(mapping):
    joy_input = []
    joy_input.append(map2speed( j_interface.get_axis(int(mapping['Lateral'])),mapping )) # Lateral
    joy_input.append(map2speed(j_interface.get_axis(int(mapping['Forward'])), mapping)) # Forward
    joy_input.append(map2pwm(j_interface.get_axis(int(mapping['Yaw'])))) # Yaw, RC 4
    joy_input.append(map2pwm(-j_interface.get_axis(int(mapping['Throttle'])))) # Throttle (Needs inverted), RC 3
    return joy_input

# Evaluates states of buttons on controller and outputs corresponding commands
def ButtonUpdates(mapping):
    global pilot_joy_enable, toggle_arm, toggle_trc
    # Arm On/off
    if (j_interface.get_button(int(mapping['Arm'])) == 1):
        toggle_arm ^= 1
        # Depending on the updated state of the button, deploy the action
        if (toggle_arm):
            vehicle.armed = True
            print("Waiting for arming")
            while not vehicle.armed:
                time.sleep(1)
            print("armed")
            # Enable Joystick input for pilot
            pilot_joy_enable = True
        else:
            vehicle.armed = False
            print("Disarmed")
    # LOITER Mode
    if (j_interface.get_button(int(mapping['Loiter'])) == 1):
        print("Mode: LOITER")
        vehicle.mode = VehicleMode("LOITER")
    # LAND mode
    if (j_interface.get_button(int(mapping['Land'])) == 1):
        print("Mode: LAND")
        vehicle.mode = VehicleMode("LAND")
    # ALT HOLD mode
    if (j_interface.get_button(int(mapping['AltHold'])) == 1):
        print("Mode: AltHold")
        vehicle.mode = VehicleMode("ALT_HOLD")
    # TRC ON
    if (j_interface.get_button(int(mapping['TRC_ON'])) == 1):
        toggle_trc ^= 1
        if (toggle_trc):
            print("TRC ON")
        else:
            print("TRC OFF")

def run_trc(vehicle, joystick_inputs):
    global start_time, forward_f, lateral_f

    # Translational Rate Control
    # First step is to get user input and run through a command filter
    pilot_input_forward = -joystick_inputs[1] # m/s, joystick need flipped
    pilot_input_lateral = joystick_inputs[0] # m/s
    # We need to transform pilot input from heading frame into reference frame
    # Heading > Inertial > Calculate Error > Heading > Send to PID
    angles = vehicle.attitude # In Radians
    R = generate_rotation_matrix([angles.roll, angles.pitch, angles.yaw]) # From NED to Heading Frame
    R_inv = np.linalg.inv(R) # From Heading to NED Frame
    #HEAD_input = [pilot_input_forward, pilot_input_lateral, 0]
    #HEAD_input_array = np.asarray(HEAD_input)
    #NED_input_array = R_inv.dot(HEAD_input_array) # Transform the vector into NED frame
    #pilot_input_forward = NED_input_array[0]
    #pilot_input_lateral = NED_input_array[1]

    #print(pilot_input_forward)
    #print(pilot_input_lateral)
    # Get time interval
    current_time = time.time()
    dt = current_time - start_time
    start_time = current_time
    # Command Filter: 
    a = dt / (RC + dt)
    pilot_input_forward_f = (a * forward_f) + ((1-a) * float(pilot_input_forward))
    pilot_input_lateral_f = (a * lateral_f) + ((1-a) * float(pilot_input_lateral))
    # differentiate
    fwd_accel = (pilot_input_forward_f - forward_f)/dt # Pilot Forward Acceleration Feedforward
    forward_f = pilot_input_forward_f # Forward Velocity Setpoint
    lat_accel = (pilot_input_lateral_f - lateral_f)/dt # Pilot Lateral Acceleration Feedforward
    lateral_f = pilot_input_lateral_f # Lateral Velocity Setpoint

    # Now apply PI controller
    pi_controller_f.setpoint = forward_f * 3.28084 # ft/s
    pi_controller_lat.setpoint = lateral_f * 3.28084 # ft/s
    # GPS Velocity
    vel_meas = vehicle.velocity # [Vx, Vy, Vz] in m/s in NED
    velArray = np.asarray(vel_meas)
    vel_meas = R.dot(velArray) # NED to Heading
    print(vel_meas)
    control_f = pi_controller_f(vel_meas[0] * 3.28084) # Feedback in ft/s
    control_lat = pi_controller_lat(vel_meas[1] * 3.28084) # Feedback in ft/s
    # Output is in Radians
    pi_output_f = (-1/g) * (control_f + fwd_accel)
    pi_output_lat = (1/g) * (control_lat + lat_accel)

    # Convert Radians to Degrees
    pi_output_f_deg = (180/math.pi) * pi_output_f
    pi_output_lat_deg = (180/math.pi) * pi_output_lat

    # Convert to PWM with 1500 Trim
    pwm_output_f = (pi_output_f_deg * -17.333) 
    pwm_output_lat = (pi_output_lat_deg * 17.333)

    pwm_f = abs(int(pwm_output_f + 1500))
    pwm_l = abs(int(pwm_output_lat + 1500))
    
    # Enforce Saturation Limits
    if (pwm_f > 2000):
        pwm_f = 2000
    elif (pwm_f < 1000):
        pwm_f = 1000
    if (pwm_l > 2000):
        pwm_l = 2000
    elif (pwm_l < 1000):
        pwm_l = 1000
    vehicle.channels.overrides[1] = pwm_l
    vehicle.channels.overrides[2] = pwm_f
    #print("Lat PWM output: %s" % pwm_l)
    #print("Long PWM output: %s" % pwm_f)




"""
    Main Code starts here
"""

# Init joystick
pygame.init()
pygame.joystick.init() 

# Create a joystick instance
try:
    j_interface = pygame.joystick.Joystick(0)
    j_interface.init()
    print("Enabled Joystick: {0}".format(j_interface.get_name()))
except pygame.error:
    print("No Joystick found, exiting program")
    exit()

# Try loading the configuration file for the button mapping
try:
    with open('config.json', 'r') as f:
        config_map = json.load(f)
except Exception:
    print("Failed to load JSON file. Please check the config.json file and try again")
    pygame.quit()
    exit()
# If the map is empty, the JSON was not loaded correctly, exit the program and try again
if (bool(config_map) == False):
    print("Configuration file was not loaded properly. Please check the config.json file and try again")
    pygame.quit()
    exit()
else:
    print("Config file loaded correctly")


# Connect to the Ardupilot SITL UDP Endpoint and wait till the Vehicle is done intializing
# Connection time out will cause program to exit. user will have to restart the program
try:
    # Connect to the UDP endpoint
    vehicle = connect(str(config_map['UDP']), wait_ready=False)
    vehicle.mode = VehicleMode("LOITER") # Change the mode to LOITER at startup
    #print("Mode: %s" % vehicle.mode.name)
    print("Waiting for vehicle to initialize...")
    while not vehicle.is_armable:
        time.sleep(1)
    print("Vehicle is ready")

except Exception:
    print("Error has occurred. Please try restarting the program")
    pygame.quit()
    exit()


# Main loop

try:
    while True:
        
        # Check to see if any Gamepad events happened
        for event in pygame.event.get():
            # Updates joystick values and sends them for every time joystick is moved from origin
            if event.type == pygame.JOYAXISMOTION:
                joystick_inputs = getJoystickUpdates(config_map)

            if event.type == pygame.JOYBUTTONDOWN:
                ButtonUpdates(config_map)
            

        # Send Joystick Inputs if enabled
        if pilot_joy_enable:
            vehicle.channels.overrides = {'3': joystick_inputs[3], '4': joystick_inputs[2]}
        
        if (toggle_trc):
            run_trc(vehicle, joystick_inputs)
            #print("Velocity (m/s): %s" % vehicle.velocity)
        
        time.sleep(float(config_map['UpdateRate'])) # Radio Update Rate

except KeyboardInterrupt:
    pygame.quit()
    vehicle.close()

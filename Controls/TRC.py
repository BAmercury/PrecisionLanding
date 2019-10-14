# Translational Rate Control

# Using Dronekit to send commands to drone
from dronekit import connect, VehicleMode
import pygame
import pygame
import time
import json
import sys
import math

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


# Function takes in joystick value and maps it to a desired speed command in m/s
def map2speed(JoystickVal, mapping):
    return float( (JoystickVal - -1 * (float(mapping['MaxSpeed']) - float(mapping['MinSpeed']) / (1 - -1) + float(mapping['MinSpeed']))))
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
    global pilot_joy_enable, toggle_custom_mode, toggle_arm, toggle_trc
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

def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

def run_trc(vehicle, joystick_inputs):
    global start_time, forward_f, lateral_f, forward_integral, lateral_integral

    # Translational Rate Control
    output = []
    # First step is to get user input and run through a command filter
    pilot_input_forward = joystick_inputs[1]
    pilot_input_lateral = joystick_inputs[0]
    #print("Desired Forward (m/s): %s" % pilot_input_forward)
    #print("Desired Lateral (m/s): %s" % pilot_input_lateral)

    # Get time interval
    current_time = time.time()
    dt = current_time - start_time
    start_time = current_time
    # Command Filter: 
    a = dt / (RC + dt)
    pilot_input_forward_f = (a * forward_f) + ((1-a) * float(pilot_input_forward))
    pilot_input_lateral_f = (a * lateral_f) + ((1-a) * float(pilot_input_lateral))
    # differentiate
    fwd_accel = (pilot_input_forward_f - forward_f)/dt
    forward_f = pilot_input_forward_f
    lat_accel = (pilot_input_lateral_f - lateral_f)/dt
    lateral_f = pilot_input_lateral_f    

    # Save output
    output.append(forward_f) # Forward Velocity Setpoint
    output.append(fwd_accel) # Pilot Forward Acceleration Feedforward
    output.append(lateral_f) # Lateral Velocity Setpoint
    output.append(lat_accel) # Pilot Lateral Acceleration Feedforward

    # Now apply PI controller

    # Get the error between pilot and aircraft
    vel_meas = vehicle.velocity # [Vx, Vy, Vz] in m/s
    forward_error = forward_f - vel_meas[0]
    lateral_error = lateral_f - vel_meas[1]
    # Apply P
    forward_p = Kp_long * forward_error
    lat_p = Kp_lat * lateral_error
    # Apply I
    forward_integral = forward_integral + forward_error * dt
    lateral_integral = lateral_integral + lateral_error * dt
    #forward_integral = constrain(forward_integral, 0, 30)
    #lateral_integral = constrain(lateral_integral, 0, 30)
    #print(forward_integral)
    # Output is in Radians
    pi_output_f = (-1/g) * (forward_p + (Ki_long * forward_integral) + output[1])
    pi_output_lat = (1/g) * (lat_p + (Ki_lat * lateral_integral) + output[3])
    # Convert Radians to Degrees
    pi_output_f_deg = (180/math.pi) * pi_output_f
    pi_output_lat_deg = (180/math.pi) * pi_output_lat
    # Convert to PWM with 1500 Trim
    pwm_output_f = (pi_output_f_deg * -16) 
    pwm_output_lat = (pi_output_lat_deg * 16)

    # Output RC override (Pitch and Roll)
    final_output_roll = abs(int(pwm_output_lat + 1500))
    final_output_roll = constrain(final_output_roll, 0, 2000)
    final_output_pitch = abs(int(pwm_output_f + 1500))
    final_output_pitch = constrain(final_output_pitch, 0, 2000)
    print("Forward Deg: %s" % final_output_pitch)
    print("Lateral Deg: %s" % final_output_roll)
    vehicle.channels.overrides[1] = final_output_roll
    vehicle.channels.overrides[2] = final_output_pitch
    #vehicle.channels.overrides = {'1': abs(int(pwm_output_f + 1500)), '2': abs(int(pwm_output_lat + 1500))}




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
                #print "Attitude: %s" % vehicle.attitude
                #print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame.alt

        # Send Joystick Inputs if enabled
        if pilot_joy_enable:
            vehicle.channels.overrides = {'3': joystick_inputs[3], '4': joystick_inputs[2]}
        
        if (toggle_trc):
            run_trc(vehicle, joystick_inputs)
        
        time.sleep(float(config_map['UpdateRate'])) # Radio Update Rate

except KeyboardInterrupt:
    pygame.quit()
    vehicle.close()

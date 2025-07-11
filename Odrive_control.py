import odrive
from odrive.enums import *
import time
import math

odrv = None
timeout_seconds = 5

while odrv is None:
    try:
        print("Trying to find ODrive...")
        odrv = odrive.find_any(timeout=timeout_seconds)
    except TimeoutError:
        print(f"No ODrive found. Retrying in {timeout_seconds} seconds...")
        time.sleep(timeout_seconds)
# Configuration
axis = odrv.axis0  # Change to axis1 if needed
velocity = 10     # Target velocity in counts/s
current_threshold = 3  # Current threshold in Amps
back_A_thresh = 4
for_A_thresh = 4

#R60 Parameters
rated_torque = 0.75 #Nm
peak_torque = 2.3 #Nm
cont_A = 9 #A
peak_A = 28 #A
kt = 0.083 #Nm/A
kv = 115 #RPM/V
V_gain = 0.0325
pos_gain = 28
V_integral_gain = 0.0589

set_torque = 0.75
low_torque = 0.05

# Set velocity ramp input mode
axis.controller.config.vel_limit = 20
axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
axis.controller.config.vel_ramp_rate = 5  # Acceleration rate in counts/s^2
axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# Helper function to hold position
def hold_position(current_position):
    axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    axis.controller.input_pos = current_position
    print(f"Holding position at {current_position}")

#hold torque 
def hold_torque(torque):
    axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
    axis.controller.input_torque = torque
    print(f"Holding Torque at {torque}")

# Function to run the sequence
def run_motor(direction):
    axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
    axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    axis.controller.config.vel_ramp_rate = 5  # rev/s^2
    axis.controller.input_vel = direction * velocity

    print(f"Accelerating in direction {direction}")

    # Monitor current
    while True:
        current = axis.motor.foc.Iq_measured
        print(f"Current: {current:.2f} A")
        
        if direction == -1:  # CCW direction
            if abs(current) >= back_A_thresh:
                print("Current threshold reached in BACK direction")
                hold_torque(direction * low_torque)
                break
        
        elif direction == 1:  # CW direction
            if abs(current) >= for_A_thresh:
                print("Current threshold reached in FOR direction")
                hold_torque(direction * set_torque)
                # Continue running without holding position
                break
        
        time.sleep(0.1)

# Main sequence
try:
    while True:
        user_input = input("Enter direction (FOR/BACK), torq,  or 'exit': ").strip().lower()
        if user_input == 'for':
            run_motor(1)   # Forward (CW)
        elif user_input == 'back':
            run_motor(-1)  # Reverse (CCW)
        elif user_input == 'exit':
            print("Exiting script")
            axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
            axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            axis.controller.input_vel = 0
            axis.controller.input_torque = 0
            axis.requested_state = AXIS_STATE_IDLE
            #time.sleep(2)
            print(axis.motor.foc.Iq_measured)
            break
        elif user_input == "torq":
            while True:
                torque_input = input("Enter new torque set point: ")
                try:
                    number = float(torque_input)
                    print("Valid number:", number)
                    set_torque = torque_input
                    break  # exit the loop once input is valid
                except ValueError:
                    print("Invalid input. Please enter a number. You gave: ", torque_input)
                
        else:
            print("Invalid input. Please enter 'FOR', 'BACK', or 'exit'.")

except KeyboardInterrupt:
    print("Script stopped")
    axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    axis.controller.input_vel = 0
    axis.controller.input_torque = 0
    axis.requested_state = AXIS_STATE_IDLE

    #time.sleep(2)
    #print(axis.motor.foc.Iq_measured)


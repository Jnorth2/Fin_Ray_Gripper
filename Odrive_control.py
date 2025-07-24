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
print("Odrive Connected")
# Configuration
axis = odrv.axis0  # Change to axis1 if needed
velocity = 25     # Target velocity in counts/s
current_threshold = 3.0  # Current threshold in Amps
back_A_thresh = 1.0
for_A_thresh = 3.0
accel = 90 # 100 is too fast
deccel = 90

#R60 Parameters
rated_torque = 0.75 #Nm
peak_torque = 2.3 #Nm
cont_A = 9 #A
peak_A = 28 #A
kt = 0.083 #Nm/A  Actual kt may be closer to 0.07143
kv = 115 #RPM/V Actual kv maybe closer to 133
v_limit = 30 #max = Voltage * kv * 0.78 ~ 35.7 @ 24 v 
V_gain = 0.0325 #micro=0.055
pos_gain = 7 #micro=7 S1=28
V_integral_gain = 0.0589 #micro=0.025

set_torque = 0.25 #0.215 ~ 3 A, 0.25 ~ 3.5 A On Odrive S1
low_torque = 0.00
close_direction = -1 #{1, -1} = {micro, S1}

#On First 3 finger setup fingers extend in - direction
home_offset = 14 #{-14, 14} = {micro, S1}
home_pos = 0
for_pos = 0

def idle(axis):
    axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    axis.controller.input_vel = 0
    axis.controller.input_torque = 0
    axis.requested_state = AXIS_STATE_IDLE
    #time.sleep(2)
    print(axis.motor.foc.Iq_measured)

def homing(axis, direction, home_offset):
    print("homing")
    #set state and movement
    axis.controller.config.vel_limit = v_limit
    axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
    axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    axis.controller.config.vel_ramp_rate = 1
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    axis.controller.input_vel = direction * 1

    #Check for limit(ie torque increase)
    current = axis.motor.foc.Iq_measured
    while abs(current) < 3:
        current = axis.motor.foc.Iq_measured
        time.sleep(0.01)

    idle(axis)

    #get position
    home = axis.pos_estimate
    for_pos = home
    home = home + home_offset
    print("Home Position: ", home)
    return home, for_pos

def go_back(axis, pos):
    axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    axis.trap_traj.config.vel_limit = v_limit
    axis.trap_traj.config.accel_limit = accel
    axis.trap_traj.config.decel_limit = deccel
    axis.controller.input_pos = pos
    print(axis.pos_estimate, pos)
    while True:
        pos_estimate = axis.pos_estimate
        pos_error = abs(pos - pos_estimate)
        if pos_error < 0.015:  # Adjust this threshold as needed
            print("Motion complete.")
            break

        time.sleep(0.01)
    print("moved to position: ", axis.pos_estimate)

def go_for(axis, pos):
    axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    axis.trap_traj.config.vel_limit = v_limit
    axis.trap_traj.config.accel_limit = accel
    axis.trap_traj.config.decel_limit = deccel
    axis.controller.input_pos = pos
    print(axis.pos_estimate, pos)
    max_vel = 0
    max_current = 0
    while True:
        pos_estimate = axis.pos_estimate
        current = axis.motor.foc.Iq_measured
        vel_estimate = axis.vel_estimate
        max_vel = max(max_vel, abs(vel_estimate))
        max_current = max(max_current, abs(current))
        pos_error = abs(pos - pos_estimate)
        #print(f"position error: {pos_error}, Current: {current}, Velocity: {vel_estimate}")
        
        if abs(current) > current_threshold:
            print(f"Max Velocity: {max_vel} | Max Current: {max_current:.10f}")
            print("gripped object")
            hold_torque(close_direction * set_torque)
            break
        elif pos_error < 0.015:  # Adjust this threshold as needed
            print(f"Max Velocity: {max_vel} | Max Current: {max_current:.10f}")
            print("Motion complete.")
            break

        time.sleep(0.01)
    print("moved to position: ", axis.pos_estimate)

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
    time.sleep(0.5)
    current = axis.motor.foc.Iq_measured
    print(f"Holding Torque at {torque} | Current: {current}")

# Function to run the sequence
def run_motor(direction):
    axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
    axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    axis.controller.config.vel_ramp_rate = accel  # rev/s^2
    axis.controller.input_vel = direction * velocity

    print(f"Accelerating in direction {direction}")

    # Monitor current
    while True:
        current = axis.motor.foc.Iq_measured
        vel = axis.vel_estimate
        print(f"Current: {current:.2f} A | Velocity: {vel:.2f} rev/s")
        
        if direction == -close_direction:  # CCW direction
            if abs(current) >= back_A_thresh:
                axis.controller.input_vel = direction * 2
                print("Current threshold reached in BACK direction")
                hold_torque(direction * low_torque)
                break
        
        elif direction == close_direction:  # CW direction
            if abs(current) >= for_A_thresh:
                print("Current threshold reached in FOR direction")
                axis.controller.input_vel = direction * 2
                hold_torque(direction * set_torque)
                # Continue running without holding position
                break
        
        time.sleep(0.01)

# Main sequence
try:
    home_pos, for_pos = homing(axis, close_direction, home_offset)
    go_back(axis, home_pos)
    
    # Set velocity ramp input mode
    axis.controller.config.vel_limit = v_limit
    axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
    axis.controller.config.vel_ramp_rate = accel  # Acceleration rate in counts/s^2
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    while True:
        user_input = input("Enter direction (FOR/BACK), torq,  or 'exit': ").strip().lower()
        if user_input == 'for':
            go_for(axis, for_pos)
            #run_motor(close_direction)   # Forward (CW)
        elif user_input == 'back':
            go_back(axis, home_pos)  # Reverse (CCW)
        elif user_input == 'exit':
            print("Exiting script")
            idle(axis)
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
    idle(axis)
    odrv.reboot()


"""New_Controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from math import pi, sin

import numpy as np

from controller import Robot, Motor, DistanceSensor, GPS, InertialUnit

MAX_SPEED = 16
NULL_SPEED = 0
MIN_SPEED = -16

# System definition
WHEEL_RADIUS = 0.031
AXLE_LENGTH = 0.271756
ENCODER_RESOLUTION = 507.9188

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

sensor = robot.getDevice("inertial unit")
sensor.enable(timestep)
gps = robot.getDevice("gps")
gps.enable(timestep)

left_motor, right_motor = robot.getDevice("left wheel motor"), robot.getDevice("right wheel motor")


e_prev = 0     # error value in the previous interation (to calculate the derivative term)
e_acc = 0      # accumulated error value (to calculate the integral term)

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:


def get_acceleration(v0, v1,t1,t2):
    return (v1-v0)/(t2-t1)
    
def get_velocity(d,t):
    return d/t

currentPos = 0
previousPos = 0

def get_wheels_speed(encoderValues, oldEncoderValues, pulses_per_turn, delta_t):
    """Computes speed of the wheels based on encoder readings
    """
    # Calculate the change in angular position of the wheels:
    ang_diff_l = 2*np.pi*(encoderValues[0] - oldEncoderValues[0])/pulses_per_turn
    ang_diff_r = 2*np.pi*(encoderValues[1] - oldEncoderValues[1])/pulses_per_turn

    # Calculate the angular speeds:
    wl = ang_diff_l/delta_t
    wr = ang_diff_r/delta_t

    return wl, wr

def get_robot_speeds(wl, wr, R, D):
    """Computes robot linear and angular speeds"""
    u = R/2.0 * (wr + wl)
    w = R/D * (wr - wl)
    
    return u, w


# pulses_per_turn = 72
# delta_t = 0.1  # time step in seconds
# encoderValues = [507.9188, 507.9188]  # Accumulated number of pulses for the left [0] and right [1] encoders.
# oldEncoderValues = [507.9188, 507.9188]     # Accumulated pulses for the left and right encoders in the previous step

# wl, wr = get_wheels_speed(encoderValues, oldEncoderValues, pulses_per_turn, delta_t)

# print(f'Left wheel speed  = {wl} rad/s.')
# print(f'Right wheel speed = {wr} rad/s.')

def wheel_speed_commands(u_d, w_d, d, r):
    """Converts desired speeds to wheel speed commands"""
    wr_d = float((2 * u_d + d * w_d) / (2 * r))
    wl_d = float((2 * u_d - d * w_d) / (2 * r))
    
    # If saturated, correct speeds to keep the original ratio
    if np.abs(wl_d) > MAX_SPEED or np.abs(wr_d) > MAX_SPEED:
        speed_ratio = np.abs(wr_d)/np.abs(wl_d)
        if speed_ratio > 1:
            wr_d = np.sign(wr_d)*MAX_SPEED
            wl_d = np.sign(wl_d)*MAX_SPEED/speed_ratio
        else:
            wl_d = np.sign(wl_d)*MAX_SPEED
            wr_d = np.sign(wr_d)*MAX_SPEED*speed_ratio
    
    return wl_d, wr_d
    
def get_pose_error(xd, yd, x, y, phi):
    """ Returns the position and orientation errors. 
        Orientation error is bounded between -pi and +pi radians.
    """
    # Position error:
    x_err = xd - x
    y_err = yd - y
    dist_err = np.sqrt(x_err**2 + y_err**2)

    # Orientation error
    phi_d = np.arctan2(y_err,x_err)
    phi_err = phi_d - phi

    # Limits the error to (-pi, pi):
    phi_err_correct = np.arctan2(np.sin(phi_err),np.cos(phi_err))

    return dist_err, phi_err_correct
    
def pid_controller(e, e_prev, e_acc, delta_t, kp=1.0, kd=0, ki=0):
    """ PID algortithm: must be executed every delta_t seconds
    The error e must be calculated as: e = desired_value - actual_value
    e_prev contains the error calculated in the previous step.
    e_acc contains the integration (accumulation) term.
    """
    P = kp*e                      # Proportional term; kp is the proportional gain
    print(f"P: {P}")
    I = e_acc + ki*e*delta_t    # Intergral term; ki is the integral gain
    print(f"I: {I}")
    D = kd*(e - e_prev)/delta_t   # Derivative term; kd is the derivative gain
    print(f"D: {D}")

    output = P + I + D              # controller output

    # store values for the next iteration
    e_prev = e     # error value in the previous interation (to calculate the derivative term)
    e_acc = I      # accumulated error value (to calculate the integral term)

    return output, e_prev, e_acc

# Main loop:
# - perform simulation steps until Webots is stopping the controller

delta_t = 0.001
e_acc = 0
e_prev = 0

# Controller gains:
kp = 25
kd = 10
ki = 0.01

# Actual robot pose:
x, y, phi = 0, 0, 0

waypoints =[ [1,0], [2.5,2.5], [-2.5,2.5], [-2.5,0]]
currentWaypoint = 0
# Desired robot position:
xd, yd = waypoints[currentWaypoint][0], waypoints[currentWaypoint][1]

position_err, orientation_err = get_pose_error(xd, yd, x, y, phi)

e = orientation_err
e_prev = orientation_err*0.9

# Obtain the desired angular speed:
w_d, e_prev, e_acc = pid_controller(e, e_prev, e_acc, delta_t, kp, kd, ki)

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)


d_max = 0.5
d_min = 0.5
u_max = 8

timestep = 16



while robot.step(16) != -1:
    print(f"Phi: {sensor.getRollPitchYaw()[2]}")
    
    print(f"DeltaT: {delta_t}")
    phi = sensor.getRollPitchYaw()[2]
    x, y = gps.getValues()[0], gps.getValues()[1]
    position_err, orientation_err = get_pose_error(xd, yd, x, y, phi)
    
    e = orientation_err
    
    print(f'Distance error    = {position_err} m.')
    print(f'Orientation error = {orientation_err} rad.')
    
    # Obtain the desired angular speed:
    w_d, e_prev, e_acc = pid_controller(e, e_prev, e_acc, delta_t, kp, kd, ki)
    # left_motor.setVelocity(0)
    # right_motor.setVelocity(0)
    if position_err <= d_min and currentWaypoint < len(waypoints)-1:
        currentWaypoint+=1
        xd, yd = waypoints[currentWaypoint][0], waypoints[currentWaypoint][1]
        u_ref = u_max
    elif currentWaypoint == len(waypoints)-1:
        currentWaypoint=0
    elif position_err > d_min:
        u_ref = u_max
    else:
        u_ref = position_err/u_max
    print(f"u_ref: {u_ref}")
    wl_d, wr_d = wheel_speed_commands(u_ref, w_d, 0.271756, 0.031)
    left_motor.setVelocity(wl_d)
    right_motor.setVelocity(wr_d)
    # if position_err <0.01:
        # left_motor.setVelocity(0)
        # right_motor.setVelocity(0)
    delta_t+=16 / 1000
    pass

# Enter here exit cleanup code.



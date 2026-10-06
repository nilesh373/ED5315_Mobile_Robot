import math
from ed5315 import sim_interface, robot_params

previous_time = None

def estimate_pose(robot_state, Vl, Vr): #Function to estimate the robot's pose based on wheel velocities and previous state
    global previous_time #GLobal variable to keep track of the previous time step for integration

    current_time = sim_interface.sim_time() #Get the current simulation time

    # Calculate the time difference (dt) between the current and previous time steps for integration
    if previous_time is None: 
        dt = 0.0
    else:
        dt = current_time - previous_time

    previous_time = current_time #Equate the previous time to the current time for the next iteration

    # Convert wheel velocities from rad/s to m/s using the wheel radius
    v_left = Vl * robot_params.wheel_radius # Left wheel linear velocity
    v_right = Vr * robot_params.wheel_radius # Right wheel linear velocity

    # Differential-drive kinematics
    V = (v_left + v_right) / 2.0 # Average linear velocity of the robot
    W = (v_right - v_left) / robot_params.track_width # Angular velocity of the robot based on the difference in wheel velocities and the track width

    x, y, theta = robot_state # Unpack the current robot state into x, y coordinates and heading angle theta

    # Dead-reckoning integration
    x += V * math.cos(theta) * dt #Update the x-coordinate based on the linear velocity, heading angle, and time step
    y += V * math.sin(theta) * dt #Update the y-coordinate based on the linear velocity, heading angle, and time step
    theta += W * dt

    theta = (theta + math.pi) % (2.0 * math.pi) - math.pi #Wrap angle between -pi and pi 

    return [x, y, theta]
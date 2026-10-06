import math

from ed5315 import sim_interface, robot_params

previous_time = None


def estimate_pose(robot_state, Vl, Vr):
    global previous_time

    current_time = sim_interface.sim_time()

    if previous_time is None:
        dt = 0.0
    else:
        dt = current_time - previous_time

    previous_time = current_time

    # Convert wheel angular velocity [rad/s]
    # to wheel linear velocity [m/s]
    v_left = Vl * robot_params.wheel_radius
    v_right = Vr * robot_params.wheel_radius

    # Differential-drive kinematics
    V = (v_left + v_right) / 2.0
    W = (v_right - v_left) / robot_params.track_width

    x, y, theta = robot_state

    # Dead-reckoning integration
    x += V * math.cos(theta) * dt
    y += V * math.sin(theta) * dt
    theta += W * dt

    # Wrap heading to [-pi, pi)
    theta = (theta + math.pi) % (2.0 * math.pi) - math.pi

    return [x, y, theta]
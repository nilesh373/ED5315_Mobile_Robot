import math

from ed5315 import sim_interface, robot_params

prev_heading_error = 0.0
total_heading_error = 0.0
previous_time = None

def wrap_to_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

def at_goal(robot_state, goal_state):
    d = math.hypot(goal_state[0] - robot_state[0], goal_state[1] - robot_state[1])
    if d <= robot_params.goal_threshold:
        return True
    return False

def gtg(robot_state, goal_state):
    global prev_heading_error, previous_time

    xr = robot_state[0]
    yr = robot_state[1]
    theta = robot_state[2]

    xg = goal_state[0]
    yg = goal_state[1]

    dx = xg - xr
    dy = yg - yr

    distance = math.hypot(dx, dy)
    desired_heading = math.atan2(dy, dx)

    heading_error = wrap_to_pi(desired_heading - theta)

    current_time = sim_interface.sim_time()

    if previous_time is None:
        dt = 0.0
    else:
        dt = current_time - previous_time

    if dt > 1e-6:
        heading_error_dot = (heading_error - prev_heading_error) / dt
    else:
        heading_error_dot = 0.0

    prev_heading_error = heading_error
    previous_time = current_time

    Kp_V = 0.8
    Kp_W = 2.0
    Kd_W = 0.4

    V = Kp_V * distance

    W = Kp_W * heading_error + Kd_W * heading_error_dot

    V = max(min(V, robot_params.pioneer_max_V), -robot_params.pioneer_max_V)
    W = max(min(W, robot_params.pioneer_max_W), -robot_params.pioneer_max_W)

    return [V, W]

def avoid_obstacles(robot_state, nearby_obstacles):
    if not nearby_obstacles:
        return [0.0, 0.0]

    xr = robot_state[0]
    yr = robot_state[1]
    theta = robot_state[2]

    rep_x = 0.0
    rep_y = 0.0

    for obs in nearby_obstacles:
        dx = xr - obs[0]
        dy = yr - obs[1]

        distance = math.hypot(dx, dy)

        if distance < 1e-6:
            continue

        influence = robot_params.obstacle_sense_radius

        if distance < influence:
            weight = (influence - distance) / (distance * distance)

            rep_x += weight * (dx / distance)
            rep_y += weight * (dy / distance)

    if math.hypot(rep_x, rep_y) < 1e-6:
        return [0.0, 0.0]

    desired_heading = math.atan2(rep_y, rep_x)

    heading_error = wrap_to_pi(desired_heading - theta)

    K_avoid_W = 2.5

    W = K_avoid_W * heading_error

    min_distance = min(
        math.hypot(obs[0] - xr, obs[1] - yr)
        for obs in nearby_obstacles
    )

    safety_distance = robot_params.obstacle_radius + 0.5

    if min_distance <= safety_distance:
        V = 0.05
    elif min_distance <= 1.2:
        V = 0.12
    else:
        V = 0.18

    if abs(heading_error) > math.pi / 2:
        V = 0.02

    W = max(min(W, robot_params.pioneer_max_W), -robot_params.pioneer_max_W)
    V = max(min(V, robot_params.pioneer_max_V), -robot_params.pioneer_max_V)

    return [V, W]

def navigation_state_machine(robot_state, goal_state, nearby_obstacles):
    if not nearby_obstacles:
        return gtg(robot_state, goal_state)

    xr = robot_state[0]
    yr = robot_state[1]

    min_distance = min(
        math.hypot(obs[0] - xr, obs[1] - yr)
        for obs in nearby_obstacles
    )

    avoidance_distance = 1.5

    if min_distance <= avoidance_distance:
        return avoid_obstacles(robot_state, nearby_obstacles)

    return gtg(robot_state, goal_state)

def differential_drive_ik(V, W):
    V = max(min(V, robot_params.pioneer_max_V), -robot_params.pioneer_max_V)

    W = max(min(W, robot_params.pioneer_max_W), -robot_params.pioneer_max_W)

    r = robot_params.wheel_radius
    L = robot_params.track_width

    Vl = (V - (L / 2.0) * W) / r
    Vr = (V + (L / 2.0) * W) / r

    return [Vl, Vr]
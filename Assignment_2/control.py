import math
#This file is NOT what you submit for Assignment 2 - only perception.py is graded.
#Paste your own working control.py solution from Assignment 1 in here to test
#perception.py locally end-to-end - at_goal/gtg/differential_drive_ik drop in
#unchanged (goal_state has the same shape as Assignment 1), but avoid_obstacles
#and navigation_state_machine need adapting to tracked_obstacles (a list of
#ed5315.sensors.TrackedObstacle, not [x, y] pairs). See README.md's Submission section.
from ed5315 import sim_interface, robot_params

prev_heading_error = 0.0
total_heading_error = 0.0
previous_time = None

def wrap_to_pi(angle): #This function wraps an angle to the range [-pi, pi).
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

def at_goal(robot_state, goal_state): #This function checks if the robot is within a certain threshold distance of the goal state.
    d = math.hypot(goal_state[0] - robot_state[0], goal_state[1] - robot_state[1]) # Calculate the Euclidean distance between the robot and the goal.
    if d <= robot_params.goal_threshold: # If the distance is less than or equal to the goal threshold then successful.
        return True
    return False

def gtg(robot_state, goal_state): #Compute the control commands to drive the robot towards the goal state using a PD controller.
    global prev_heading_error, previous_time

    xr = robot_state[0]
    yr = robot_state[1]
    theta = robot_state[2]

    xg = goal_state[0]
    yg = goal_state[1]

    dx = xg - xr # Calculate the difference in x-coordinates between the goal and the robot.
    dy = yg - yr # Calculate the difference in y-coordinates between the goal and the robot.

    distance = math.hypot(dx, dy) # Calculate the Euclidean distance between the robot and the goal.
    desired_heading = math.atan2(dy, dx) #Calculate the desired heading angle to reach the goal.

    heading_error = wrap_to_pi(desired_heading - theta) #Difference between the desired heading and the current heading of the robot, wrapped to the range [-pi, pi).

    current_time = sim_interface.sim_time() #Get the current simulation time.

    if previous_time is None: #If this is the first time step, set dt to 0.0 to avoid division by zero in the derivative calculation.
        dt = 0.0
    else:
        dt = current_time - previous_time

    if dt > 1e-6:
        heading_error_dot = (heading_error - prev_heading_error) / dt
    else:
        heading_error_dot = 0.0

    prev_heading_error = heading_error
    previous_time = current_time

    #PD controller gains
    Kp_V = 0.8
    Kp_W = 2.0
    Kd_W = 0.4

    #Compute the linear and angular velocities based on the distance to the goal and the heading error.
    V = Kp_V * distance

    W = Kp_W * heading_error + Kd_W * heading_error_dot

    V = max(min(V, robot_params.pioneer_max_V), -robot_params.pioneer_max_V) #Clamp the linear velocity to the maximum and minimum values defined in robot_params.
    W = max(min(W, robot_params.pioneer_max_W), -robot_params.pioneer_max_W) #Clamp the angular velocity to the maximum and minimum values defined in robot_params.

    return [V, W]

def avoid_obstacles(robot_state, tracked_obstacles):

    if not tracked_obstacles:
        return [0.0, 0.0]

    xr = robot_state[0]
    yr = robot_state[1]
    theta = robot_state[2]

    rep_x = 0.0
    rep_y = 0.0

    relevant_obstacles = []

    # Only consider obstacles within the sensing/influence radius.
    for obs in tracked_obstacles:

        dx = obs.world_x - xr
        dy = obs.world_y - yr

        distance = math.hypot(dx, dy)

        if distance < robot_params.obstacle_sense_radius:
            relevant_obstacles.append(obs)

    if not relevant_obstacles:
        return [0.0, 0.0]

    # Compute repulsive force.
    for obs in relevant_obstacles:

        dx = xr - obs.world_x
        dy = yr - obs.world_y

        distance = math.hypot(dx, dy)

        if distance < 1e-6:
            continue

        influence = robot_params.obstacle_sense_radius

        if distance < influence:

            weight = ((influence - distance)/ (distance * distance))

            rep_x += weight * (dx / distance)
            rep_y += weight * (dy / distance)

    if math.hypot(rep_x, rep_y) < 1e-6:
        return [0.0, 0.0]

    desired_heading = math.atan2(rep_y, rep_x)

    heading_error = wrap_to_pi(
        desired_heading - theta
    )

    K_avoid_W = 2.5

    W = K_avoid_W * heading_error

    min_distance = min(math.hypot(obs.world_x - xr,obs.world_y - yr)for obs in relevant_obstacles)

    safety_distance = (robot_params.obstacle_radius + 0.5)

    if min_distance <= safety_distance:
        V = 0.05

    elif min_distance <= 1.2:
        V = 0.12

    else:
        V = 0.18

    if abs(heading_error) > math.pi / 2:
        V = 0.02

    W = max(min(W, robot_params.pioneer_max_W),-robot_params.pioneer_max_W)

    V = max(min(V, robot_params.pioneer_max_V),-robot_params.pioneer_max_V)

    return [V, W]


def navigation_state_machine(
    robot_state,
    goal_state,
    tracked_obstacles
):

    if not tracked_obstacles:
        return gtg(robot_state, goal_state)

    xr = robot_state[0]
    yr = robot_state[1]

    relevant_obstacles = []

    for obs in tracked_obstacles:

        distance = math.hypot(
            obs.world_x - xr,
            obs.world_y - yr
        )

        if distance <= robot_params.obstacle_sense_radius:
            relevant_obstacles.append(obs)

    if not relevant_obstacles:
        return gtg(robot_state, goal_state)

    min_distance = min(
        math.hypot(
            obs.world_x - xr,
            obs.world_y - yr
        )
        for obs in relevant_obstacles
    )

    avoidance_distance = 1.5

    if min_distance <= avoidance_distance:
        return avoid_obstacles(
            robot_state,
            relevant_obstacles
        )

    return gtg(robot_state, goal_state)

def differential_drive_ik(V, W):

    V = max(
        min(V, robot_params.pioneer_max_V),
        -robot_params.pioneer_max_V
    )

    W = max(
        min(W, robot_params.pioneer_max_W),
        -robot_params.pioneer_max_W
    )

    r = robot_params.wheel_radius
    L = robot_params.track_width

    Vl = (
        V - (L / 2.0) * W
    ) / r

    Vr = (
        V + (L / 2.0) * W
    ) / r

    return [Vl, Vr]

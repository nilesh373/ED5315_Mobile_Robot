import math

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

def avoid_obstacles(robot_state, nearby_obstacles):
    if not nearby_obstacles:
        return [0.0, 0.0]

    xr = robot_state[0]
    yr = robot_state[1]
    theta = robot_state[2]

    rep_x = 0.0 #Initialize the repulsive force in the x-direction to zero.
    rep_y = 0.0 #Initialize the repulsive force in the y-direction to zero.

    #Compute the repulsive forces from nearby obstacles
    for obs in nearby_obstacles:
        dx = xr - obs[0]
        dy = yr - obs[1]

        distance = math.hypot(dx, dy)

        if distance < 1e-6: #Simply skip this obstacle if the distance is too small to avoid division by zero.
            continue 

        influence = robot_params.obstacle_sense_radius #Defined in robot_params.py, this is the distance within which obstacles influence the robot's motion.

        if distance < influence:
            weight = (influence - distance) / (distance * distance)

            rep_x += weight * (dx / distance)
            rep_y += weight * (dy / distance)

    if math.hypot(rep_x, rep_y) < 1e-6: 
        return [0.0, 0.0]

    desired_heading = math.atan2(rep_y, rep_x)

    heading_error = wrap_to_pi(desired_heading - theta)

    K_avoid_W = 2.5 #Gain factor for the angular velocity when avoiding obstacles. This value determines how aggressively the robot will turn to avoid obstacles.

    W = K_avoid_W * heading_error #The angular velocity is proportional to the heading error, scaled by a gain factor.

    min_distance = min(math.hypot(obs[0] - xr, obs[1] - yr) for obs in nearby_obstacles)

    safety_distance = robot_params.obstacle_radius + 0.5

    if min_distance <= safety_distance: #Simply set the linear velocity to a small value if the robot is too close to an obstacle.
        V = 0.05
    elif min_distance <= 1.2:
        V = 0.12
    else:
        V = 0.18

    if abs(heading_error) > math.pi / 2: #Simply set the linear velocity to a small value if the robot is turning away from the obstacles.
        V = 0.02

    W = max(min(W, robot_params.pioneer_max_W), -robot_params.pioneer_max_W)
    V = max(min(V, robot_params.pioneer_max_V), -robot_params.pioneer_max_V)

    return [V, W]

def navigation_state_machine(robot_state, goal_state, nearby_obstacles):
    if not nearby_obstacles:
        return gtg(robot_state, goal_state)

    xr = robot_state[0]
    yr = robot_state[1]

    #Compute the minimum distance to any nearby obstacle
    min_distance = min(math.hypot(obs[0] - xr, obs[1] - yr) for obs in nearby_obstacles) 

    avoidance_distance = 1.5 #Simply set the avoidance distance to a fixed value.
    if min_distance <= avoidance_distance:
        return avoid_obstacles(robot_state, nearby_obstacles)

    return gtg(robot_state, goal_state)

def differential_drive_ik(V, W):
    V = max(min(V, robot_params.pioneer_max_V), -robot_params.pioneer_max_V)

    W = max(min(W, robot_params.pioneer_max_W), -robot_params.pioneer_max_W)

    r = robot_params.wheel_radius
    L = robot_params.track_width

    #Compute the left and right wheel velocities based on the linear and angular velocities of the robot using the differential drive kinematics equations.
    Vl = (V - (L / 2.0) * W) / r
    Vr = (V + (L / 2.0) * W) / r

    return [Vl, Vr]
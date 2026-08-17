import math

from ed5315 import sim_interface, robot_params

prev_heading_error = 0.0
total_heading_error = 0.0
previous_time = None

def wrap_to_pi(angle): #Wrap an angle in radians to [-pi, pi].
    return (angle + math.pi) % (2.0 * math.pi) - math.pi 

def at_goal(robot_state, goal_state):
    #check if we have reached goal point based on robot_params.goal_threshold
    #return True/False
    d = math.hypot(goal_state[0] - robot_state[0], goal_state[1] - robot_state[1]) #Find the distance to the goal from the robot's current position
    if d <= robot_params.goal_threshold: #Check if the distance is less than or equal to the goal threshold = 0.3
        return True #We have reached the goal
    return False #We have not reached the goal

def gtg(robot_state, goal_state):
    #The Go to goal controller
    #Create a PD controller for angular velocity
    #Create a seperate P controller for linear velocity
    #return [V, W]
    global prev_heading_error, previous_time #Call the global variables to keep track of the previous heading error and previous time

    xr = robot_state[0] #Robot's x-coordinate at the current time step
    yr = robot_state[1] #Robot's y-coordinate at the current time step
    theta = robot_state[2] #Robot's orientation at the current time step

    xg = goal_state[0] #Goal's x-coordinate
    yg = goal_state[1] #Goal's y-coordinate

    dx = xg - xr #Difference in x-coordinates between the goal and the robot
    dy = yg - yr #Difference in y-coordinates between the goal and the robot

    distance = math.hypot(dx, dy) #Calculate the Euclidean distance between the robot and the goal
    desired_heading = math.atan2(dy, dx) #Actual heading of the robot to the goal in radians

    heading_error = wrap_to_pi(desired_heading - theta) #Calculate the heading error by subtracting the robot's orientation from the desired heading and wrapping it to the range [-pi, pi]

    current_time = sim_interface.sim_time() #Get the current simulation time from the sim_interface module

    if previous_time is None: 
        dt = 0.0 #If this is the first time step, set dt to 0.0
    else:
        dt = current_time - previous_time #Calculate the time difference between the current time and the previous time step

    if dt > 1e-6: #Check if the time difference is greater than a small threshold to avoid division by zero
        heading_error_dot = (heading_error - prev_heading_error) / dt #Calculate the derivative of the heading error by dividing the change in heading error by the time difference
    else:
        heading_error_dot = 0.0

    prev_heading_error = heading_error
    previous_time = current_time

    Kp_V = 0.8 #Proportional gain for linear velocity
    Kp_W = 2.0 #Proportional gain for angular velocity
    Kd_W = 0.4 #Derivative gain for angular velocity

    V = Kp_V * distance #Calculate the linear velocity using a proportional controller based on the distance to the goal.                                                                

    W = (Kp_W * heading_error + Kd_W * heading_error_dot) #Calculate the angular velocity using a PD controller based on the heading error and its derivative.

    V = max(min(V, robot_params.pioneer_max_V), -robot_params.pioneer_max_V) #Limit the linear velocity to the robot's maximum speed.

    W = max(min(W, robot_params.pioneer_max_W), -robot_params.pioneer_max_W) #Limit the angular velocity to the robot's maximum turning rate.

    return [V, W]

def avoid_obstacles(robot_state, nearby_obstacles): #Needs robot_state and nearby_obstacles as inputs to calculate the avoidance behavior
    #Reactive obstacle avoidance controller.
    #nearby_obstacles: list of [x, y] world positions of obstacles currently
    #within robot_params.obstacle_sense_radius of the robot (see main.py). Every obstacle has radius robot_params.obstacle_radius.
    #Design your own strategy here.
    #return [V, W]
    if not nearby_obstacles: #If there are no nearby obstacles, return zero linear and angular velocities, indicating that the robot can proceed without any avoidance maneuvers.
        return [0.0, 0.0]

    xr = robot_state[0] #Robot's x-coordinate at the current time step
    yr = robot_state[1] #Robot's y-coordinate at the current time step
    theta = robot_state[2] #Robot's heading at the current time step

    rep_x = 0.0 #Maintain a cumulative repulsive force in the x-direction, initialized to zero
    rep_y = 0.0 #Maintain a cumulative repulsive force in the y-direction, initialized to zero

    for obs in nearby_obstacles:
        dx = xr - obs[0] #Calculate the difference in x-coordinates between the robot and the obstacle
        dy = yr - obs[1] #Calculate the difference in y-coordinates between the robot and the obstacle

        distance = math.hypot(dx, dy) #Calculate the Euclidean distance between the robot.

        if distance < 1e-6: #If the distance is very small (less than a threshold), skip this obstacle to avoid division by zero or numerical instability in the calculations.
            continue

        weight = 1.0 / (distance * distance) #Calculate a weight for the repulsive force. This means that closer obstacles will exert a stronger repulsive force on the robot.

        rep_x += weight * (dx / distance) #Add the x-component of the repulsive force to the cumulative sum
        rep_y += weight * (dy / distance) #Add the y-component of the repulsive force to the cumulative sum

    if math.hypot(rep_x, rep_y) < 1e-6: #If the cumulative repulsive force is very small (less than a threshold), return zero linear and angular velocities, indicating that the robot can proceed without any avoidance maneuvers.
        return [0.0, 0.0]

    desired_heading = math.atan2(rep_y, rep_x) #Calculate the desired heading for the robot to move away from the obstacles based on the cumulative repulsive force vector.

    heading_error = wrap_to_pi(desired_heading - theta) #Calculate the heading error by subtracting the robot's current heading from the desired heading and wrapping it to the range [-pi, pi].

    K_avoid_W = 2.5 #Gain for angular velocity in the obstacle avoidance controller. This gain determines how aggressively the robot will turn to avoid obstacles.

    W = K_avoid_W * heading_error #Calculate the angular velocity for obstacle avoidance using a proportional controller based on the heading error.

    min_distance = min(math.hypot(obs[0] - xr, obs[1] - yr) for obs in nearby_obstacles) #Calculate the minimum distance to any nearby obstacle by computing the Euclidean distance from the robot to each obstacle and taking the minimum value.

    safety_distance = robot_params.obstacle_radius + 0.5 #Define a safety distance that is the sum of the obstacle radius and an additional buffer (0.5 meters) to ensure the robot maintains a safe distance from obstacles.

    if min_distance <= safety_distance: #If the minimum distance to any nearby obstacle is less than or equal to the safety distance, set the linear velocity to a very low value (0.05 m/s) to prioritize obstacle avoidance and prevent collisions.
        V = 0.05
    else:
        V = 0.15

    if abs(heading_error) > math.pi / 2: #If the absolute value of the heading error is greater than 90 degrees (pi/2 radians), it means the robot is facing away from the desired direction. In this case, set the linear velocity to a very low value (0.02 m/s) to allow the robot to turn in place and reorient itself towards the desired heading before moving forward.
        V = 0.02

    W = max(min(W, robot_params.pioneer_max_W), -robot_params.pioneer_max_W) #Limit the angular velocity to the robot's maximum turning rate, ensuring that the robot does not exceed its physical capabilities when avoiding obstacles.

    V = max(min(V, robot_params.pioneer_max_V), -robot_params.pioneer_max_V) #Limit the linear velocity to the robot's maximum speed, ensuring that the robot does not exceed its physical capabilities when avoiding obstacles.

    return [V, W]

def navigation_state_machine(robot_state, goal_state, nearby_obstacles):
    #Combine gtg() and avoid_obstacles() into a state machine that switches
    #between goal-seeking and obstacle-avoidance behaviour.
    #Design your own switching logic here.
    #return [V, W]
    if nearby_obstacles: #If there are nearby obstacles, call the avoid_obstacles() function to calculate the linear and angular velocities for obstacle avoidance. This ensures that the robot prioritizes avoiding collisions with obstacles over moving towards the goal.
        return avoid_obstacles(robot_state, nearby_obstacles)

    return gtg(robot_state, goal_state) #If there are no nearby obstacles, call the gtg() function to calculate the linear and angular velocities for goal-seeking behavior. This allows the robot to move towards the goal when it is safe to do so.

def differential_drive_ik(V, W):
    #Ensure the given V and W fall within robot_params.pioneer_max_W, robot_params.pioneer_max_V
    #Convert a desired linear velocity V [m/s] and angular velocity W [rad/s]
    #into left/right wheel angular velocities [rad/s], using
    #robot_params.wheel_radius and robot_params.track_width.
    #return [Vl, Vr]
    V = max(min(V, robot_params.pioneer_max_V), -robot_params.pioneer_max_V) #Limit the linear velocity to the robot's maximum speed, ensuring that the robot does not exceed its physical capabilities when moving towards the goal or avoiding obstacles.

    W = max(min(W, robot_params.pioneer_max_W), -robot_params.pioneer_max_W) #Limit the angular velocity to the robot's maximum turning rate, ensuring that the robot does not exceed its physical capabilities when turning or avoiding obstacles.

    r = robot_params.wheel_radius
    L = robot_params.track_width

    Vl = (V - (L / 2.0) * W) / r #Calculate the left wheel angular velocity (Vl) based on the desired linear velocity (V), angular velocity (W), wheel radius (r), and track width (L). The formula accounts for the contribution of the robot's turning motion to the left wheel's speed.
    Vr = (V + (L / 2.0) * W) / r #Calculate the right wheel angular velocity (Vr) based on the desired linear velocity (V), angular velocity (W), wheel radius (r), and track width (L). The formula accounts for the contribution of the robot's turning motion to the right wheel's speed.

    return [Vl, Vr]

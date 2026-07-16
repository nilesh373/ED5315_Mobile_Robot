from ed5315 import sim_interface, robot_params

prev_heading_error = 0.0
total_heading_error = 0.0
previous_time = None

def at_goal(robot_state, goal_state):
    #check if we have reached goal point based on robot_params.goal_threshold
    #return True/False
    
def gtg(robot_state, goal_state):
    #The Go to goal controller
    #Create a PD controller for angular velocity
    #Create a seperate P controller for linear velocity

    V = ??
    W = ??

    return[V,W]

def avoid_obstacles(robot_state, nearby_obstacles):
    #Reactive obstacle avoidance controller.
    #nearby_obstacles: list of [x, y] world positions of obstacles currently
    #within OBSTACLE_SENSE_RADIUS of the robot (see main.py) - not a memory,
    #may be empty, and an obstacle drops back out once the robot moves away
    #from it. Every obstacle has radius robot_params.obstacle_radius.
    #Design your own strategy here - e.g. steer away from the nearest one.

    V = ??
    W = ??

    return[V,W]

def navigation_state_machine(robot_state, goal_state, nearby_obstacles):
    #Combine gtg() and avoid_obstacles() into a state machine that switches
    #between goal-seeking and obstacle-avoidance behaviour, e.g. based on how
    #close the nearest currently-nearby obstacle is.
    #Design your own switching logic here.

    V = ??
    W = ??

    return[V,W]

def differential_drive_ik(V, W):
    #Ensure the given V and W fall within robot_params.pioneer_max_W, robot_parasm.pioneer_max_V
    #Convert a desired linear velocity V [m/s] and angular velocity W [rad/s]
    #into left/right wheel angular velocities [rad/s], using
    #robot_params.wheel_radius and robot_params.track_width.
    #

    Vl = ??
    Vr = ??
    
    #request robot to execute velocity
    return [Vl, Vr]
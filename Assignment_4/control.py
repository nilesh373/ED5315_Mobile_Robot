#This file is NOT what you submit for Assignment 4 - only localization.py is graded.
#Paste your own working control.py solution from Assignment 1/2/3 in here to test
#localization.py locally end-to-end - it's unchanged since Assignment 2: robot_state
#is just an [x, y, theta] triple to this file either way, it doesn't know or
#care whether it's ground truth (Assignment 1/2), your own odometry estimate
#(Assignment 3), or your own EKF estimate (this assignment). See README.md's
#Submission section.
from ed5315 import sim_interface, robot_params

prev_heading_error = 0.0
total_heading_error = 0.0
previous_time = None

def at_goal(robot_state, goal_state):
    #check if we have reached goal point based on robot_params.goal_threshold
    #return True/False
    raise NotImplementedError

def gtg(robot_state, goal_state):
    #The Go to goal controller
    #Create a PD controller for angular velocity
    #Create a seperate P controller for linear velocity
    #return [V, W]
    raise NotImplementedError

def avoid_obstacles(robot_state, tracked_obstacles):
    #Reactive obstacle avoidance controller.
    #tracked_obstacles: the full list of perception.TrackedObstacle - may be
    #empty, and it's not filtered to "currently visible" for you: every
    #obstacle ever detected stays in it, each with .id, .world_x, .world_y,
    #.velocity_x, .velocity_y, .detected_time. Decide yourself which ones are
    #still relevant (e.g. by distance, or how recent .detected_time is) and
    #whether to treat a moving obstacle (.velocity_x/.velocity_y) differently
    #from a stationary one. Every obstacle has radius robot_params.obstacle_radius.
    #Design your own strategy here - e.g. steer away from the nearest one.
    #return [V, W]
    raise NotImplementedError

def navigation_state_machine(robot_state, goal_state, tracked_obstacles):
    #Combine gtg() and avoid_obstacles() into a state machine that switches
    #between goal-seeking and obstacle-avoidance behaviour, e.g. based on how
    #close the nearest relevant obstacle in tracked_obstacles is.
    #Design your own switching logic here.
    #return [V, W]
    raise NotImplementedError

def differential_drive_ik(V, W):
    #Ensure the given V and W fall within robot_params.pioneer_max_W, robot_params.pioneer_max_V
    #Convert a desired linear velocity V [m/s] and angular velocity W [rad/s]
    #into left/right wheel angular velocities [rad/s], using
    #robot_params.wheel_radius and robot_params.track_width.
    #return [Vl, Vr]
    raise NotImplementedError

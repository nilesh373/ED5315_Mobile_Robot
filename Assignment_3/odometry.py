from ed5315 import sim_interface, robot_params

previous_time = None

def estimate_pose(robot_state, Vl, Vr):
    #Dead-reckoning: integrate the two wheels' angular velocities into an
    #updated pose estimate.
    #
    #robot_state: [x, y, theta] - your own previous pose ESTIMATE, not ground
    #    truth. main.py seeds this once at the start from
    #    sim_interface.localize_robot(), then only ever updates it by
    #    calling this function from here on - it's never re-queried from the
    #    simulator.
    #Vl, Vr: left/right wheel angular velocities [rad/s], from
    #    ed5315.sensors.read_wheel_velocities() - these carry simulated
    #    encoder noise (not the commanded velocity), so your estimate will
    #    drift from ground truth over time. That drift is expected - it's
    #    exactly what this assignment is about.
    #
    #Use ed5315.sim_interface.sim_time() (and a module-level variable, e.g.
    #previous_time above) to get the elapsed time dt since your last call,
    #the same way Assignment 1/2's gtg sources its own dt.
    #Convert Vl/Vr to linear wheel speeds via robot_params.wheel_radius, then
    #to robot linear/angular velocity via robot_params.track_width, then
    #integrate x/y/theta forward by dt.
    #
    #return the updated [x, y, theta] estimate
    raise NotImplementedError

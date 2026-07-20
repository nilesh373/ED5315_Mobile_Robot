#This file is NOT what you submit for Assignment 4 - only localization.py is graded.
#Paste your own working odometry.py solution from Assignment 3 in here -
#localization.py's EKF calls estimate_pose() below directly as its own
#prediction step, so a working odometry.py is required for localization.py to
#run at all, not just for local testing. See README.md's Submission section.
from ed5315 import sim_interface, robot_params

previous_time = None

def estimate_pose(robot_state, Vl, Vr):
    #Dead-reckoning: integrate the two wheels' angular velocities into an
    #updated pose estimate.
    #
    #robot_state: [x, y, theta] - the previous pose ESTIMATE this call starts
    #    from - localization.py passes in its own EKF mean here each call, not
    #    ground truth.
    #Vl, Vr: left/right wheel angular velocities [rad/s], from
    #    ed5315.sensors.read_wheel_velocities() - these carry simulated
    #    encoder noise (not the commanded velocity), so your estimate will
    #    drift from ground truth over time. That drift is expected - it's
    #    what localization.py's EKF correction step is for.
    #
    #Use ed5315.sim_interface.sim_time() (and a module-level variable, e.g.
    #previous_time above) to get the elapsed time dt since your last call,
    #the same way Assignment 1/2/3's gtg/estimate_pose source their own dt.
    #Convert Vl/Vr to linear wheel speeds via robot_params.wheel_radius, then
    #to robot linear/angular velocity via robot_params.track_width, then
    #integrate x/y/theta forward by dt.
    #
    #return the updated [x, y, theta] estimate
    raise NotImplementedError

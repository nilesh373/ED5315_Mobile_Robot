#This IS what you submit for Assignment 4 - the only file that's graded. See
#README.md's Submission section.
from ed5315 import sim_interface, robot_params
import odometry

covariance = None  # your EKF's own pose covariance - lazily initialized on
                    # first call, exactly like odometry.py's previous_time

def estimate_pose(robot_state, Vl, Vr, tracked_obstacles, ground_truth_map):
    #EKF-based pose estimate: fuses wheel odometry (prediction) with
    #known-landmark range/bearing corrections (update) to counter the drift
    #Assignment 3 left uncorrected. This is EKF *localization* - the map
    #(ground_truth_map) is given and fixed, never estimated - not EKF SLAM.
    #
    #robot_state: [x, y, theta] - your own previous pose ESTIMATE (the EKF's
    #    mean from the last call), never re-queried from the simulator after
    #    the very first call in main.py. This is also the exact pose
    #    perception.py used this same iteration to convert its detections
    #    into tracked_obstacles' world_x/world_y - useful if you want to
    #    recover the raw (range, bearing) measurement it started from:
    #    given world_x, world_y and this same robot_state = [x, y, theta],
    #        range_m = hypot(world_x - x, world_y - y)
    #        bearing = atan2(world_y - y, world_x - x) - theta
    #    exactly inverts perception.py's own range/bearing -> world_x/world_y
    #    conversion.
    #Vl, Vr: left/right wheel angular velocities [rad/s] - same noisy
    #    readings as Assignment 3. Use odometry.estimate_pose(robot_state,
    #    Vl, Vr) directly for your EKF's prediction-step mean (no need to
    #    reimplement the motion model here) - paste your own working
    #    Assignment 3 solution into odometry.py first. You'll separately need
    #    to propagate your covariance through that same motion model's
    #    Jacobians (state Jacobian wrt [x, y, theta], control Jacobian wrt
    #    [Vl, Vr]) plus process noise from robot_params.wheel_velocity_noise_std.
    #tracked_obstacles: list of perception.TrackedObstacle (same shape as
    #    Assignment 2/3) - filter to whichever entries were actually detected
    #    THIS call to use as EKF correction measurements this step; a stale
    #    entry from an earlier call shouldn't correct against a pose you've
    #    since moved away from. Careful how you check "this call", though:
    #    perception.py stamped detected_time using sim_time() *before*
    #    main.py's step() this iteration, but by the time main.py calls you,
    #    step() has already run - so comparing against a fresh
    #    ed5315.sim_interface.sim_time() call from inside this function will
    #    never match (it reads one step later) and silently discards every
    #    detection. Track your own previous_time the same way odometry.py
    #    does, and compare obs.detected_time against the OLD value (from
    #    before you update it this call) - that's the sim time perception.py
    #    actually saw this iteration.
    #ground_truth_map: {tag_id: (x, y)} from
    #    ed5315.sim_interface.get_ground_truth_map() - the known, true
    #    position of every obstacle. Look up each detected obstacle's true
    #    position by its .id and use it as a known landmark in your EKF's
    #    update step (predicted range/bearing from your current pose estimate
    #    to that known position, vs. the measured range/bearing recovered
    #    above - the innovation corrects your pose and shrinks your
    #    covariance). Assumed measurement noise for this step:
    #    robot_params.ekf_range_noise_std / ekf_bearing_noise_std.
    #
    #Maintain your own covariance as module-level state (see the variable
    #declared above) - main.py never sees or passes it in, exactly like
    #Assignment 1-3's dt handling. Initialize it the first time this function
    #is called (covariance is None) - since main.py seeds robot_state from
    #ground truth, a small initial covariance is reasonable.
    #
    #return the updated [x, y, theta] estimate
    raise NotImplementedError

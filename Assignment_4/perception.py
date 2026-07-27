#This file is NOT what you submit for Assignment 4 - only localization.py is graded.
#Paste your own working perception.py solution from Assignment 2/3 in here to test
#localization.py locally end-to-end. See README.md's Submission section.
from ed5315 import sim_interface, robot_params
from ed5315.sensors import TrackedObstacle

def detect_obstacles(image, lidar_scan, robot_state, tracked_obstacles):
    #Detect every currently-visible obstacle from the camera image and lidar
    #scan, and update tracked_obstacles so each obstacle's velocity can be
    #estimated across calls.
    #
    #image: RGB array from ed5315.sensors.read_camera_image() - already
    #    upscaled for you, so ArUco tags decode reliably.
    #lidar_scan: list of (angle_rad, dist_m|None) from ed5315.sensors.read_lidar()
    #robot_state: [x, y, theta] - the robot's own pose estimate (from
    #    localization.py this assignment, not ground truth) - use it to
    #    convert a detection from the robot/camera frame into world
    #    coordinates. Since robot_state can drift from the truth, a detected
    #    obstacle's world position inherits that same drift - that's
    #    expected, and exactly what localization.py's EKF corrects for.
    #tracked_obstacles: a list of TrackedObstacle (imported above, from
    #    ed5315.sensors - not defined here) - whatever
    #    you returned from the previous call, starting as an empty list on
    #    the first call. When an id is seen again, update its existing
    #    entry's world_x/world_y/velocity_x/velocity_y/detected_time (use
    #    ed5315.sim_interface.sim_time() and the entry's previous
    #    detected_time to work out velocity_x/velocity_y). The first time an
    #    id is seen, append a new TrackedObstacle for it (velocity 0). Every
    #    obstacle has radius robot_params.obstacle_radius.
    #
    #Returns the updated tracked_obstacles (list of TrackedObstacle), passed
    #back into the next call *and* used directly by control.py for
    #navigation. If you want navigation to ignore obstacles that haven't been
    #seen in a while, decide that yourself (e.g. in control.py, using an
    #entry's detected_time).
    raise NotImplementedError

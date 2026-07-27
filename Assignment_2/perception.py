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
    #robot_state: [x, y, theta] - the robot's own ground-truth pose - use it to convert a detection from the
    #    robot/camera frame into world coordinates, so a moving obstacle's
    #    velocity isn't confounded with the robot's own motion.
    #tracked_obstacles: a list of TrackedObstacle (imported above, from
    #    ed5315.sensors - not defined here, so your submission can't change
    #    its shape) - whatever you returned from the previous call, starting
    #    as an empty list on the first call. When an id is seen again, update its existing
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

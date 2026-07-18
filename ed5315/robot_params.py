wheel_radius = 0.0975  # [m] Radius of the wheels on each pioneer
track_width = 0.4  # [m] Distance between the two wheels of pioneer

# Max angular velocity of a single wheel motor
max_wheel_W = 7.7949  # [rad/s] Maximum angular velocity of a single wheel

pioneer_max_W = 0.3 #[rad/seconds] Maximum angular velocity (omega) of pioneer
pioneer_max_V = 0.7 #[m/seconds] Maximum linear velocity (V) of pioneer

goal_threshold = 0.3  # [m] Threshold distance at which robot is declared to be at goal

obstacle_radius = 0.3  # [m] Radius of each obstacle sphere in the shared scene

vision_fov = 0.8727  # [rad] Onboard camera's field of view (~50 deg)
vision_resolution = 256  # [px] Onboard camera's native resolution (square: 256x256)
vision_upscale = 3  # [x] Factor ed5315.sensors.read_camera_image() upscales the native
                    # frame by - CoppeliaSim renders fine detail (e.g. ArUco tags) too
                    # soft to reliably decode at native resolution otherwise
aruco_tag_size = 0.3  # [m] Side length of each obstacle's ArUco tag plane

scan_resolution = 180  # [beams] Number of beams per full lidar sweep

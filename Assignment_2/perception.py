import math
import cv2
import numpy as np

from ed5315 import sim_interface, robot_params
from ed5315.sensors import TrackedObstacle


def detect_obstacles(image, lidar_scan, robot_state, tracked_obstacles):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is None:
        return tracked_obstacles

    current_time = sim_interface.sim_time()
    image_width = image.shape[1]
    image_center = image_width / 2.0
    focal_length = image_center / math.tan(robot_params.vision_fov / 2.0)

    processed_ids = set()

    for marker_corners, marker_id in zip(corners, ids.flatten()):
        marker_id = int(marker_id)

        if marker_id not in {0, 1, 2, 3, 4, 5} or marker_id in processed_ids:
            continue

        processed_ids.add(marker_id)

        points = marker_corners.reshape(4, 2)
        pixel_x = float(np.mean(points[:, 0]))
        bearing = math.atan2(pixel_x - image_center, focal_length)

        lidar_distance = None
        lidar_angle = None
        smallest_angle_error = float("inf")

        for angle, distance in lidar_scan:
            if distance is None:
                continue

            angle_error = (angle - bearing + math.pi) % (2.0 * math.pi) - math.pi

            if abs(angle_error) < smallest_angle_error:
                smallest_angle_error = abs(angle_error)
                lidar_distance = distance
                lidar_angle = angle

        if lidar_distance is None or lidar_angle is None:
            continue

        if smallest_angle_error > math.radians(5.0):
            continue

        print("ID:", marker_id, "bearing:", bearing, "lidar_angle:", lidar_angle, "distance:", lidar_distance)

        local_x = lidar_distance * math.cos(lidar_angle)
        local_y = lidar_distance * math.sin(lidar_angle)

        xr, yr, theta = robot_state
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        world_x = xr + local_x * cos_theta - local_y * sin_theta
        world_y = yr + local_x * sin_theta + local_y * cos_theta

        existing = None

        for obstacle in tracked_obstacles:
            if obstacle.id == marker_id:
                existing = obstacle
                break

        if existing is None:
            tracked_obstacles.append(TrackedObstacle(marker_id, world_x, world_y, 0.0, 0.0, current_time))
        else:
            dt = current_time - existing.detected_time

            if dt > 1e-6:
                velocity_x = (world_x - existing.world_x) / dt
                velocity_y = (world_y - existing.world_y) / dt
            else:
                velocity_x = 0.0
                velocity_y = 0.0

            existing.world_x = world_x
            existing.world_y = world_y
            existing.velocity_x = velocity_x
            existing.velocity_y = velocity_y
            existing.detected_time = current_time

    return tracked_obstacles
import math
import cv2
import numpy as np

from ed5315 import sim_interface, robot_params
from ed5315.sensors import TrackedObstacle


def detect_obstacles(image, lidar_scan, robot_state, tracked_obstacles):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50) #Chose a dictionary with 50 markers, each with a 4x4 grid. This is a good balance between the number of unique markers and the size of the markers.
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #Convert the image to grayscale
    parameters = cv2.aruco.DetectorParameters() #Create a detector parameters object
    detector = cv2.aruco.ArucoDetector(dictionary, parameters) #Create an Aruco detector
    corners, ids, _ = detector.detectMarkers(gray) #Detect the Aruco markers in the image

    # If no markers are detected, return the tracked obstacles
    if ids is None:
        return tracked_obstacles

    current_time = sim_interface.sim_time() #Get the current simulation time
    image_width = image.shape[1] #Get the width of the image
    image_center = image_width / 2.0 #Calculate the center of the image in pixels
    focal_length = image_center / math.tan(robot_params.vision_fov / 2.0) #Calculate the focal length of the camera in pixels

    processed_ids = set() #Keep track of the marker IDs that have already been processed to avoid duplicate processing

    # Loop through the detected markers and process them
    for marker_corners, marker_id in zip(corners, ids.flatten()): #For each detected marker, get the corners and ID
        marker_id = int(marker_id)

        if marker_id not in {0, 1, 2, 3, 4, 5} or marker_id in processed_ids: #If the marker ID is not in the set of valid IDs or has already been processed, skip it
            continue

        processed_ids.add(marker_id) #Add the marker ID to the set of processed IDs

        points = marker_corners.reshape(4, 2) #Reshape the corners to a 4x2 array of points
        pixel_x = float(np.mean(points[:, 0])) #Calculate the average x-coordinate of the marker corners in pixels
        bearing = -math.atan2(pixel_x - image_center, focal_length) #Calculate the bearing of the marker relative to the camera in radians

        lidar_distance = None 
        lidar_angle = None
        smallest_angle_error = float("inf")

        for angle, distance in lidar_scan: #Loop through the lidar scan data, which consists of angle-distance pairs
            if distance is None: #If the distance is None, skip this point
                continue

            angle_error = (angle - bearing + math.pi) % (2.0 * math.pi) - math.pi #Calculate the angle error between the lidar point and the marker bearing, normalized to the range [-pi, pi]

            if abs(angle_error) < smallest_angle_error: #If the absolute angle error is smaller than the smallest angle error found so far, update the smallest angle error and store the corresponding lidar distance and angle
                smallest_angle_error = abs(angle_error)
                lidar_distance = distance
                lidar_angle = angle

        if lidar_distance is None or lidar_angle is None: 
            continue

        if smallest_angle_error > math.radians(5.0):
            continue

        # Convert the lidar distance and angle to world coordinates
        local_x = lidar_distance * math.cos(lidar_angle)
        local_y = lidar_distance * math.sin(lidar_angle)

        # Convert the local coordinates to world coordinates using the robot's state
        xr, yr, theta = robot_state
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        world_x = xr + local_x * cos_theta - local_y * sin_theta
        world_y = yr + local_x * sin_theta + local_y * cos_theta

        # Update the tracked obstacles  
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

import math
import numpy as np
from ed5315 import sim_interface, robot_params
import odometry

covariance = None
previous_time = None

def wrap_to_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

def estimate_pose(robot_state, Vl, Vr, tracked_obstacles, ground_truth_map):
    global covariance, previous_time # Global variables for the EKF

    current_time = sim_interface.sim_time() # Get the current simulation time

    old_time = previous_time # Store the previous time for the EKF update
    previous_time = current_time # Update the previous time to the current time for the next iteration

    # Initialize the covariance matrix if it is None
    if covariance is None:
        covariance = np.diag([1e-4, 1e-4, 1e-4])
    
    x, y, theta = robot_state # Unpack the robot's current state (x, y, theta)
    predicted_state = odometry.estimate_pose(robot_state, Vl, Vr) # Predict the new state based on odometry
    xp, yp, thetap = predicted_state # Unpack the predicted state (xp, yp, thetap)

    dt = 0.0 if old_time is None else max(0.0, current_time - old_time) 

    # Update the covariance matrix based on the motion model
    r = robot_params.wheel_radius
    L = robot_params.track_width

    V = (Vl + Vr) * r / 2.0 # Calculate the linear velocity of the robot based on wheel velocities and wheel radius

    F = np.array([[1.0, 0.0, -V * math.sin(theta) * dt], [0.0, 1.0, V * math.cos(theta) * dt], [0.0, 0.0, 1.0]]) # State transition matrix based on the motion model

    G = np.array([[0.5 * r * math.cos(theta) * dt, 0.5 * r * math.cos(theta) * dt], [0.5 * r * math.sin(theta) * dt, 0.5 * r * math.sin(theta) * dt], [-r * dt / L, r * dt / L]]) # Control input matrix based on the motion model
    
    sigma_w = robot_params.wheel_velocity_noise_std #Wheel velocity noise standard deviation
   
    Q = np.diag([sigma_w**2, sigma_w**2]) # Process noise covariance matrix based on the wheel velocity noise
    
    covariance = F @ covariance @ F.T + G @ Q @ G.T # Update the covariance matrix based on the motion model and process noise
    covariance = 0.5 * (covariance + covariance.T) # Ensure the covariance matrix is symmetric

    # Update the state estimate based on the observations of tracked obstacles
    if old_time is None:
        return predicted_state
    
    R = np.diag([robot_params.ekf_range_noise_std**2, robot_params.ekf_bearing_noise_std**2]) # Measurement noise covariance matrix

    for obs in tracked_obstacles: # Iterate through each tracked obstacle to update the state estimate based on the observations
        if abs(obs.detected_time - old_time) > 1e-6:
            continue
        landmark = ground_truth_map.get(obs.id)

        if landmark is None: # If the landmark is not found in the ground truth map, skip this observation
            continue

        lx, ly = landmark # Get the landmark's position from the ground truth map
        dx_meas = obs.world_x - x # Calculate the difference in x between the observed landmark and the robot's current position
        dy_meas = obs.world_y - y # Calculate the difference in y between the observed landmark and the robot's current position
        measured_range = math.hypot(dx_meas, dy_meas) # Calculate the measured range to the observed landmark

        if measured_range < 1e-6: # If the measured range is too small (indicating the robot is too close to the landmark), skip this observation
            continue

        measured_bearing = wrap_to_pi(math.atan2(dy_meas, dx_meas) - theta) # Calculate the measured bearing to the observed landmark
        dx = lx - xp # Calculate the difference in x between the landmark's position and the predicted robot position
        dy = ly - yp # Calculate the difference in y between the landmark's position and the predicted robot position
        q = dx * dx + dy * dy # Calculate the squared distance between the landmark's position and the predicted robot position

        if q < 1e-12:
            continue

        predicted_range = math.sqrt(q) # Calculate the predicted range to the landmark based on the predicted robot position

        predicted_bearing = wrap_to_pi(math.atan2(dy, dx) - thetap) # Calculate the predicted bearing to the landmark based on the predicted robot position

        z = np.array([measured_range, measured_bearing]) # Create the measurement vector based on the observed range and bearing to the landmark
        h = np.array([predicted_range, predicted_bearing]) # Create the predicted measurement vector based on the predicted range and bearing to the landmark
 
        innovation = z - h # Calculate the innovation (difference between the observed and predicted measurements)
        innovation[1] = wrap_to_pi(innovation[1]) # Wrap the innovation in bearing to the range [-pi, pi]

        H = np.array([[-dx / predicted_range, -dy / predicted_range, 0.0], [dy / q, -dx / q, -1.0]]) # Calculate the Jacobian matrix of the measurement model based on the predicted robot position and landmark position

        S = H @ covariance @ H.T + R # Calculate the innovation covariance matrix based on the predicted covariance and measurement noise

        K = np.linalg.solve(S, H @ covariance).T # Calculate the Kalman gain based on the innovation covariance and predicted covariance

        correction = K @ innovation # Calculate the correction to the predicted state based on the Kalman gain and innovation

        xp += correction[0] # Update the predicted x position based on the correction  
        yp += correction[1] # Update the predicted y position based on the correction

        thetap = wrap_to_pi(thetap + correction[2])

        I = np.eye(3) # Create the identity matrix for the state dimension
        A = I - K @ H # Calculate the updated covariance matrix based on the Kalman gain and Jacobian matrix

        covariance = A @ covariance @ A.T + K @ R @ K.T # Update the covariance matrix based on the Kalman gain, Jacobian matrix, and measurement noise
        covariance = 0.5 * (covariance + covariance.T) # Ensure the covariance matrix is symmetric
        
    return [xp, yp, wrap_to_pi(thetap)]

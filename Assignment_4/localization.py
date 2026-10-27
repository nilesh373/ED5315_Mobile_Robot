
import math
import numpy as np
from ed5315 import sim_interface, robot_params
import odometry

covariance = None
previous_time = None

def wrap_to_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

def estimate_pose(robot_state, Vl, Vr, tracked_obstacles, ground_truth_map):
    global covariance, previous_time
    current_time = sim_interface.sim_time()
    old_time = previous_time
    previous_time = current_time
    if covariance is None:
        covariance = np.diag([1e-4, 1e-4, 1e-4])
    x, y, theta = robot_state
    predicted_state = odometry.estimate_pose(robot_state, Vl, Vr)
    xp, yp, thetap = predicted_state
    dt = 0.0 if old_time is None else max(0.0, current_time - old_time)
    r = robot_params.wheel_radius
    L = robot_params.track_width
    V = (Vl + Vr) * r / 2.0
    F = np.array([[1.0, 0.0, -V * math.sin(theta) * dt], [0.0, 1.0, V * math.cos(theta) * dt], [0.0, 0.0, 1.0]])
    G = np.array([[0.5 * r * math.cos(theta) * dt, 0.5 * r * math.cos(theta) * dt], [0.5 * r * math.sin(theta) * dt, 0.5 * r * math.sin(theta) * dt], [-r * dt / L, r * dt / L]])
    sigma_w = robot_params.wheel_velocity_noise_std
    Q = np.diag([sigma_w**2, sigma_w**2])
    covariance = F @ covariance @ F.T + G @ Q @ G.T
    covariance = 0.5 * (covariance + covariance.T)
    if old_time is None:
        return predicted_state
    R = np.diag([robot_params.ekf_range_noise_std**2, robot_params.ekf_bearing_noise_std**2])
    for obs in tracked_obstacles:
        if abs(obs.detected_time - old_time) > 1e-6:
            continue
        landmark = ground_truth_map.get(obs.id)
        if landmark is None:
            continue
        lx, ly = landmark
        dx_meas = obs.world_x - x
        dy_meas = obs.world_y - y
        measured_range = math.hypot(dx_meas, dy_meas)
        if measured_range < 1e-6:
            continue
        measured_bearing = wrap_to_pi(math.atan2(dy_meas, dx_meas) - theta)
        dx = lx - xp
        dy = ly - yp
        q = dx * dx + dy * dy
        if q < 1e-12:
            continue
        predicted_range = math.sqrt(q)
        predicted_bearing = wrap_to_pi(math.atan2(dy, dx) - thetap)
        z = np.array([measured_range, measured_bearing])
        h = np.array([predicted_range, predicted_bearing])
        innovation = z - h
        innovation[1] = wrap_to_pi(innovation[1])
        H = np.array([[-dx / predicted_range, -dy / predicted_range, 0.0], [dy / q, -dx / q, -1.0]])
        S = H @ covariance @ H.T + R
        K = np.linalg.solve(S, H @ covariance).T
        correction = K @ innovation
        xp += correction[0]
        yp += correction[1]
        thetap = wrap_to_pi(thetap + correction[2])
        I = np.eye(3)
        A = I - K @ H
        covariance = A @ covariance @ A.T + K @ R @ K.T
        covariance = 0.5 * (covariance + covariance.T)
    return [xp, yp, wrap_to_pi(thetap)]

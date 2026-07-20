"""
Raw sensor access only - no obstacle/goal interpretation logic here.

Turning a raw lidar reading or camera image into "obstacle at range r,
bearing theta" is an assignment task.
"""

import math

import cv2
import numpy as np

from . import robot_params


def read_lidar(sim_interface, num_beams=robot_params.scan_resolution):
    # The Hokuyo's own built-in sweep script was removed from the scene. This
    # drives the joint directly instead: set an angle, force a fresh reading
    # with handleProximitySensor, repeat around a full circle.
    sim = sim_interface._sim
    joint = sim_interface._h['lidar_joint']
    laser = sim_interface._h['lidar']

    scan = []
    for i in range(num_beams):
        angle = -math.pi + i * (2 * math.pi / num_beams)
        sim.setJointPosition(joint, angle)
        res, dist, point, obj, normal = sim.handleProximitySensor(laser)
        scan.append((angle, dist if res else None))
    return scan


def read_wheel_velocities(sim_interface):
    # Actual joint velocity, not the commanded one - the motor doesn't track
    # a target instantly, and real wheel encoders aren't exact either, so a
    # small Gaussian noise term is added to each reading.
    sim = sim_interface._sim
    Vl = sim.getJointVelocity(sim_interface._h['left_motor'])
    Vr = sim.getJointVelocity(sim_interface._h['right_motor'])
    Vl += float(np.random.normal(0.0, robot_params.wheel_velocity_noise_std))
    Vr += float(np.random.normal(0.0, robot_params.wheel_velocity_noise_std))
    return Vl, Vr


def read_camera_image(sim_interface):
    img, resolution = sim_interface._sim.getVisionSensorImg(sim_interface._h['vision'])
    arr = np.array(sim_interface._sim.unpackUInt8Table(img), dtype=np.uint8)
    arr = arr.reshape(resolution[1], resolution[0], 3)

    # CoppeliaSim's vision sensor buffer is bottom-to-top
    arr = np.flipud(arr)

    # Upscale - CoppeliaSim renders fine detail (e.g. ArUco tags) too soft to
    # reliably decode at the sensor's native resolution.
    scale = robot_params.vision_upscale
    new_size = (arr.shape[1] * scale, arr.shape[0] * scale)
    arr = cv2.resize(arr, new_size, interpolation=cv2.INTER_CUBIC)
    resolution = (arr.shape[1], arr.shape[0])

    return arr, resolution

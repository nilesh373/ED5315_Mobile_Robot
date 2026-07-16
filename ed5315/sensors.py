"""
Raw sensor access only - no interpretation/detection logic here.

Turning a raw lidar reading or camera image into "obstacle at range r,
bearing theta" is an assignment task, not shared infrastructure.
"""

import math

import numpy as np


def read_lidar(sim_interface, num_beams=180):
    # The Hokuyo's own built-in sweep script was removed from the scene (it
    # never exposed its scan data externally - see session notes). This
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


def read_camera_image(sim_interface):
    img, resolution = sim_interface._sim.getVisionSensorImg(sim_interface._h['vision'])
    arr = np.array(sim_interface._sim.unpackUInt8Table(img), dtype=np.uint8)
    arr = arr.reshape(resolution[1], resolution[0], 3)
    # Note: CoppeliaSim vision sensor images are commonly returned bottom-to-top.
    # Verify orientation visually before relying on pixel-row order.
    return arr, resolution

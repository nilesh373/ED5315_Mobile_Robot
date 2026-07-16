"""
ED5315 CoppeliaSim interface - ZeroMQ remote API.

Object paths below were confirmed against the real scene.
"""

from . import robot_params

_client = None
_sim = None
_h = {}  # object handles, keyed by name

_NUM_OBSTACLES = 6  # /Obstacle0 .. /Obstacle5, see scene documentation


def sim_init():
    global _client, _sim
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient

    try:
        _client = RemoteAPIClient()  # defaults: localhost:23000
        _sim = _client.require('sim')
    except Exception as exc:
        print(f"Could not connect to CoppeliaSim: {exc}")
        return False
    return True


def get_handles():
    _h['pioneer'] = _sim.getObject('/Pioneer1')
    _h['left_motor'] = _sim.getObject('/Pioneer1/Pioneer1_left')
    _h['right_motor'] = _sim.getObject('/Pioneer1/Pioneer1_right')
    _h['goal'] = _sim.getObject('/Sphere')
    _h['lidar'] = _sim.getObject('/Pioneer1/Hokuyo/joint/laser')
    _h['lidar_joint'] = _sim.getObject('/Pioneer1/Hokuyo/joint')
    _h['vision'] = _sim.getObject('/Pioneer1/visionSensor')
    _h['obstacles'] = [_sim.getObject('/Obstacle%d' % i) for i in range(_NUM_OBSTACLES)]


def start_simulation():
    _sim.setStepping(True)  # synchronous stepping - the point of this port
    res = _sim.startSimulation()
    return True


def step():
    _sim.step()


def sim_time():
    return _sim.getSimulationTime()


def localize_robot():
    # PS. THE ORIENTATION WILL BE RETURNED IN RADIANS
    pos = _sim.getObjectPosition(_h['pioneer'], -1)
    orient = _sim.getObjectOrientation(_h['pioneer'], -1)
    return [pos[0], pos[1], orient[2]]


def get_goal_pose():
    # PS. THE ORIENTATION WILL BE RETURNED IN RADIANS
    pos = _sim.getObjectPosition(_h['goal'], -1)
    orient = _sim.getObjectOrientation(_h['goal'], -1)
    return [pos[0], pos[1], orient[2]]


def get_obstacle_positions():
    # Ground-truth (x, y) position of every obstacle in the scene.
    return [_sim.getObjectPosition(h, -1)[:2] for h in _h['obstacles']]


def setvel_pioneers(Vl, Vr):
    # Takes raw left/right wheel angular velocities [rad/s] directly 
    Vl = max(min(Vl, robot_params.max_wheel_W), -robot_params.max_wheel_W)
    Vr = max(min(Vr, robot_params.max_wheel_W), -robot_params.max_wheel_W)
    _sim.setJointTargetVelocity(_h['left_motor'], Vl)
    _sim.setJointTargetVelocity(_h['right_motor'], Vr)


def sim_shutdown():
    _sim.stopSimulation()

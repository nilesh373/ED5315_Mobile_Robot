# Demo

Compatibility check / sensor smoke test — confirms your setup works before you start an
assignment. See the [repository root README](../README.md) for setup and installation.

## Usage:

  1. Complete the installation steps in the root README, then launch CoppeliaSim and open
  `scenes/mobile_robot.ttt`. Run the simulation by clicking the play button, or let
  `main.py` start it for you.

  2. From the repository root, run `python Demo/main.py`.

  You should see the robot drive forward, turn, and print its pose each time, followed by a lidar range reading and the camera image's resolution/array shape. This is just a compatibility/sensor smoke test, not a graded assignment.

## Working explained:

### main.py
Sets up the simulation and drives/reads sensors using the shared `ed5315` package.

### ed5315.sim_interface
Connects to CoppeliaSim over the [ZeroMQ remote API](https://manual.coppeliarobotics.com/en/zmqRemoteApiOverview.htm), obtains handles for objects in the scene, reads the position/orientation of the robot and goal, sets wheel velocities, and steps the simulation synchronously. Key functions:

#### sim_init()
Connects to the running CoppeliaSim instance's ZeroMQ remote API server.

#### get_handles()
To interact with any object in Coppeliasim we first need its handle. This is called once after connecting.

#### start_simulation()
Starts the simulation with synchronous stepping enabled.

#### step()
Advances the simulation by exactly one timestep. Callers that need "run for N seconds of sim time" loop on `step()` and `sim_time()` deltas rather than sleeping.

### ed5315.sensors
Raw lidar and camera access (`read_lidar`, `read_camera_image`). 

### ed5315.robot_params
Stores the physical parameter values (wheel radius, track width, velocity limits) that are important for the simulation.

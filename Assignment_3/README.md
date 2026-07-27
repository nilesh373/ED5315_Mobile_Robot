# ED5315 Assignment-3
Wheel odometry for a differential-drive mobile robot (Pioneer p3dx) in CoppeliaSim - the
robot no longer knows its own pose from simulation and must estimate it by dead-reckoning
from its wheel velocities instead.

## Setup
See the [repository root README](../README.md) for installation and setup.

## What's different from Assignment 1/2

The goal is still ground truth (`sim_interface.get_goal_pose()`, exactly like Assignment
1/2), and obstacles are still detected via `perception.py` exactly like Assignment 2. The
one thing that's changed is the robot's own pose: `main.py` calls
`sim_interface.localize_robot()` **once**, at the very start, to seed the robot's initial
pose - after that, `robot_state` is never queried from the simulator again. Every loop
iteration, it's instead updated by your own **estimate_pose** in **odometry.py**, from the
robot's wheel velocities:
- `Vl`, `Vr`: left/right wheel angular velocities [rad/s], from
  `ed5315.sensors.read_wheel_velocities`. These are **not** the commanded velocity - they
  carry simulated encoder noise (`robot_params.wheel_velocity_noise_std`), so your pose
  estimate will drift from the truth over time.

Everything downstream (`perception.py`'s world-coordinate conversion, `control.py`'s
`gtg`/`avoid_obstacles`) just takes `robot_state` as a plain `[x, y, theta]`.

## Task: Odometry

Complete **estimate_pose** in **odometry.py** - this is the file you submit (see
Submission below). Convert `Vl`/`Vr` to linear wheel speeds via `robot_params.wheel_radius`,
combine them into the robot's linear/angular velocity via `robot_params.track_width`, then
integrate `robot_state`'s `x`/`y`/`theta` forward by the elapsed time `dt` since your last
call. Source `dt` yourself via `ed5315.sim_interface.sim_time()` (same pattern as
Assignment 1/2's `gtg`) - it is not passed in as a parameter.

## control.py / perception.py

Both carried over unchanged from Assignment 1/2 - same function names, signatures, and
`dt`-handling. Neither is graded here; see Submission below. Do not make any changes to
`main.py`.

## Submission

**odometry.py** is the only file you submit, and the only one that's graded -
`control.py`/`perception.py` in this folder are *not* evaluated, they're only here so you
can run and test `main.py` locally end-to-end. Paste your own working **control.py** from
Assignment 1/2 and **perception.py** from Assignment 2 into this folder.

## Instructions

  1. Download the setup provided in this repository (or `git pull` if you already have it).

  2. Copy your working **control.py** and **perception.py** from Assignment 1/2 over this
     folder's copies.

  3. Complete **estimate_pose** in **odometry.py** - this is the file you'll submit.

  4. Launch CoppeliaSim. Click File -> Open Scene, and open the shared
     [`scenes/mobile_robot.ttt`](../scenes/mobile_robot.ttt). Run the simulation with the play button.

  5. Run `main.py` from this folder (or `python Assignment_3/main.py` from the repository
     root).

  6. Always ensure the simulation is running before you launch the code, otherwise you'll
     get "Failed connecting to remote API server."

  7. If your implementation is correct, the robot will still find its way to the goal
     while avoiding obstacles, but - because it's navigating off its own odometry estimate
     instead of ground truth - it may not stop exactly on the true goal position.

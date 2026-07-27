# ED5315 Assignment-1
Known-map navigation of a differential-drive mobile robot (Pioneer p3dx) in CoppeliaSim,
with obstacle avoidance against a bounded-range obstacle sensing radius.

## Setup
See the [repository root README](../README.md) for installation and setup.

## Task 1: Go-to-goal control

Complete **at_goal**, **gtg**, and **differential_drive_ik** in **control.py**.

- **at_goal(robot_state, goal_state)** returns whether the robot is within
  `robot_params.goal_threshold` of the goal.
- **gtg(robot_state, goal_state)** is the go-to-goal controller: a PD controller for
  angular velocity and a separate P controller for linear velocity, returning `[V, W]`.
- **differential_drive_ik(V, W)** must first clamp `V`/`W` to `robot_params.pioneer_max_V`/
  `pioneer_max_W`, then convert them into left/right wheel angular velocities using
  `robot_params.wheel_radius` and `robot_params.track_width`. Clamp before converting, not
  after — a large, unclamped `V` dominates both wheels' target speed, and once both
  saturate at `setvel_pioneers`'s per-wheel `max_wheel_W` limit they clamp to the *same*
  value, silently erasing whatever turn `W` was supposed to produce.

## Task 2: Reactive obstacle avoidance

Complete **avoid_obstacles** and **navigation_state_machine** in **control.py**. The robot
always knows its own pose (from simulation) and the goal pose, but obstacle positions are
only reported within a limited sensing radius: each control-loop iteration, `main.py`
reports the ground-truth `(x, y)` position of every obstacle *currently* within
`robot_params.obstacle_sense_radius` (2 m) of the robot, as `nearby_obstacles`. This is **not** a
memory — it's recomputed fresh every iteration, so an obstacle drops back out again as
soon as the robot moves away from it, and `nearby_obstacles` may be empty. Every obstacle
has radius `robot_params.obstacle_radius`.

- **avoid_obstacles(robot_state, nearby_obstacles)** is a reactive controller that steers
  the robot away from the obstacles currently within sensing range, returning `[V, W]`.
- **navigation_state_machine(robot_state, goal_state, nearby_obstacles)** is what actually
  drives the robot — `main.py` calls it instead of `gtg` directly, then passes its
  `[V, W]` output through your own `differential_drive_ik`. It's meant to switch between
  `gtg()` and `avoid_obstacles()` as a state machine (e.g. based on whether
  `nearby_obstacles` is empty, or how close the nearest one currently is) rather than
  blend them — design your own switching logic.

## Submission

**control.py** is the only file you submit, and the only one that's graded. Do not modify `main.py` or any other provided file.

## Instructions

  1. Download the setup provided in this repository (or `git pull` if you already have it).

  2. Complete **at_goal**, **gtg**, **differential_drive_ik**, **avoid_obstacles**, and
     **navigation_state_machine** in **control.py**.

  3. Launch CoppeliaSim. Click File -> Open Scene, and open the shared
     [`scenes/mobile_robot.ttt`](../scenes/mobile_robot.ttt). Run the simulation with the play button.

  4. Run `main.py` from this folder (or `python Assignment_1/main.py` from the repository
     root).

  5. Always ensure the simulation is running before you launch the code, otherwise you'll
     get "Failed connecting to remote API server."

  6. If your implementation is correct, the robot will drive itself to the goal sphere
     while steering around obstacles as it comes within sensing range of them.

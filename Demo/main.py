#!/usr/bin/env python

"""
ED5315 compatibility check / sensor smoke test.
"""

# Import files
from ed5315 import sim_interface, sensors, robot_params


def run_for(duration):
    # Step the simulation forward by 'duration' seconds of SIM time
    t0 = sim_interface.sim_time()
    while sim_interface.sim_time() - t0 < duration:
        sim_interface.step()


def main():
    if (sim_interface.sim_init()):

        # Obtain handles to sim elements
        sim_interface.get_handles()

        # Start simulation
        if (sim_interface.start_simulation()):

            # Stop robot
            sim_interface.setvel_pioneers(0.0, 0.0)

            # Obtain goal state
            goal_state = sim_interface.get_goal_pose()
            print("Goal pose", goal_state)

            # Obtain robots position
            robot_state = sim_interface.localize_robot()
            print("Robot pose", robot_state)

            # Drive forward: same velocity on both wheels
            w = 0.5 * robot_params.max_wheel_W
            sim_interface.setvel_pioneers(w, w)
            run_for(2.0)
            # Stop robot
            sim_interface.setvel_pioneers(0.0, 0.0)
            print("New robot pose", sim_interface.localize_robot())

            # turn: opposite velocity on each wheel
            sim_interface.setvel_pioneers(w, -w)
            run_for(2.0)
            # Stop robot
            sim_interface.setvel_pioneers(0.0, 0.0)
            print("New robot pose", sim_interface.localize_robot())

            # Sensor smoke test: lidar + camera
            scan = sensors.read_lidar(sim_interface)
            hits = [d for _, d in scan if d is not None]
            closest = f"{min(hits):.3f} m" if hits else "none"
            print(f"Lidar scan: {len(scan)} beams, {len(hits)} detections, closest {closest}")

            image, resolution = sensors.read_camera_image(sim_interface)
            print("Camera resolution:", resolution, "image array shape/dtype:", image.shape, image.dtype)

        else:
            print('Failed to start simulation')
    else:
        print('Failed connecting to remote API server')

    # stop robot
    sim_interface.setvel_pioneers(0.0, 0.0)
    sim_interface.sim_shutdown()
    return


# run
if __name__ == '__main__':

    main()
    print('Program ended')

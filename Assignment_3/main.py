#!/usr/bin/env python

"""
Mobile robot simulation setup
@author: Bijo Sebastian
"""

#Import files
from ed5315 import sim_interface, sensors, plotting, robot_params
import perception
import control
import odometry

def main():
    if (sim_interface.sim_init()):

        #Obtain handles to sim elements
        sim_interface.get_handles()

        #Start simulation
        if (sim_interface.start_simulation()):

            #Stop robot
            sim_interface.setvel_pioneers(0.0, 0.0)

            #Obtain goal position (ground truth - same as Assignment 1/2)
            goal_state = sim_interface.get_goal_pose()

            #Obtain robot's own STARTING pose only (ground truth) - from here
            #on robot_state is your own odometry estimate, never queried from
            #simulation again
            robot_state = sim_interface.localize_robot()

            #Your own obstacle-tracking state - starts empty, a list of
            #ed5315.sensors.TrackedObstacle
            tracked_obstacles = []

            #Record both paths for the result plot at the end - true_path is
            #ground truth, queried purely for comparison/plotting and never
            #fed into navigation; est_path is what the robot actually
            #steers by (odometry.estimate_pose's output)
            true_path = [robot_state[:2]]
            est_path = [robot_state[:2]]

            while not control.at_goal(robot_state, goal_state):

                #Raw sensor reads
                image, _ = sensors.read_camera_image(sim_interface)
                lidar_scan = sensors.read_lidar(sim_interface)

                #Detect obstacles from the raw sensor data
                tracked_obstacles = perception.detect_obstacles(image, lidar_scan, robot_state, tracked_obstacles)

                [V, W] = control.navigation_state_machine(robot_state, goal_state, tracked_obstacles)
                [Vl, Vr] = control.differential_drive_ik(V, W)
                sim_interface.setvel_pioneers(Vl, Vr)

                #step the simulation forward one timestep
                sim_interface.step()
                true_path.append(sim_interface.localize_robot()[:2])

                #Estimate the robot's new pose from wheel odometry - this
                #replaces sim_interface.localize_robot() from here on
                Vl_actual, Vr_actual = sensors.read_wheel_velocities(sim_interface)
                robot_state = odometry.estimate_pose(robot_state, Vl_actual, Vr_actual)
                est_path.append(robot_state[:2])

            #Stop robot
            sim_interface.setvel_pioneers(0.0, 0.0)

            #Plot the map (boundary, ground-truth obstacles, detected
            #obstacles, goal) and both paths followed
            fig, ax = plotting.new_plot()
            plotting.draw_boundary(ax)
            plotting.draw_obstacles(ax, sim_interface.get_obstacle_positions(), radius=robot_params.obstacle_radius,
                                     color='red', label='Obstacles (ground truth)')
            detected_positions = [[o.world_x, o.world_y] for o in tracked_obstacles]
            plotting.draw_obstacles(ax, detected_positions, color='orange', marker='x',
                                     label='Obstacles (detected)')
            plotting.draw_goal(ax, goal_state)
            plotting.draw_path(ax, true_path, color='blue', label='Ground-truth path')
            plotting.draw_path(ax, est_path, color='purple', linestyle='--', label='Odometry-estimated path')
            plotting.finish_plot(ax, 'Assignment 3: wheel odometry', 'Assignment_3/result.png')

        else:
            print('Failed to start simulation')
    else:
        print('Failed connecting to remote API server')

    sim_interface.setvel_pioneers(0.0, 0.0)
    sim_interface.sim_shutdown()
    return

#run
if __name__ == '__main__':

    main()
    print('Program ended')

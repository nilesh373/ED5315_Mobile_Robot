#!/usr/bin/env python

"""
Mobile robot simulation setup
@author: Bijo Sebastian
"""

#Import files
from ed5315 import sim_interface, sensors
import perception
import control

def main():
    if (sim_interface.sim_init()):

        #Obtain handles to sim elements
        sim_interface.get_handles()

        #Start simulation
        if (sim_interface.start_simulation()):

            #Stop robot
            sim_interface.setvel_pioneers(0.0, 0.0)

            #Obtain goal position (ground truth - same as Assignment 1)
            goal_state = sim_interface.get_goal_pose()

            #Obtain robots position (ground truth - still given this assignment)
            robot_state = sim_interface.localize_robot()

            #Your own obstacle-tracking state - starts empty, a list of
            #perception.TrackedObstacle
            tracked_obstacles = []

            while not control.at_goal(robot_state, goal_state):

                #Raw sensor reads - no obstacle ground truth this assignment
                image, _ = sensors.read_camera_image(sim_interface)
                lidar_scan = sensors.read_lidar(sim_interface)

                #Detect obstacles from the raw sensor data
                tracked_obstacles = perception.detect_obstacles(image, lidar_scan, robot_state, tracked_obstacles)

                [V, W] = control.navigation_state_machine(robot_state, goal_state, tracked_obstacles)
                [Vl, Vr] = control.differential_drive_ik(V, W)
                sim_interface.setvel_pioneers(Vl, Vr)

                #step the simulation forward one timestep
                sim_interface.step()
                robot_state = sim_interface.localize_robot()

            #Stop robot
            sim_interface.setvel_pioneers(0.0, 0.0)

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

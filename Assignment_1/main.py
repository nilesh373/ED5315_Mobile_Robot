#!/usr/bin/env python

"""
Mobile robot simulation setup
@author: Bijo Sebastian
"""

import math

#Import files
from ed5315 import sim_interface, plotting, robot_params
import control

def get_nearby_obstacles(robot_state, obstacle_positions):
    #Simulates a bounded-range sensing radius: only obstacles currently within
    #robot_params.obstacle_sense_radius of the robot are reported.
    nearby = []
    for obs in obstacle_positions:
        d = math.hypot(obs[0] - robot_state[0], obs[1] - robot_state[1])
        if d <= robot_params.obstacle_sense_radius:
            nearby.append(obs)
    return nearby

def main():
    if (sim_interface.sim_init()):

        #Obtain handles to sim elements
        sim_interface.get_handles()

        #Start simulation
        if (sim_interface.start_simulation()):

            #Stop robot
            sim_interface.setvel_pioneers(0.0, 0.0)

            #Obtain goal state (this gives us the co-ordinates of the goal sphere)
            goal_state = sim_interface.get_goal_pose()

            #Obtain robots position
            robot_state = sim_interface.localize_robot()

            #Record the path followed, for the result plot at the end
            path = [robot_state[:2]]

            while not control.at_goal(robot_state, goal_state):

                obstacle_positions = sim_interface.get_obstacle_positions()
                nearby_obstacles = get_nearby_obstacles(robot_state, obstacle_positions)

                [V, W] = control.navigation_state_machine(robot_state, goal_state, nearby_obstacles)
                [Vl, Vr] = control.differential_drive_ik(V, W)
                sim_interface.setvel_pioneers(Vl, Vr)

                #step the simulation forward one timestep
                sim_interface.step()
                robot_state = sim_interface.localize_robot()
                path.append(robot_state[:2])

            #Stop robot
            sim_interface.setvel_pioneers(0.0, 0.0)

            #Plot the map (boundary, obstacles, goal) and the path followed
            fig, ax = plotting.new_plot()
            plotting.draw_boundary(ax)
            plotting.draw_obstacles(ax, sim_interface.get_obstacle_positions(), radius=robot_params.obstacle_radius)
            plotting.draw_goal(ax, goal_state)
            plotting.draw_path(ax, path, label='Robot path')
            plotting.finish_plot(ax, 'Assignment 1: known-map navigation', 'Assignment_1/result.png')

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

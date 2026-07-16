> **Stale — pending rewrite.** This README (and the ultrasonic-based `navigate()` task
> below) describes the pre-redesign Assignment 1. The shared scene now used by this
> assignment (`scenes/mobile_robot.ttt`) has no ultrasonic sensors and adds red tagged
> obstacles + a lidar + a vision sensor instead. Only the scene file path below has been
> updated so the Demo/compatibility instructions still work; the task description itself
> is not yet rewritten for the new arc.

# ED5315 Assignment-1
Waypoint control of a mobile robot(differential drive) in Coppleiasim(V-REP), with reactive obstacle avoidance

## Setup:
OS: Windows 10/11; Ubuntu 20.04


Python: 3.6.x
Coppeliasim: V4.3.0

To check the compatibility of your system, follow the instructions [here](https://github.com/BijoSebastian/ED5315_Mobile_Robot_Sim_Setup/tree/main/Demo) and run the demo script.

## Task 1: Go-to-goal control

Complete **at_goal** and **gtg** in **control.py** to drive the robot to the goal sphere using a PD heading controller and P linear-velocity control. Do not make any changes to the other code files provided to you.

## Task 2: Reactive obstacle avoidance

Complete **navigate** in **control.py**. Pioneer1 is equipped with a 16-sensor ultrasonic ring (front 8 + rear 8, the standard Pioneer_p3dx sonar layout). Each control-loop iteration, `main.py` calls `sim_interface.read_proximity_sensors()` and passes you the result as `sensor_data`: a list of 16 values, ordered per `robot_params.pioneer_sensor_names` (sensor 1 first), where each entry is either a distance in metres or `None` if that sensor detects nothing.

`navigate(robot_state, goal_state, sensor_data)` is what actually drives the robot — `main.py` calls it instead of `gtg` directly. How you combine goal-seeking with obstacle avoidance (potential fields, a switching strategy, or anything else) is entirely up to you; you're free to call your own `gtg` from inside it. The exact mounting angle of each sensor around the ring isn't given here — if you need it, check the sensor positions directly in the CoppeliaSim scene.

## Instructions:

  1. Download the setup provided in this repository. If you are familiar with how to use git on windows do that, if not click on the green button that says code and click on download zip. Once the download is complete, double click to extract the contents and place them in a location of your choice, the downloads folder itself works fine.

  2. Complete **at_goal**, **gtg**, and **navigate** in the file **control.py**. Do not make any changes to the other code files provided to you.

  3. Once you have completed the go to goal and obstacle avoidance implementation, launch Coppeliasim. Click on File->Open Scene. Navigate to the downloaded setup's **`scenes`** folder and select the file **`mobile_robot.ttt`** (this scene is shared across all assignments, not copied per-assignment). Run the simulation by clicking on the light blue play button.

  4. Launch Spyder. Click on File -> Open and navigate to the downloaded setup. Select the file main.py, run it by clicking on the green play button. (Ensure you are in the right folder, Assignment-1 in this case) 

  5. Always ensure that the simulation is running before you launch the code, otherwise you will get an error that says **"Failed connecting to the remote API server. Program ended"**.

  6.	If your implementation is correct, the robot will drive itself to the goal sphere while steering around the maze walls, as shown below:

## Solution video:
The goal pose moves randomly every time a new instance is launched.

![Solution run 1](solution/Solution1.gif)
![Solution run 2](solution/Solution2.gif)

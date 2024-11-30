# ucf_go1_util

This package provides utilities such as python scripts for use with the go1 robot

## function_generator.py
This class publishes sinusoidal joint trajectories for each of the 12 go1 joints
By default, the script publishes a sinusoid that operates over the joints limits centered
at the center of those limits. It publishes a sinusoid over a duration and then updates the trajectory
at some rate.

Example (remaps the default command output to gazebo command)
```
roslaunch ucf_go1_bringup gazeboSim.launch
rosrun ucf_go1_util function_generator.py /traj_controller/command:=/go1_gazebo/traj_controller/command
```
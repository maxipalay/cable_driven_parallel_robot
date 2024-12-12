# Cable driven parallel robot

<p align="center">
  <img src="https://github.com/user-attachments/assets/eb40a965-1b77-451c-8792-6edabf923432" />
</p>

This is the repository for the code running a custom Cable Driven Parallel Robot (CDPR). The code is to be run on to two machines, a computer where the ROS2 code runs and a microcontroller (Teensy 4.1) where lower level communications with the motor drives (Odrives) happens.

## Repository contents

- `arduino/cdpr_interface_ff` - this folder contains the set of files to be compiled and uploaded to the Teensy 4.1 running on the robot.
- `cdpr_bringup` - ROS2 package with launch files and configuration files to launch the ROS2 system.
- `cdpr_driver` - ROS2 package that handles the interfacing between the microcontroller and the ROS2 infrastructure.
- `cdpr_kinematics` - ROS2 package that performs the inverse kinematics and feedforward torque calculation based on target end effector position.
- `cdpr_kinematics_interface` - ROS2 package with custom message definition for the commands and feedback of the robot.

## How to run

Running the system should be as simple as plugging in the ROS2-enabled computer to the microcontroller, building the workspace, sourcing it and entering the command:

`ros2 launch cdpr_bringup launch_cdpr.launch.py`

This will launch the `kinematics` node, the `driver` node, and `rviz2` with the proper configuration file to visualize robot frames and markers for the inverse kinematics cables calculations.

If this is all running, the robot is listening for commands on the `/ik_request` topic. As soon as a message is published on that topic, the inverse kinematics will be calculated and the robot will attempt to achieve the target position as quickly as possible.
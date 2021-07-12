# UR5-ROS

## About
Contains ROS packages handling the ROS interface with UR5.

## Setting up the interface

### Method 1: Easy way
Use the python script that will be developed in the future

### Method 2: Manual way
1. 
    ```bash
    roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.0.100 kinematics_config:=$HOME/my_robot_calibration.yaml
    ```
2. Load the `ros_interface.urp` program on the UR5 polyscope.

3. 
    ```bash
    roslaunch ur_calibration calibration_correction.launch  robot_ip:=192.168.0.100 target_filename:=$HOME/my_robot_calibration.yaml
    ```
4. 
    ```bash
    roslaunch ur5_newest_moveit_config ur5_moveit_planning_execution.launch
    ```
5. 
    ```bash
    roslaunch ur5_newest_moveit_config moveit_rviz.launch  config:=true
    ```
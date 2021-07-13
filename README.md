# ROS-Drivers

## About
Contains ROS packages that work as ROS drivers for interfacing the UR5 cobot and AG95 gripper.

## Initializing the drivers

### Method 1: Easy way
Use the python script that will be developed in the future

### Method 2: Manual way
1. Perform the calibration step to learn about the current state of the cobot before planning motion.
    ```bash
    roslaunch ur-ros-driver-config calibrate_cobot.launch
    ```

2. Launch the ROS controllers to drive the cobot.
    ```bash
    roslaunch ur-ros-driver-config spawn_controllers.launch
    ```

3. Load the `ros_interface.urp` program on the UR5 polyscope to receive the control commands.

4. Launch the driver to run the gripper.
    ```bash
    roslaunch dh_gripper_driver dh_gripper.launch
    ```

> To control the cobot, get the repository [ur5-ag95-resources]

[ur5-ag95-resources]: https://github.com/RIL-IISc/ur5-ag95-resources.git
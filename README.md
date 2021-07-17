# ROS-Drivers

## About
Contains ROS packages that work as ROS drivers for interfacing the UR5 cobot, AG95 gripper, and IntelRealsense camera.

## Initializing the drivers

### Method 1: Easy way
Use the python script that will be developed in the future

### Method 2: Manual way
1. Perform the calibration step to learn about the current state of the cobot before planning motion.
    ```bash
    roslaunch ur-ros-driver-config calibrate_cobot.launch
    ```

2. Launch the ROS controllers to drive the cobot and AG95 gripper
    ```bash
    roslaunch ur-ros-driver-config spawn_controllers.launch launch_camera_topics:=false
    ```

    Launch the ROS controllers to drive the cobot, AG95 gripper, and RealSense camera topics
    ```bash
    roslaunch ur-ros-driver-config spawn_controllers.launch
    ```

3. Load the `ros_interface.urp` program on the UR5 polyscope to receive the control commands.

## Publishing camera_link
> There is no need to perform this step if you are launching `ur5_moveit_control.launch` file.
1. After performing all the aforementioned steps
    ```bash
    roslaunch ur-ros-driver-config calibrate_camera.launch.xml
    ```

> To control the cobot, get the repository [ur5-ag95-resources]

[ur5-ag95-resources]: https://github.com/RIL-IISc/ur5-ag95-resources.git
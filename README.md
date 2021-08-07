# ROS-Drivers

## About
Contains ROS packages that work as ROS drivers for interfacing the UR5 cobot, AG95 gripper, and IntelRealsense camera.

## Build the driver sources
0. Make sure you are at within the `src` folder of your catkin workspace.
1.  Recursively clone this repository and it's submodules
    ```bash
    git clone --recursive https://github.com/RIL-IISc/ROS-Drivers.git
    ```
    Alternatively you could clone the repository and then update the submodules:
    ```bash
    git clone https://github.com/RIL-IISc/ROS-Drivers.git
    git submodule update --init --recursive
    ```
2. Go to the top workspace directory 
    ```bash
    cd ../
    ```
3. Install the dependencies required by the submodules
    ```bash
    # UR5 driver, AG95, and IntelRealsense dependencies
    sudo apt update
    rosdep update
    rosdep install --from-paths src --ignore-src -y

    # UR5 Intel RealSense calibration dependencies
    sudo apt-get install ros-${ROS_DISTRO}-visp
    sudo apt-get install ros-${ROS_DISTRO}-industrial-msgs
    sudo apt-get install ros-${ROS_DISTRO}-ros-control ros-${ROS_DISTRO}-ros-controllers

    # (optional: might not get installed with ros-controllers)
    sudo apt install ros-${ROS_DISTRO}-rqt-joint-trajectory-controller
    ```
4. Install Intel RealSense SDK 2.0 by following instructions from [here](https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md).
5. Build the workspace
    ```bash
    catkin_make
    ```
    > You might have to build the workspace multiple times with each time percentage of build increasing monotonically.
6. Provide permission to access AG95 gripper via port `/dev/ttyUSB0`. Follow instructions from [here](./AG95-ROS/README.md#setting-up-permissions).
7. Download python libraries to access Intel RealSense.
    ```bash
    pip install pyrealsense2
    ```

## Initializing the drivers

### Step 0:
Copy the file named `ur5_realsense_handeyecalibration_eye_on_hand.yaml` present in `ur-ros-driver-config/config` directory to `${HOME}/.ros/easy_handeye/`. Create the directories if required. Now follow one of the following methods:

### Method 1: Easy way
Use the python script that will be developed in the future

### Method 2: Manual way
1. Perform the calibration step to learn about the current state of the cobot before planning motion.
    ```bash
    roslaunch ur-ros-driver-config calibrate_cobot.launch
    ```

2. Launch the ROS controllers to drive the cobot and AG95 gripper
    ```bash
    roslaunch ur-ros-driver-config spawn_controllers.launch
    ```

    Launch the ROS controllers to drive the cobot, AG95 gripper, and RealSense camera topics
    ```bash
    roslaunch ur-ros-driver-config spawn_controllers.launch launch_camera_topics:=true
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
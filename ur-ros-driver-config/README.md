# ur-ros-driver-config

## About
A ROS package to house custom launch files that make use of various dependencies. This package has been created for convenience purposes to simplify the launching of ROS nodes and storing parameters. This README as serves to describe the various dependencies that have been used to develop the complete workspace to work with cobot UR5 with DH robotics' AG95 gripper and Intel's Realsense camera.

## On UR5 dependencies

TODO: Briefly describe what all things are needed

## On AG95 dependencies

TODO: Briefly describe what all things are needed

## On Intel Realsense dependencies
TODO: Briefly describe what all things are needed

#### Description of various parameters used in `calibrate_camera.launch`

- `namespace`: the calibrator script will save the result in `~/.ros/easy_handeye/$namespace`; the publisher will load the data from the same path.
- `eye_on_hand`: if true, this is an eye-on-hand calibration, else eye-on-base.
- `tracking_base_frame`: contains the tf id of the tracking system coordinate origin frame.
- `tracking_marker_frame`: contains the tf id of the tracking system target.
- `robot_base_frame`: contains the tf id of the robot's base link.
- `robot_effector_frame`: contains the tf id of the robot's end effector link.
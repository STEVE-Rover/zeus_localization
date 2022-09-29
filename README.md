# zeus_localization
Fuses heading data with position data to create a full pose estimation. This package was created as an alternative to [robot_localization](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html) in the case where we only need to combine position and heading without using complex filters like EKF.

## Subscribed topics
- position (geometry_msgs/PoseWithCovarianceStamped): Pose message of the position data. If the position data comes from a GPS, you can use [gnss_to_map](https://github.com/STEVE-Rover/gnss_to_map) to convert it to a pose message.
- heading (sensor_msgs/Imu): Imu message of the robot's global heading.

## Published topics
- localization_odom (nav_msgs/Odometry): Output odometry message of the fused localization.<br>
    **Note**: As of now, the twist is not calculated in the odometry message.

## Parameters
- ~base_frame (string, default: "base_link"): Name of the base frame.
- ~world_frame (string, default: "map"): Name of the world frame.
- ~publish_tf (bool, default: true): Publish the transform between the base frame and the world frame.

## TODO
- Calculate twist for the odometry message

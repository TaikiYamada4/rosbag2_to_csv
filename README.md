# rosbag2_to_csv

This is a tool to create csvs from ros2 bags.
This repository is temporary and will be archived soon.

このリポジトリでは以下の型のメッセージだけを変換できます。

- geometry_msgs/msg/PoseWithCovarianceStamped
- geometry_msgs/msg/TwistWithCovarianceStamped
- sensor_msgs/msg/Imu
- nav_msgs/msg/Odometry
- tier4_debug_msgs/msg/Float32Stamped

## Usage

```bash
ros2 run rosbag2_to_csv rosbag2_to_csv <bag_file> <topic_name> <message_type> <csv_file>
```

Example

```bash
ros2 run rosbag2_to_csv rosbag2_to_csv ./rosbags/example/ /localization/kinematic_state nav_msgs/msg/Odometry ./csv/kinematic_state.csv
```

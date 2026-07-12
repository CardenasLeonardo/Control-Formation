# Terminal 1: Lanzar simulación con 3 robots
ros2 launch simulation_pkg spawn_multi_robot.launch.py n_robots:=3

# Terminal teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot0/cmd_vel
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot1/cmd_vel
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot2/cmd_vel

# Terminal pose
ros2 topic echo /robot0/odom --field pose.pose
ros2 topic echo /robot1/odom --field pose.pose
ros2 topic echo /robot2/odom --field pose.pose

# Terminal laser
ros2 topic echo /robot0/scan
ros2 topic echo /robot0/scan --flow-style


ros2 topic echo /robot1/scan
ros2 topic echo /robot1/scan --flow-style

ros2 topic echo /robot2/scan
ros2 topic echo /robot2/scan --flow-style

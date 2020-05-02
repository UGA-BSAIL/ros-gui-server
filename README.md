# ROS GUI Server

The ROS GUI Server package is the component of the ROS GUI system which runs on the target robot. It includes a ros_gui_server_node which subscribes to an nav_msgs/Odometry topic, exposes Odometry, rviz visualization, and robot navigation through actionlib actions to the ROS GUI Client.

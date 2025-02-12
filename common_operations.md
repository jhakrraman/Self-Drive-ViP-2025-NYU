**launch robot**\
roslaunch turtlebot3_bringup turtlebot3_robot.launch\
roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch

**teleop**\
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

**create package**\
cd ~/catkin_ws/src\
catkin_create_pkg my_package rospy std_msgs

|||
|---|---|
|*touch* | (create file) |
|*mkdir*| (create folder)|
|*chmod +x \** | (make all\* e**x**ecutable)|


## record using rosbag:
- rostopic list
- rosbag record -O "custom_name.bag" /odom /camera/image/

## visualize rosbag:
- rosrun rviz rviz
    - fixed frame enter: odom
    - add by display type odometry 
    - topic enter: /odom
    - add by display type images
    - topic enter: /camera/image
- rosbag play rosbag_filepath_name.bag
    
## launch exploration:

#### Use default filepath (creates in workspace)
roslaunch exploration exploration.launch

#### Custom filepath 
roslaunch exploration exploration.launch bag_filename:=bags/session_1_data.bag


## launch navigation
roslaunch navigation navigation.launch\
bag_file_path:="/home/linux/catkin_s25/bags/021025test_2025-02-10-20-55-42.bag"\
target_image_path:="/home/linux/catkin_s25/src/navigation/target.jpg"
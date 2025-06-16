TurtleBot3 SLAM with GMapping

This project sets up SLAM (Simultaneous Localization and Mapping) using the gmapping package on a TurtleBot3 robot in ROS (Robot Operating System).
Prerequisites

Ensure that your TurtleBot3 is properly launched and connected. This guide assumes you have a ROS workspace (catkin_ws) and the package test_tb3_pkg set up.
Steps to Run GMapping SLAM
1. Launch RViz

rosrun rviz rviz

2. Publish Static Transform

rosrun tf static_transform_publisher 0 0 0 0 0 0 base_footprint base_scan 100

3. Build the Workspace

Navigate to your catkin workspace and build it:

cd ~/catkin_ws/
catkin_make

Creating the GMapping Launch File

Navigate to your package launch folder:

cd ~/catkin_ws/src/test_tb3_pkg/launch

Create the GMapping launch file:

touch gmapping.launch

Then open gmapping.launch in a text editor and paste the following content:

<launch>
  <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
    <!-- Basic configuration -->
    <param name="base_frame" value="base_footprint" />
    <param name="odom_frame" value="odom" />
    <param name="map_update_interval" value="5.0" />
    <param name="maxUrange" value="6.0" />
    <param name="delta" value="0.01" />
    
    <!-- Laser topic -->
    <remap from="scan" to="/scan" />
  </node>
</launch>

Launch Your Robot and Start Mapping

Run your custom launch file:

roslaunch test_tb3_pkg test_tb3_launch.launch

Then run the map saver to save the generated map:

rosrun map_server map_saver

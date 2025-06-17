(After launching robot)

rosrun rviz rviz

rosrun tf static_transform_publisher 0 0 0 0 0 0 base_footprint base_scan 100

cd catkin_ws/

catkin_make

catkin_ws -> src -> test_tb3_pkg -> launch

^^^open in terminal

touch gmapping.launch

Inside the file, put:

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


roslaunch test_tb3_pkg test_tb3_launch.launch


In the Rviz:

Add (left down)

TF

In the left taskbar, change Global Options -> Fixed frame to odom


Add

Map

In the left taskbar, change Map -> Topic to /map





rosrun map_server map_saver -f ~/map

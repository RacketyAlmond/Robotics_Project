(After launching robot)

rosrun rviz rviz

rosrun tf static_transform_publisher 0 0 0 0 0 0 base_footprint base_scan 100

cd catkin_ws/

catkin_make


*catkin_ws -> src -> test_tb3_pkg -> launch*

^^^open in terminal

touch gmapping.lounch

Inside the file, put:
*

*

roslaunch test_tb3_pkg test_tb3_launch.launch

rosrun map_server map_saver

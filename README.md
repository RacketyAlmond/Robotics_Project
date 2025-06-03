--- INICIAR ROBOT Y SERVER ---

1. Modificar .bashrc en PC:
	export ROS_MASTER_URI=http://10.10.73.162:11311
	export ROS_HOSTNAME=10.10.73.162
	export TURTLEBOT3_MODE=burger
	
2. Ejecutar 'roscore' en el PC

3. Abrir una nueva terminal y hacer 'ssh ubuntu@10.10.73.253' con password: 'turtlebot'

4. Modificar el .bashrc en ubuntu del robot:
	export ROS_MASTER_URI=http://10.10.73.162:11311
	export ROS_HOSTNAME=10.10.73.253
	export TURTLEBOT3_MODE=burger
	
5. Ejecutar 'roslaunch turtlebot3_bringup turtlebot3_robot.launch'

6. Abrir ternimal y ejecutar programa deseado; e.g.: 'roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch'

--- CREAR Y EJECUTAR PROGRAMA ---

1. Ir a 'catkin_ws/src' y ejecutar 'catkin_create_pkg <nombre_pkg> roscpp geometry_msgs nav_msgs tf'

2. Crear un fichero src/<nombre_cpp>.cpp

3. Crear directorio de launch 'roscd <nombre_pkg>' y 'mkdir launch'

4. Crear un fichero 'touch <nombre_launch>.launch'

5. Rellenar el archivo con:
	<launch>
		<!-- My Package launch file -->
		<node pkg="<nombre_pkg>" type="<nombre_exe>" name="<nombre>" output="screen">
		</node>
	</launch>
	
6. Modificar el fichero 'CMakeLists.txt' apartado 'Build':
	add_executable(<nombre_exe> src/<nombre_cpp>.cpp)
	add_dependencies(<nombre_exe> ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
	target_link_libraries(<nombre_exe> ${catkin_LIBRARIES})
	
7. Ir a 'catkin_ws' y ejecutar 'catkin_make'

8. Ir a '/home' y ejecutar 'roslaunch <nombre_pkg> <nombre_launch>.launch'

--- VER TOPICS ROS ---

rosmsg show <???_msgs>/<Message>

------------------------------- LAB -------------------------------

--- CONFIG ---

<nombre_pkg> = move_map_pkg
<nombre_cpp> = wall_follow
<nombre_exe> = wall_follow_exe
<nombre_launch> = wall_follow_launch
<nombre> = wall_follow

--- TERMINALES ---

Terminal 1: core
	roscore

Terminal 2: turtlebot
	roslaunch turtlebot3_bringup turtlebot3_robot.launch

Terminal 3: RVIZ
	rosrun rviz rviz

Terminal 4: Link base_scan frame to base_footprint
	rosrun tf static_transform_publisher 0 0 0 0 0 0 base_footprint base_scan 100

Terminal 5: gmapping
	roslaunch move_map_pkg gmapping.launch
	
Terminal 6: wall follower
	roslaunch move_map_pkg wall_follow_launch.launch

--- RVIZ ---

FixedFrame: odom

TF
LaserScan
Odometry
Map

--- GMAPPING ---

En la misma carpeta launch del pkg: launch/gmapping.launch

<launch>
  <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
    <!-- Configuración básica -->
    <param name="base_frame" value="base_footprint" />       <!-- Frame del robot -->
    <param name="odom_frame" value="odom" />           <!-- Frame de odometría -->
    <param name="map_update_interval" value="5.0" />    <!-- Tiempo de actualización del mapa (seg) -->
    <param name="maxUrange" value="6.0" />              <!-- Rango máximo del LIDAR -->
    <param name="delta" value="0.01" />                <!-- Resolución del mapa (metros/pixel) -->
    
    <!-- Topic del láser -->
    <remap from="scan" to="/scan" />                   <!-- Ajusta si tu topic es diferente -->
  </node>
</launch>
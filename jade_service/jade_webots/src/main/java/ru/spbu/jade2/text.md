# How to start with ROS
0. Source setup.bash of ros
1. Start roscore: ```roscore```
2. Start rosbridge: ```roslaunch rosbridge_server rosbridge_websocket.launch```
3. Run JADE process with following set of args:
   1. Main container: ```<first_robot_number> <robots_count_in_container> <rosbridge_websocket_uri> true```
   2. Plain (non-main) container: ```<first_robot_number> <robots_count_in_container> <rosbridge_websocket_uri> false <main_container_host> <main_container_port>```
4. Run Webots process:
   1. Build package with webots node: ```catkin build```
   2. Source another setup.sh: ```source devel/setup.bash```
   3. Export WEBOTS_HOME var: ```export WEBOTS_HOME=/usr/local/bin```
   4. Run Webots as ros node: ```roslaunch webots_logic webots_ros_25_robots_jade_python.launch```
5. Enjoy...
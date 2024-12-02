# BHARAT-FORGE-IITK

```
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/jjbigdub/gitrepo/rosslam_ws3/src/m-explore-ros2/map_merge/launch/tb3_simulation/models
ros2 launch multirobot_map_merge multi_tb3_simulation_launch.py slam_toolbox:=True

ros2 launch multirobot_map_merge map_merge.launch.py

rviz2 -d ~/gitrepo/rosslam_ws3/src/m-explore-ros2/map_merge/launch/map_merge.rviz
**(change path in the above code piece)**

python3 updated_map.py

Navigate to a given point

ros2 action send_goal /robot1/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```
###
NO LONGER NEEDED TO EXPORT THE FIRST LINE

Add turtlebot3_waffle in same folder as models and just export the second line
###

## FILES FOR CHANGES

cd src/m-explore-ros2/map_merge/launch/tb3_simulation/config
ADD a new file for additional bot:
only change is the port numbers: add any random 1000-1999 numbers distinct

cd src/m-explore-ros2/map_merge/config
Add params for another robot as well

cd src/m-explore-ros2/map_merge/launch/multi_tb3_simulation_launch.py
Add unknown and known pos for additional bot and add the return later in the same file

cd src/m-explore-ros2/map_merge/launch/rom_map_server.launch.py
increase the number of bots.

ros2 action send_goal /robot2/compute_path_to_pose nav2_msgs/action/ComputePathToPose "{goal: {pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}, header: {frame_id: 'map'}}}"





## For YOLO
In turtlebot3 replace the waffle folder

Then, Run the above given command

And to start saving the cordinates of object run final_coord.py --robot k, for kth robot in the common_text_file file workspace

All the yolos will make a text file and store the detected objects....now run a node to publish those objects

(Assumption is that no 2 same objects will be present within 0.1 m range of one)

ros2 run dynamic_object_publisher dynamic_object_publisher --ros-args -p log_file_path:=src/detected_objects.txt

For running listening use the following (it subscribes map and yolo data and publishes updated map and object coordinates)

ros2 run odom_and_map_publisher odom_and_map_publisher

For RL workspace

ros2 run exploration_env exploration_env


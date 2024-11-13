# bharatforge-iitk

``` export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
ros2 launch multirobot_map_merge multi_tb3_simulation_launch.py slam_toolbox:=True

ros2 launch multirobot_map_merge map_merge.launch.py

rviz2 -d ~/gitrepo/rosslam_ws/src/m-explore-ros2/map_merge/launch/map_merge.rviz```

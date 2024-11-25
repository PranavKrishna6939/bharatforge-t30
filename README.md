# BHARAT-FORGE-IITK

```
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
ros2 launch multirobot_map_merge multi_tb3_simulation_launch.py slam_toolbox:=True

ros2 launch multirobot_map_merge map_merge.launch.py

rviz2 -d ~/gitrepo/rosslam_ws/src/m-explore-ros2/map_merge/launch/map_merge.rviz
```

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


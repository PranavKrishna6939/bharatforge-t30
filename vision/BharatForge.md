# BharatForge-T30 SWARNA
## INTER-IIT 13.0 Submission

Team 30 *SWARNA: SWarm Autonmous RL-aware Navigation Task Assistant* submission for INTER-IIT problem statement given by *BHARATFORGE KALYANI - CENTRALISED INTELLIGENCE FOR DYNAMIC SWARM NAVIGATION*. The aim is to map the given enviornment when a certain number of bots are spawned in an unknown enviornment and to perform efficient task handling. We have used Turtlebot3-Waffle bots for the task.

## PLATFORM USED:

Ubuntu 22.04 LTS

ROS2 Humble

Gazebo 11 Classic

RVIZ


## DEPENDENCIES:
```
xargs -a apt-requirements.txt sudo apt install -y
pip install -r requirements.txt
```

### Turtlebot3
```
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git -b humble-devel
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git -b humble-devel
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git -b humble-devel
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git -b humble-devel
```

## EXECUTION
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:{PATH}/BharatForge/src/m-explore-ros2/map_merge/launch/tb3_simulation/models
```
To launch the Gazebo world alongside other individual costmaps
```
ros2 launch multirobot_map_merge multi_tb3_simulation_launch.py slam_toolbox:=True
```
To Launch the merged map
```
ros2 launch multirobot_map_merge map_merge.launch.py
rviz2 -d ~{PATH}/BharatForge/src/m-explore-ros2/map_merge/launch/map_merge.rviz
```

#### To change the number of Bots:
Change the number of bots as needed in the following path
```
{PATH}/BharatForge/src/m-explore-ros2/map_merge/config/params.yaml
```

#### To change the world file:
Update the world file in 
```
{PATH}/BharatForge/src/m-explore-ros2/map_merge/launch/tb3_simulation/world
```
And add the corresponding name in the path in line no. 127
```
{PATH}/BharatForge/src/m-explore-ros2/map_merge/launch/tb3_simulation/multi_tb3_simulation_launch.py
```

### RL WEIGHTS:
PATH : 
```
{PATH}/BharatForge/src
```
model95_4.pth is the weights for 4 robots
model5_6.pth is the weights for 6 robots

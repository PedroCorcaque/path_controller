# Path_controller

## ROS - Noetic
You can find the packages the I used here:
- https://github.com/ROBOTIS-GIT/turtlebot3
- https://github.com/ROBOTIS-GIT/turtlebot3_msgs
- https://github.com/ROBOTIS-GIT/turtlebot3_simulations

```
cd ~/catkin_ws/src/
git clone https://github.com/PedroCorcaque/path_controller.git
cd ~/catkin_ws
```

Remember to export the model of the turtlebot3 used (it can be saved in the .bashrc or .zshrc):
```
export TURTLEBOT3_MODEL=waffle
```

### SAC repository

```
cd ~/catkin_ws/src
git clone https://github.com/PedroCorcaque/sac.git
```

#### Build the packages

```
cd ~/catkin_ws/
catkin build
```

### Run package 
```
roslaunch path_controller path_controller_sac.launch
```

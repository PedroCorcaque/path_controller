# Path_controller

## ROS - Noetic
You can find the packages the I used here:
- https://github.com/ROBOTIS-GIT/turtlebot3
- https://github.com/ROBOTIS-GIT/turtlebot3_msgs
- https://github.com/ROBOTIS-GIT/turtlebot3_simulations

```
cd ~/catkin_ws/src/
git clone -b noetic-devel {link_git}
cd ~/catkin_ws && catkin_make
```
Remember to export the model of the turtlebot3 used (it can be saved in the .bashrc or .zshrc):
```
export TURTLEBOT3_MODEL=burger
```
### Run package 
```
roslaunch path_controller launcher.launch
```

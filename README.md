# TurtleMover

Package for gazebo simulation of turtlebot

This is a simulation I made to complete the following task:
> Write ros node to control the movement of turtlebot so as to reach from current
starting point to position marked in red. Use LaserScan data and move turtlebot in the
direction of farthest obstacle in the world. No predefined movement to be written in the
node.
> The safe distance from obstacle is 0.5 m.

turtlebot is a tool that helps people to explore simulations.

## Installation

After installing [ROS Kinetic Desktop Full](http://wiki.ros.org/kinetic/Installation/Ubuntu) on [Ubuntu 16.04](http://cdimage.ubuntu.com/netboot/16.04/), install [turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo) package then run the following commands:

```
$ export TURTLEBOT_GAZEBO_WORLD_FILE="/opt/ros/kinetic/share/turtlebot_gazebo/worlds/corridor.world"
$ roslaunch turtlebot_gazebo turtlebot_world.launch
```
After this Gazebo should open up, move the turtlebot to starting position, it should look something like this:

![gazebo_turtle image](assets/gazebo_turtle.png)

If you are facing any issues please refer to [ROS](https://answers.ros.org/questions/) or [Gazebo](http://answers.gazebosim.org/questions/) forums. If you are a newcomer to ROS or Gazebo please refer to [A Gentle Introduction to ROS](https://cse.sc.edu/~jokane/agitr/) and [Gazebo Tutorials](http://gazebosim.org/tutorials)

## Code Execution

Download and extract the code in a Directory say `ROS_WORKSPACE`, From the `ROS_WORKSPACE` Directory in your terminal run the following commands:

```
$ catkin_make
$ source devel/setup.bash #use source devel/setup.zsh for zsh
$ chmod 777 src/gazebo_turtle/turtle_path.py
$ rosrun gazebo_turtle/turtle_path.py
```

After running the above commands the turtlebot should start moving as shown in the video below:

![gazebo_turtle video](assets/gazebo_turtle.mp4)

# fws_robot_harmonic


[YouTube Video
](https://www.youtube.com/watch?v=b8VwSsbZYn0)


Do this in every terminal
```
source /opt/ros/jazzy/setup.bash
```

```
source install/setup.bash
```

clone this repo inside your workspace and build it by `colcon build`. Then, this:
```
ros2 launch fws_robot_sim fws_robot_spawn.launch.py
```
In new terminal: 
```
ros2 launch velocity_pub four_ws_control.launch.py
```


## To Do
1. To control the robot via keyboard
2. Then refactor the code.

# Assignment-Goat-Robotics

## Launch Robot in Gazebo
```bash
ros2 launch amr_description gazebo.launch.py world_name:=small_house
```

## Launch Robot Controller
```bash
ros2 launch amr_controller controller.launch.py
```

## Launch Navigation File
```bash
ros2 launch amr_navigation navigation.launch.py
```
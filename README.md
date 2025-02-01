# Assignment-Goat-Robotics

## Installation & Setup
Before launching the robot, ensure that you have sourced your ROS 2 Humble environment and workspace:
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Launching the Simulation

### 1. Launch the Robot in Gazebo

This command starts the robot in a simulated environment. You can specify the world file.
```bash
ros2 launch amr_description gazebo.launch.py world_name:=small_house
```
### 2. Start the Robot Controller

This command initializes the robot's controller, enabling movement and interaction.
```bash
ros2 launch amr_controller controller.launch.py
```

### 3. Launch the Navigation Stack

This command starts the navigation module, allowing the robot to autonomously reach its destinations.
```bash
ros2 launch amr_navigation navigation.launch.py
```


# Problem

When an order is received, the robot should:

1. ***Move from its home position to the kitchen.***

2. ***Move from the kitchen to the designated table for food delivery.***

3. ***After delivering the food, return to the home position automatically.***

### Implementation

To achieve this, a ROS 2 service is used to send single or multiple goal locations dynamically.

**Running the Delivery System**
```bash
ros2 run amr_utils problem_one.py
```
**Sending a Goal to the Robot**

Use the following command to send a goal (e.g., moving to table1):

```bash
ros2 service call /position_list amr_custom_msg/srv/GoalList "goal_list: [table1]"
```

### Output Video

[![problem](./data/problem_one.webm)](./data/problem_one.webm)






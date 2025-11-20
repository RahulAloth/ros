## Turtle Sim

Since we already learnt the basics, we now learn little bit complex topic.
Here are three progressively deeper projects you can try:

### Publisher–Subscriber Demo (Beginner)
In the beginer project we studied: 

* Create a custom package in C++ or Python.
* Write a publisher node that sends sensor-like data (e.g., temperature).
* Write a subscriber node that listens and logs it.
* This teaches you ROS 2 basics: nodes, topics, messages.

### TurtleSim Controller (Intermediate)

    sudo apt install ros-jazzy-turtlesim -y

### TODO:

* Write a node that publishes velocity commands to move the turtle in patterns (circle, square).
* Add a subscriber that listens to its pose and adjusts movement.
* This introduces you to services, parameters, and visualization.

### Practise

##### Step 1: Install TurtleSim

Make sure you have it: sudo apt install ros-jazzy-turtlesim -y

##### Run it in one terminal

    ros2 run turtlesim turtlesim_node

#### Create a Workspace & Package

    mkdir -p ~/ros2_ws/turtle_sim/src
    cd ~/ros2_ws/turtle_sim
    colcon build
    source install/setup.bash

 Or 
    ros2 pkg create --build-type ament_python turtle_sim --dependencies rclpy geometry_msgs

Copy [turtle_square.py](https://github.com/RahulAloth/ros/blob/main/turtle_square.py) to ../src

Copy [setup.py](https://github.com/RahulAloth/ros/blob/main/turtle_sim/setup.py) to turtle_sim/setup.py
Do the following:

    colcon build --packages-select turtle_square --install-base ~/ros2Installs

In a new Window, run

    ros2 run turtle_square turtle_square

Now we can see Turtle is moving across the window, according to our program.

## What all we learned from this intermediate example:
1. ROS2 Node Fundamentals
* Creating your own node (Python/C++) instead of relying on built-in teleop.
* Understanding how nodes interact via topics (/turtle1/cmd_vel).

2. Publishers & Message Types
* Publishing geometry_msgs/msg/Twist messages to control velocity.
* Structuring messages with linear and angular components.

3. Parameters
* Declaring and retrieving parameters inside a node (speed, angular_velocity, duration).
* Using parameters for flexibility instead of hardcoding values.

4. Timers & Execution Flow
* Using ROS2 timers to schedule periodic publishing.
* Coordinating motion sequences (e.g., move forward for X seconds, then rotate).
  
5. State Machines
* Implementing simple finite state machines inside a node.
* Switching between states like MOVE_FORWARD → ROTATE → NEXT_SIDE.
  
6. Workspace & Package Management
* Creating isolated workspaces (src, build, install) for reproducibility.
* Writing package.xml and CMakeLists.txt to define dependencies and build rules.
* 
7. Launch Files
* Writing .launch.py files to start multiple nodes together (turtlesim_node + controller).
* Passing parameters through launch files for modular testing.

8. ROS2 Build System (colcon)
* Building packages with colcon build.
* Using source install/setup.bash to overlay workspaces safely.
Now lets Jump into some Advance c++ program with ROS2.
Refer : https://docs.ros.org/en/jazzy/Tutorials.html






# Basic Concepts of ROS

## ROS Nodes

ROS nodes are the fundamental building blocks of a ROS system: each node is a process that performs a specific computation and communicates with other nodes through topics, services, and parameters.

A node is simply a program (process) in ROS that performs one task.
Nodes are designed to be modular and fine-grained, so instead of one giant program, a robot system is split into many smaller nodes.

Nodes form a graph where they exchange information using:

     Topics (publish/subscribe model for streaming data),
     Services (request/response model),
     Parameters (configuration values stored on the parameter server).

## ROS Topics

A ROS topic is a named communication channel that allows nodes to exchange messages using a publish–subscribe model. Publishers send data to a topic, and subscribers receive data from it. This is the primary way nodes in ROS share information.

## ROS Publisher

A ROS publisher is a node that sends messages on a topic so other nodes (subscribers) can receive and process them. Publishers are one half of the ROS communication model, enabling data flow between different parts of a robotic system.
A publisher is a ROS node that advertises a topic and publishes messages to it.
Other nodes can subscribe to that topic to receive the data.
Topics are named buses over which nodes exchange messages.
For example:

    A camera driver node publishes images on /camera/image_raw.

## ROS Subscriber

A ROS subscriber is a node that listens to a topic and processes the mesTF (Transform Library)

    Definition: A ROS library for keeping track of coordinate frames over time.

    Purpose: Allows nodes to understand spatial relationships (e.g., where the robot’s arm is relative to its base).

    Example:

        Transform between /map, /odom, and /base_link.sages published on it. Subscribers are the receiving half of the 
publish–subscribe communication model in ROS, enabling nodes to react to sensor data, commands, or any other information flowing through the system.
A subscriber connects to a topic and waits for messages.
When a message arrives, the subscriber’s callback function is triggered to handle the data.
Multiple subscribers can listen to the same topic, and each will receive all messages published there

## ROS Master

The ROS Master provides naming and registration services to the rest of the nodes in the ROS system. It tracks publishers and subscribers to topics as well as services. The role of the Master is to enable individual ROS nodes to locate one another. Once these nodes have located each other they communicate with each other peer-to-peer. 
https://wiki.ros.org/Master


## ROS Services

ROS services provide a request–response communication mechanism between nodes, similar to a remote procedure call (RPC). Unlike topics (which are continuous data streams), services are used for one-time interactions where a client sends a request and waits for a reply.

* Model: Request–response (like RPC).
* Use case: One-time actions (not continuous).

Example:

* A node calls /spawn_robot service to add a robot to a simulation.
* Difference from topics: Topics = continuous stream; Services = one-off interaction.

## ROS Messages

Definition: The data structures used to communicate between nodes via topics and services.

Details:
* Defined in .msg files (for topics) or .srv files (for services).
* Can be simple (like integers, strings) or complex (like sensor data, arrays).

Example:

* std_msgs/String → a simple text message.
* sensor_msgs/Image → structured image data.

## ROS Parameter Server

Definition: A shared, multi-variable dictionary accessible to all nodes.

Purpose: Stores configuration values and runtime parameters.

Example:
* A navigation node retrieves /max_speed parameter from the server.
* Useful for tuning without changing code.

## ROS Bag (rosbag)

Definition: A file format for recording and playing back ROS message data.

Use case: Debugging, simulation, and analysis.

Example:
* Record /camera/image_raw and /odom during a robot run, then replay later to test algorithms.

## ROS Launch Files

Definition: XML files (.launch) that start multiple nodes and set parameters together.

Purpose: Simplifies running complex systems.

Example:
* A launch file starts the robot’s camera, lidar, and navigation stack with one command.

## ROS Packages

Definition: The organizational unit in ROS containing nodes, libraries, configuration files, and launch files.

Purpose: Makes code modular and reusable.

Example:
* turtlebot3_navigation package contains everything needed for TurtleBot navigation.

## TF (Transform Library)

Definition: A ROS library for keeping track of coordinate frames over time.
Purpose: Allows nodes to understand spatial relationships (e.g., where the robot’s arm is relative to its base).

Example:
* Transform between /map, /odom, and /base_link.

## ROS Tools (CLI Utilities)

    Examples:

        roscore → starts the ROS Master.

        rostopic → inspect, publish, or subscribe to topics.

        rosservice → call or list services.

        rosparam → interact with the parameter server.

        rosbag → record/playback data.

        rviz → visualize sensor data and robot state.

        rqt → GUI tools for debugging and monitoring.


## Extended Comparison Table

| Concept          | Role / Purpose                   | Example Use Case      |
|------------------|----------------------------------|-----------------------|
| Messages         | Data structure for communication | sensor_msgs/Image     |
| Parameter Server | Store config values	          | /max_speed parameter  |
| Bag (rosbag)	   | Record/playback ROS data	      | Replay sensor logs    |
| Launch Files	   | Start multiple nodes + configs	  | Start navigation stack|
| Packages	       | Organizational unit in ROS	      | turtlebot3_navigation |
| TF	           | Coordinate frame transforms	  | Map → Odom → Base link|
| Tools	           | CLI/GUI utilities for debugging  | rviz, rostopic, rosbag|

## Eco system

                    +-------------------+
                    |      ROS Master   |
                    | (registration,    |
                    |  discovery)       |
                    +---------+---------+
                            |
                            v
    +-------------------+     +-------------------+     +-------------------+
    |   Node A (Sensor) | --> |   Topic: /scan    | --> | Node B (Mapping)  |
    |   Publisher       |     |   (Laser data)    |     | Subscriber        |
    +-------------------+     +-------------------+     +-------------------+

    +-------------------+     +-------------------+     +-------------------+
    | Node C (Camera)   | --> | Topic: /image_raw | --> | Node D (Vision)   |
    | Publisher         |     | (Image stream)    |     | Subscriber        |
    +-------------------+     +-------------------+     +-------------------+

    +-------------------+     +-------------------+
    | Node E (Client)   | --> | Service: /reset   |
    | Sends Request     |     | Node F (Server)   |
    |                   | <-- | Sends Response    |
    +-------------------+     +-------------------+

    +-------------------+
    | Parameter Server  |
    | Stores configs    |
    | e.g. /max_speed   |
    +-------------------+
            ^
            |
    +-------------------+
    | Node G (Planner)  |
    | Reads parameters  |
    +-------------------+

    +-------------------+
    | TF Library        |
    | Manages frames    |
    | (map → odom → base_link) |
    +-------------------+

## reating the First ROS 2 C++ Project
### Workspace Setup
* A ROS 2 workspace is a directory where packages live.
* We created one:
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws

### Package Creation

We scaffolded a new package using ROS 2 tools:
    
    ros2 pkg create --build-type ament_cmake my_first_cpp_pkg --dependencies rclcpp

This generated:
    CMakeLists.txt → build instructions
    package.xml → metadata and dependencies
    src/ → source folder for C++ files

### Writing the First Node

File: src/my_node.cpp

    #include "rclcpp/rclcpp.hpp"
    class MinimalNode : public rclcpp::Node {
    public:
        MinimalNode() : Node("minimal_node") {
            RCLCPP_INFO(this->get_logger(), "Hello ROS2 from C++!");
        }
    };

    int main(int argc, char * argv[]) {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<MinimalNode>());
        rclcpp::shutdown();
        return 0;
    }

This node simply prints a log message when run.


### Configuring CMakeLists.txt

We updated the file to properly build and install the node:

    cmake_minimum_required(VERSION 3.8)
    project(my_first_cpp_pkg)

    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    # Dependencies
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)

    # Executable
    add_executable(my_node src/my_node.cpp)
    ament_target_dependencies(my_node rclcpp)

    # Install target (critical for ros2 run)
    install(TARGETS
    my_node
    DESTINATION lib/${PROJECT_NAME})

    ament_package()


#### Key Points:
* find_package(rclcpp REQUIRED) → locates headers in /opt/ros/<distro>/include/rclcpp.
* ament_target_dependencies(my_node rclcpp) → links the executable with ROS 2 libraries.
* install(... DESTINATION lib/${PROJECT_NAME}) → ensures ros2 run can find the binary.

### Building the Package

We compiled the package with a custom install path:

    colcon build --packages-select my_first_cpp_pkg --install-base ~/ros2Installs

Output: 

    ~/ros2Installs/
    ├── my_first_cpp_pkg/
    │   ├── lib/my_first_cpp_pkg/my_node   <-- executable
    │   └── share/my_first_cpp_pkg/package.xml
    ├── setup.bash
    ├── setup.sh
    └── ...


### Running the Node

Source the custom install:

    source ~/ros2Installs/setup.bash

Run the node:
    ros2 run my_first_cpp_pkg my_node

Output
    [INFO] [minimal_node]: Hello ROS2 from C++!

### Summary

We successfully:
* Created a workspace and package.
* Wrote a minimal C++ node.
* Configured CMakeLists.txt correctly.
* Built with colcon into a custom install path (~/ros2Installs).
* Verified the node runs with ros2 run.

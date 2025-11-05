# ROS2

## Basics

ROS2 stands for Robotics Operating System 2. It is an open-source software framework and a set of tools, libraries and conventions for building software.

Reference: Youtube DIGIKEY

```bash
# activate ros env
source /opt/ros/jazzy/setup.bash

# chatter example
# talker
ros2 run demo_nodes_cpp talker

# listerner in new terminal
ros2 run demo_nodes_py listener
```

This example create a c++ talker which send "Hello World 1" each second and python listener which receives logs data from talker.

- This shows ROS2 can communicate between different languages.

## Debug or extra options availables

```bash
# shows all running topics
ros2 topic list

# shows info about the running topic
ros2 topic info /chatter

# quickly receives the message 
ros2 topic echo /chatter

# Likewise service,...
```

### RQT

rqt is the graphical framework for ROS 2. It provides a suite of tools for visualizing data and the system architecture.

```bash
rqt
# or
rqt_graph
```

## Ros2 begining

[https://github.com/ShawnHymel/introduction-to-ros/](https://github.com/ShawnHymel/introduction-to-ros/)

```bash
source /opt/ros/jazzy/setup.bash

mkdir src
cd src

ros2 pkg create --build-type ament_python my_py_pkg

```

- Ament is a build system for Python.

CREATE FILES (minimal_publisher, minimal_subscriber) inside FOLDERNAME

### Add created publisher to entry_points in setup.py

```py
...
entry_points={
        'console_scripts': [
            "minimal_publisher = my_py_pkg.my_minimal_publisher:main",
            ...
        ],
    },
...
```

```bash
cd ..

# This builds the project
colcon build

# use this to rebuild the edited package which reduce build time
colcon build --package-select my_py_pkg
```

New terminal

```bash
# Activate ros
source /opt/ros/jazzy/setup.bash

# install our script
source GitHub/ros-tutorial/install/setup.bash

# run the program
#        MY_PKG_NAME  FUNC_NAME
ros2 run my_py_pkg my_minimal_publisher
```

This create the custom publisher and lisener.

#### (optional) Register into package.xml for standardise

```xml
...
<depend>rclpy</depend>
<depend>example_interfaces</depend>
...
```

## Services

```bash
ros2 interface list

# Under services
# sample codes
...
example_interfaces/srv/AddTwoInts
example_interfaces/srv/SetBool
example_interfaces/srv/Trigger
...

# Check the details - returns type of i/p and o/p
ros2 srv interface show example_interfaces/srv/AddTwoInts
```

Create server(service)

```bash
# Run the service
# then check the running service
ros2 service list

ros2 service info /add_inits

ros2 service call /add_inits example_interfaces/srv/AddTwoInts "{a: 5, b: 2}"
# Spaces are necessory after colon {a: 2, b: 2}
```

Create Client (minimal_client)

## Simulate Gazebo
<!-- 
Reference: [How to Simulate a Robotic Arm in Gazebo - ROS 2 Jazzy](https://www.youtube.com/watch?v=ZYNCQIMkWmE&t=367s)

```bash
# Create packages
ros2 pkg create --build-type ament_cmake mycobot_bringup

ros2 pkg create --build-type ament_cmake mycobot_gazebo

ros2 pkg create --build-type ament_cmake mycobot_moveit_config

ros2 pkg create --build-type ament_cmake mycobot_system_tests
``` -->

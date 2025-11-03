# ROS2

## Basics

ROS2 stands for Robotics Operating System 2. It is an open-source software framework and a set of tools, libraries and conventions for building software.

Reference DIGIKEY

```bash
# activate ros env
source /opt/ros/jazzy/setup.bash

# chatter example
# talker
ros2 run demo_nodes_cpp talker

# listerner in new terminal
ros2 run demo_nodes_py listener
```

## Debug or extra options availables

```bash
# shows all running topics
ros2 topic list

# shows info about the running topic
ros2 topic info /<PROCESS(/chatter)>

# quickly receives the message 
ros2 topic echo /chatter
```

### RQT

ros graphical framework

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

Add created publisher to entry_points in setup.py

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

(optional) Register into package.xml for standardise

```xml
...
<depend>rclpy</depend>
<depend>example_interfaces</depend>
...
```

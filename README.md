# Development of TurtleBot3-Roomba

In this branch an implementation of a simple walker algorithm much like a Roomba robot vacuum cleaner has been developed using ROS 2.

## Overview
The exercise includes the following:
1. Learning about [Gazebo](https://classic.gazebosim.org/tutorials?cat=guided_b&tut=guided_b1), it's [GUI](https://classic.gazebosim.org/tutorials?cat=guided_b&tut=guided_b2), understanding the [Model Editor](https://classic.gazebosim.org/tutorials?cat=guided_b&tut=guided_b3) and installing [Gazebo packages](http://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros) in ROS 2.
2. Learning about TurtleBot from [here](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Turtlebot.html).
3. Creating a custom Gazebo World from [here](https://classic.gazebosim.org/tutorials?tut=build_world&cat=build_world).

## Dependencies
| Name | Version | License |
| :--- | :--- | :--- |
| Ubuntu | 20.04(LTS) | FSF Licenses |
| ROS 2 | Humble Hawksbill | Apache License 2.0 |
| C++ | 14 | Creative Commons Attribution-ShareAlike 3.0 Unported License |

## Packages
Packages that need to be in source directory:
* `gazebo_ros_packages`
* `turtlebot3`
* `turtlebot3_msgs`
* `turtlebot3_simulations`
* `vision_opencv`

## Setting up environment: part 1
This is the 'overlay' workspace present at the Top.
```
. <path-to-ros2_humble>/ros2_humble/install/local_setup.bash
```

## Commands to build this package
```
cd ~/ros2_ws/src/
git clone https://github.com/roboticistjoseph/TurtleBot3-Roomba.git
cd TurtleBot3-Roomba
rosdep install -i --from-path src --rosdistro humble -y
cd ../.. # Get back to your ros2 workspace
colcon build --packages-select project_roomba
```

## Setting up environment: part 2
- This is the 'underlay' workspace present beneath 'overlay'.
- Both the 'setting up of the environment' procedures must be done for every new terminal opened.
```
. install/setup.bash
```

## Set the Gazebo model path and TurtleBot3 model
- The launch file will not work unless these two parameters are set.
- Run the below commands in the terminal you wish to run the obstacle avoidance.
```
export GAZEBO_MODEL_PATH=`ros2 pkg prefix turtlebot3_gazebo`/share/turtlebot3_gazebo/models/
export TURTLEBOT3_MODEL=burger
```

## Launching `project_roomba`
ros2 launch project_roomba roomba_launcher.py record_flag:=False

## Using ROS2 Bag files to store the published data
- A launch file is created in the launch directory that calls the TurtleBot3 gazebo world and obstacle avoidance node.
- Note that walker_bag should not be in your directory to launch with recording. Delete the folder if it exists.
- Command to launch with recording:
```
ros2 launch project_roomba launch_walker.py record_flag:=True
```

## Code Analysis
Running 'cpplint' and 'cppcheck' to check for coding style and detect bugs.
### cpplint
Change to the root directory of the package, ```/TurtleBot3-Roomba```, and run:
```
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order ./src/*.cpp ./include/*.hpp > ./results/cpplint.txt
```
The results of running ```cpplint``` can be found in ```/results/cpplint.txt```.

### cppcheck
Change to the root directory of the package, ```/TurtleBot3-Roomba```, and run:
```
cppcheck --enable=all --std=c++17 ./src/*.cpp ./include/*.hpp --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=missingInclude --suppress=useInitializationList > results/cppcheck.txt
```
The results of running ```cppcheck``` can be found in ```/results/cppcheck.txt```.
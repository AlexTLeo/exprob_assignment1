**Institution:** University of Genoa<br>
**Course:** MSc in Robotics Engineering<br>
**Subject:** Experimental Robotics Laboratory<br>
**Author:** ***Alex Thanaphon Leonardi*** <thanaphon.leonardi@gmail.com><br>

# Assignment 2
[Full documentation here](https://thanaphonleonardi.github.io/exprob_assignment1/)

## 1. Introduction
This is the second part of the Experimental Robotics Laboratory assignment.
Given the first assignment, everything has been applied to a simulated robot
moving in a 3D space. Please refer to the first assignment's documentation for
any additional information regarding the base structure of this program.

The additional tools used in this assignment for the simulation are:
- [Gazebo](https://gazebosim.org/home)
- [Rviz](http://wiki.ros.org/rviz)
- [move_base](http://wiki.ros.org/move_base)
- [gmapping](http://wiki.ros.org/gmapping)
- [aruco_detect](http://wiki.ros.org/aruco_detect)

## 2. Running the program
### Installation
This simulation runs on ROS Noetic. Clone this repository into your ROS
workspace's src/ folder and switch to the correct branch:
```
git clone https://www.github.com/ThanaphonLeonardi/exprob_assignment1
git checkout assignment2
```
There is a heavy dependency on another repository: assignment2 from CarmineD8,
which has been forked and expanded for this assignment.
```
git clone https://www.github.com/ThanaphonLeonardi/assignment2
```
It also requires [ARMOR](https://github.com/EmaroLab/armor) (follow the
[tutorial presented here](https://github.com/EmaroLab/armor/issues/7)) and the
[ArmorPy API](https://github.com/EmaroLab/armor_py_api):
```
git clone https://github.com/EmaroLab/armor_py_api
```
Additionally, make sure to have [xterm](https://invisible-island.net/xterm/)
installed:
```
sudo apt-get install xterm
```

Navigate to the ROS workspace and run ```catkin_make```.

### Execution
Launch the simulation:
```
roslaunch assignment2 assignment.launch
roslaunch exprob-assignment1 mute.launch
```
The first launch will run gazebo and rviz. The second launch will start the
state machine and the simulation logic in a single terminal. It is also possible
to view each node in its own terminal by launching the ***debug.launch*** file.

## 3. The Robot
The simulated robot has been created and described via **.xacro** and **.gazebo**
files. These are found in the [assignment2](https://github.com/ThanaphonLeonardi/assignment2)
package.

For movement, it uses four wheels configured in skid-steering mode via
the gazebo plugin **skid_steer_drive_controller**. A laser scanner has been
mounted on top of it via the gazebo plugin **gazebo_ros_head_hokuyo_controller**,
with 360 degree range. Finally, an RGB camera was mounted on top as well, via
the gazebo plugin **camera_controller** which simulates a wge100 camera that
has been specifically configured for this assignment to have a higher resolution
and a tighter field of view. The camera is mounted on two joints: a prismatic
joint allows it to move up and down, and a rotational joint allows it to turn.

## 4. The Simulation Phases
Initially, the robot spawns in a special room and scans all the markers around
it using the [aruco_detect](http://wiki.ros.org/aruco_detect) library. Having
received all the marker IDs, it queries a server for information on the rooms
and builds the map (as an ontology) through the owl_interface node. The state
machine then instructs the robots on where to go next based on the first
assignment's logic, and the robot is physically moved to the various rooms
by using [move_base](http://wiki.ros.org/move_base) and
[gmapping](http://wiki.ros.org/gmapping).

## 5. Configuration files
The [move_base](http://wiki.ros.org/move_base) and [gmapping](http://wiki.ros.org/gmapping)
packages have been considerably tuned for this assignment, via the **.yaml** files
found in the config/ directory.

## 6. Camera Manager
A new component has been implemented: the **camera_manager**, which provides
an interface for commanding the camera joints. This node is used for the initial
and subsequent "scans" of the rooms.

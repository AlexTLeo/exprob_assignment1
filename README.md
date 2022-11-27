**Institution:** University of Genoa<br>
**Course:** MSc in Robotics Engineering<br>
**Subject:** Experimental Robotics Laboratory<br>
**Author:** ***Alex Thanaphon Leonardi*** <thanaphon.leonardi@gmail.com><br>

# Assignment 1

## 1. Introduction
This is the first assignment of the "Experimental Robotics Laboratory" course, for the Robotics Engineering degree, University of Genoa.

The goal of this assignment was to simulate, through an ontology, an environment
comprised of rooms and corridors, and to move a robot around these rooms based
on priority mechanisms. A Finite State Machine (FSM) represents the robot state.

The FSM is implemented through the [ROS SMACH library](http://wiki.ros.org/smach),
while the ontology is implemented through the [ARMOR framework](https://github.com/EmaroLab/armor).

## 2. Running the program
### Installation
This simulation runs on ROS Noetic. Clone this repository into your ROS workspace's src/ folder:
```
git clone https://www.github.com/ThanaphonLeonardi/exprob-assignment1
```
It also requires [ARMOR](https://github.com/EmaroLab/armor) (follow the
[tutorial presented here](https://github.com/EmaroLab/armor/issues/7)) and the
[ArmorPy API](https://github.com/EmaroLab/armor_py_api):
```
git clone https://github.com/EmaroLab/armor_py_api
```
Additionally, make sure to have [xterm](https://invisible-island.net/xterm/) installed:
```
sudo apt-get install xterm
```

### Execution
Launch the simulation:
```
roslaunch exprob-assignment1 assignment1.launch
```

## 3. Description
<img alt="Map of the environment" src="media/img/ontology_map.png" height="500"><br>
The environment is comprised of locations called **rooms** and **corridors**,
connected to each other by **doors**, and a **robot** that can occupy each of
these locations one at a time.<br>

- A **room** is an entity that can contain the **robot** and it is characterised
by having only *one* **door** connecting it to another location.<br>
- A **corridor** is a **room** with *two or more* **doors**.<br>

Each location is additionally characterised by a numeric data property called
*visitedAt* that represents the last moment it has been visited by the **robot**,
expresed in Unix time (since epoch).<br>

Before describing the **robot**'s behaviour, the concepts of *urgency* and
*reachability* should be introduced:
- a location is considered *reachable* if it is adjacent to the **robot**.
- a location is considered *urgent* if it has not been visited by the **robot**
for a certain amount of seconds, expressed by the *urgencyThreshold* property of
the **robot**.<br>

The **robot** exhibits the following behaviour: first, it looks for the next
*reachable* location to visit, following a priority list:
  1. urgent locations
  2. corridors
  3. rooms

Then, it goes to that location and waits for some time (to simulate work), after
which a new location is searched for and so on.

During its entire operation, the robot constantly monitors its own battery. If
the battery level falls below the first threshold (battery *low*), then the robot
will finsh up its current work and then immediately after go charge itself. If
however the battery level falls below the second threshold (battery *critical*),
then the robot will immediately stop its work and find a charging station.

A location is considered a **charger** if it allows the **robot** to charge. In this
simulation, only the **corridor E** is considered a **charger**.

### Demo Video

## 4. Behind The Scenes
The simulation incorporates several elements: a Finite State Machine governs the
flow of events, while a component diagram describes the interactions among the
ROS nodes involved.

### Finite State Machine
![The Finite State Machine of the robot](media/img/state_diagram.png)<br><br>
The robot starts in state **CHARGING** with a battery level determined by the
global variable *STARTING_BATTERY_LEVEL* defined in **robot_state.py**, and
positioned in location 'E' (which can be changed from the ontology, if desired).<br>
If the battery is low, the robot will immediately charge. If the battery is full,
the robot will transition to state **PLAN_GOAL** where
it will look for the next location to move towards.<br>

Once a new goal is planned, the robot transitions to state
**MOVING** where it will move to change its location. If, while moving, the
robot detects critically low battery, the state will transition back to **CHARGING**.
However, since charging can only happen in location 'E', if the robot is not
in the right place, then a new plan will immediately be created to bring the
robot to 'E', and the robot will once again attempt to charge.<br>
If, instead, everything goes well and the robot reaches its final destination,
then it will transition to state **WAITING** where the robot will simulate
work by waiting in the new location for a random number of seconds.

If during this wait the battery gets critically low, then the robot will
immediately attempt to charge. If, instead, everything goes well and the robot
completes its work, then the transition to state **PLAN_GOAL** will be made and
the robot will look for a new goal.

### Component Diagram
![Component Diagram of the system](media/img/compdiag2.png)<br>
#### Component: OWL Interface
#### Component: Robot State
#### Component: Controller
#### Component: Planner
#### Component: Behaviour

### Temporal Diagram

### Extras
The **roslaunch** file is set by default to print to the terminal. The program is
capable of **logging** and can be set to do so, to the default ROS logs, by
modifying the launch file accordingly.

## 5. System Analysis
### Assumptions
1. The robot starts in state CHARGING and in room 'E', even though these can be
tweaked from the ontology and from the behaviour.py script.
### Current Limitations
### Possible Improvements
graph search

**Institution:** University of Genoa<br>
**Course:** MSc in Robotics Engineering<br>
**Subject:** Experimental Robotics Laboratory<br>
**Author:** ***Alex Thanaphon Leonardi*** <thanaphon.leonardi@gmail.com><br>

# Assignment 1

## Introduction
This is the first assignment of the "Experimental Robotics Laboratory" course, for the Robotics Engineering degree, University of Genoa.

The goal of this assignment was to simulate, through an ontology, an environment
comprised of rooms and corridors, and to move a robot around these rooms based
on priority mechanisms. A Finite State Machine (FSM) represents the robot state.

The FSM is implemented through the [ROS SMACH library](http://wiki.ros.org/smach),
while the ontology is implemented through the [ARMOR framework](https://github.com/EmaroLab/armor).

## Running the program
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

## Description
The environment is comprised of locations called **rooms** and **corridors**,
connected to each other by **doors**, and a **robot** that can occupy each of
these locations one at a time.
A **room** is an entity that can contain the **robot** and it is characterised
by having only *one* **door** connecting it to another location. A **corridor**
is a **room** with *two or more* **doors**

### Demo Video

## Behind The Scenes

### Extras
The **roslaunch** file is set by default to print to the terminal. The program is
capable of **logging** and can be set to do so, to the default ROS logs, by
modifying the launch file accordingly.

### Finite State Machine

### Component Diagram
#### Component: OWL Interface
#### Component: Robot State
#### Component: Controller
#### Component: Planner
#### Component: Behaviour

### Temporal Diagram

## System Analysis
### Current Limitations
### Possible Improvements

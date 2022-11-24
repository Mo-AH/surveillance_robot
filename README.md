# Surveillance robot #

**First Assignment of Experimental Robotic Laboratory - Robotics Engineering - UniGE**

In this repository is developed the simluation of a robot deployed in a
indoor environment for surveillance purposes.

---

### Introduction ###

This software is ROS-based, written in python and in particular it uses:
  - [smach](http://wiki.ros.org/smach) State machine library to simulate the robot behaviour.
  - [topological_map](https://github.com/buoncubi/topological_map) - An ontology previously created with Protegé: it represents an Indoor Environment.
  - [armor_py_api](https://github.com/EmaroLab/armor_py_api) - A useful interface to manaipulate and query the ontology, using the Pellet reasoner.

Full documentation can be found [here](https://Mo-AH.github.io/surveillance_robot/).

---

## Scenario ##

The scenario involves a robot deployed in a indoor environment for surveillance purposes.
The robot's objective is to visit the different locations and stay there for some time.

The robot behaviour can be divided into two phases.

 - Phase 1:
    - The robot spawns in his starting location.
    - The robot waits until it receives all the information to build the topological map.
 
 - Phase 2:
    - The robot moves in a new location, and waits some time before to visit another location. (This behavior is repeated in a infinite loop).
    - If the battery get low, it leave the task it was doing to reach the charging location and waits some time to recharge.

The time duration of those tasks are considered as ros parameters.

### Environment ###

In this assignment, we are considering a 2D environment made of 4 rooms and 3 corridors, that could be like the one shown in the figure (which is also the default one).

***FIGURA***

The indoor environment is composed of locations and doors.
 - A room is a location with just one door
 - A corridor is a location with 2 or more doors
 - If two locations have the same door, they are connected and the robot can move from one to the other
 - If a location has not been visited for some time (parameter `urgencyThreshold`), it becomes urgent


### Assumptions ###

There are multiple ways for achieving such behaviour and a set of assumptions should be made. The ones of this repository are the following:
 - The environment has no obstacles.
 - The environment does not change in time.
 - The robot can move only to locations connected to the current location.
 - The only location that the robot can always reach is the recharging one.
 - All the locations except the recharging one can become urgent.
 - The battery can become low at any time.


### Surveillance Policy ###

This represents the phase 2 of the scenario and it works as follows.
When robot's battery is not low, it should move among locations with 2 basic rules:
 - It should mainly stay on corridors.
 - If a reachable room has not been visited for some times it should visit it.

In this repository, a surveillance policy that relies on the above rules has been developed and its pseudocode is the following:

``` 
    # [1] Decide and move to next location
    if there are urgent locations reachable:
      move to the most urgent
    else:
      move to a corridor

    check the location [2]
    

    # [2] Check the location if it is urgent
    if reached location is urgent:
      check the location
    start again from [1]


    # [0] Battery checking:
    if battery is low:
      move to charging location
      recharge
      start again from [1]

```
Note that, while performing `[1]` or `[2]`, it is always aware of the battery level. Moreover, if the battery is low, the `[0]` algorithm cancels the task it was doing. 

---

## Software Architecture ##

---

## How to Run ##

The repository has been developed in this [Docker-based environment](https://hub.docker.com/repository/docker/carms84/exproblab) with [ROS Noetic](http://wiki.ros.org/noetic) installed.
It uses a couple of elements that should be installed before running the code:
  - [ARMOR Server](https://github.com/EmaroLab/armor), which can be installed by following the instructions described in the Readme.
  - [xterm](https://wiki.archlinux.org/title/Xterm), which can be installed by running from the terminal `$ sudo apt install -y xterm`.


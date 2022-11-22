# Surveillance robot #

**First Assignment of Experimental Robotic Laboratory - Robotics Engineering - UniGE**

In this repository is developed the simluation of a robot deployed in a
indoor environment for surveillance purposes.

---

## Introduction ##

This software is ROS-based, written in python and in particular it uses:
  - [smach](http://wiki.ros.org/smach) state machine to simulate the robot behaviour
  - [topological_map]() an ontology previously created with Protegé: it represents an Indoor Environment
  - [armor_py_api](https://github.com/EmaroLab/armor_py_api), a useful interface to manaipulate and query the ontology, using the Pellet reasoner.

Full documentation can be found [here](https://Mo-AH.github.io/surveillance_robot/).


### Environment ###

In this assignment, we are considering a 2D environment made of 4 rooms and 3 corridors, which could be like the one shown in the figure.***FIGURA***
The indoor environment is composed of locations and doors.
 - A room is a location with just one door
 - A corridor is a location with 2 or more doors
 - If two locations have the same door, they are connected and the robot can move from one to the other
 - If a location has not been visited for some time (parameter `urgencyThreshold`), it becomes urgent


## Scenario ##

The scenario involves a robot deployed in a indoor environment for surveillance purposes.
The robot's objective is to visit the different locations and stay there for some time.


The robot behaviour can be divided into two phases.

 1. Phase 1:
    - The robot spawns in his starting location.
    - The robot waits until it receives all the information to build the topological map.
 
 2. Phase 2:
    - The robot moves in a new location, and waits for some times before to visit another location. (This behavior is repeated in a infinite loop).
    - If the battery get low, it leave the task it was doing to reach the charging location and waits some time to recharge.

The time duration of those tasks are considered as ros parameters.


### Assumptions ###

There are multiple ways for achieving such behaviour and a set of assumptions should be made. The ones of this repository are the following:
 - The environment has no obstacles.
 - The environment does not change in time.
 - The robot can move only to locations connected to the current location.
 - The only location that the robot can always reach is the recharging one.
 - All the locations except the recharging one can become urgent.
 - The battery can become low at any time.


### Surveillance Policy ###

When robot's battery is not low, it should move among locations with 2 basic rules:
 - It should mainly stay on corridors.
 - If a reachable room has not been visited for some times it should visit it.

In this repository, a surveillance policy that relies on the above rules has been developed and its pseudocode is the following:


``` 
    # Decide and move to next location
    [1] if there are urgent locations reachable:
      move to the most urgent
    else:
      move to a corridor

    check the location [2]
    

    # Check the location if it is urgent
    [2] if reached location is urgent:
      check the location
    start again from [1]


    # Battery low behaviour
    [0] Always check the battery level:
      if it is low:
        move to charging location
        recharge
        start again from [1]

```

[1] If there are urgent locations reachable, it goes to the most urgent
      else, it goes to a corridor

[2] If the reached location is urgent, it should check it before moving to a new location
      else, it passes directly to the next location

[3] If battery get low, it leaves any task it was doing and moves to the charging location to recharge. After that, it begins again from [1].

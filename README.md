# Surveillance Robot #

**First Assignment of Experimental Robotic Laboratory - Robotics Engineering - UniGE**

In this repository is developed a ROS-based simulation of a robot in a
indoor environment for surveillance purposes.

---

## Scenario ##

### Environment ###


In this assignment, we are considering a 2D environment made of 4 rooms and 3 corridors, that could be like the one shown in the figure (which is also the default one).

***FIGURA***

The indoor environment is composed of locations and doors.
 - A room is a location with just one door
 - A corridor is a location with 2 or more doors
 - If two locations have the same door, they are connected and the robot can move from one to the other
 - If a location has not been visited for some time (parameter `urgencyThreshold`), it becomes urgent


For the environment representation, it has been used the [topological_map](https://github.com/buoncubi/topological_map) ontology, which has been previously created with Protegé. In particular, the [file](https://github.com/Mo-AH/surveillance_robot/tree/main/ontologies) used in this software is completely without the Abox.

### Assumptions and Behaviour ###

The robot behaviour can be devided into two phases:

 - Phase 1:
    - The robot spawns in his starting location.
    - The robot waits until it receives all the information to build the topological map.
 
 - Phase 2:
    - The robot moves in a new location, and waits some time before to visit another location. (This behavior is repeated in a infinite loop).
    - If the battery get low, it leave the task it was doing to reach the charging location and waits some time to recharge.

The time duration of those tasks are considered as ros parameters.

Moreover, when robot's battery is not low, it should move among locations with 2 basic rules:
 - It should mainly stay on corridors.
 - If a reachable room has not been visited for some times it should visit it.
 - 
There are a lot of ways for achieving a surveillance behaviour and a set of assumptions should be made:
 - The environment has no obstacles.
 - The environment does not change in time.
 - The robot can move only to locations connected to the current location.
 - The only location that the robot can always reach is the recharging one.
 - All the locations except the recharging one can become urgent.
 - The battery can become low at any time.

After having received the informations to build the map, it starts in a loop the phase 2 of the simulation, which can be modeled in a several ways depending on further specifications (e.g. only rooms should be urgent or maybe even the charging location should be be visited normally).
The behaviour implemented in this repository follows the policy described in this pseudocode:

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

[smach](http://wiki.ros.org/smach) - State machine library to simulate the robot behaviour.



The full documentation can be found [here](https://Mo-AH.github.io/surveillance_robot/).

---

## How to Run ##

Thi software is developed with a [ROS Noetic](http://wiki.ros.org/noetic) environment and you need to have a ROS workspace initialized in order to run the simulation. Moreover you should have installed:
  - [ARMOR Server](https://github.com/EmaroLab/armor), a ROS integration to manipulate online OWL ontologies, which can be installed by following the instructions in the README.
  - [armor_py_api](https://github.com/EmaroLab/armor_py_api), a library to simplify the python calls to the ARMOR Server, which can be installed by following the instructions in the README.
  - [xterm](https://wiki.archlinux.org/title/Xterm), a terminal simulator, which can be installed by running from the terminal `$ sudo apt install -y xterm`.
  
After that, follow those steps:
  1. In the `src` folder of your ROS workspace, clone this repository by running `git clone https://github.com/Mo-AH/surveillance_robot`
  2. Move first to `src/surveillance_robot/scripts` and then to `src/surveillance_robot/utilities/surveillance_robot` and run a `chmod +x <script_name>.py` for each Python module in both folders.
  3. Build the ROS workspace by running `catkin_make` in its root folder.
  4. Launch the simulation by running `roslaunch surveillance_robot simulation.launch`

You should now see something like this:

***figura***







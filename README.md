# Surveillance Robot #

**First Assignment of Experimental Robotic Laboratory - Robotics Engineering - UniGE**

In this repository is developed a ROS-based simulation of a robot in a
indoor environment for surveillance purposes.

---

## Scenario ##

### Environment ###


In this assignment, we are considering a 2D environment made of 4 rooms and 3 corridors, that could be like the one shown in the figure (which is also the default one).

![default_environment](https://user-images.githubusercontent.com/91679281/203867615-f3655f83-87aa-480c-9f89-6022ed4af79a.png)

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

### Components diagram ###

![component_diagram](https://user-images.githubusercontent.com/91679281/203868060-07eac4a6-41d4-48bb-b6a8-51ac289e9a0c.png)

### Sequence diagram ###
![sequence_diagram](https://user-images.githubusercontent.com/91679281/203868067-1aaa2c30-93bb-4eab-866e-c3edf35ddefa.png)

### States diagram ###
![states_diagram drawio](https://user-images.githubusercontent.com/91679281/203871623-20364fd2-2646-4bc9-aca6-3f416d9bb0f7.png)

---

## How to Run ##

This software is developed with a [ROS Noetic](http://wiki.ros.org/noetic) environment and you need to have a ROS workspace initialized in order to run the simulation. Moreover you should have installed:
  - [ARMOR Server](https://github.com/EmaroLab/armor), a ROS integration to manipulate online OWL ontologies, which can be installed by following the instructions in the README.
  - [armor_py_api](https://github.com/EmaroLab/armor_py_api), a library to simplify the python calls to the ARMOR Server, which can be installed by following the instructions in the README.
  - [xterm](https://wiki.archlinux.org/title/Xterm), a terminal simulator, which can be installed by running from the terminal `$ sudo apt install -y xterm`.
  
After that, follow those steps:
  1. In the `src` folder of your ROS workspace, clone this repository by running `git clone https://github.com/Mo-AH/surveillance_robot`
  2. Move first to `src/surveillance_robot/scripts` and then to `src/surveillance_robot/utilities/surveillance_robot` and run a `chmod +x <script_name>.py` for each Python module in both folders.
  3. Build the ROS workspace by running `catkin_make` in its root folder.
  4. Now, you can launch the simulation in two different modes:
      - Changing the battery state manually with a terminal interface, by running `roslaunch surveillance_robot manual_batter.launch`.
      - Changing the battery state randomly within the time interval specified in the corrisponding parameter, by running `roslaunch surveillance_robot random_battery.launch`.
      

### Parameters ###

There are some parameters that are setted by default, but they can be changed to meet some specification:

  - `test/random_sense/active` it defines the battery mode: True for randomly change the state, False to change the state manually.
  - `test/random_sense/battery_time` when random mode is active, this parameter specifies the time interval within which the state will be changed.
  - `test/plan_points` it's an interval that defines the number of points of a plan.
  - `test/plan_time` it's an interval that defines the time duration of the planner.
  - `test/motion_time` it's an interval that defines the time duration of the controller.
  - `test/checking_time` it's a number that defines the time required for the check of a location.
  - `test/charging_time` it's a number that defines the time required to recharge the battery.
  - `state/initial_pose` it's a x-y coordinate that represents the initial position of the robot.
  - `config/environment_size` it contains the maximum coordinate both for x/y-axis.

Other parameters regarding the ontology, such as starting location, charging location or connections list, can be changed directly in the `architecture_name_mapper.py` module.

### Running Simulation ###

### System limitations and possible improvements ##

---

## Credits ##

The software has been developed starting from the [arch_skeleton](https://github.com/buoncubi/arch_skeleton) repository, created by prof. Luca Buoncompagni. In particular, the following python modules has been used almost as they were, with only some small edit:
  - `robot_state`
  - `planner`
  - `controller`
  - `action_client_helper`

_Author_: Mohammad Al Horany

_Email_: s5271212@studenti.unige.it






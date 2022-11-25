# Surveillance Robot #

**First Assignment of Experimental Robotic Laboratory - Robotics Engineering - UniGE**

In this repository is developed a ROS-based simulation of a robot in a
indoor environment for surveillance purposes.

It relies on [smach](http://wiki.ros.org/smach), a state machine library to simulate the robot behaviour.

The full documentation can be found [here](https://Mo-AH.github.io/surveillance_robot/).

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


For the environment representation, it has been used the [topological_map](https://github.com/buoncubi/topological_map) ontology, which was previously created with Protegé. In particular, the [file](https://github.com/Mo-AH/surveillance_robot/tree/main/ontologies) used in this software is completely without the Abox.

### Assumptions and Behaviour ###

The robot behaviour can be devided into two phases:

 - Phase 1:
    - The robot spawns in his starting location.
    - The robot waits until it receives all the information to build the topological map.
 
 - Phase 2:
    - The robot moves in a new location, and waits some time before to visit another location. (This behavior is repeated in a infinite loop).
    - If the battery get low, it leave the task it was doing to reach the charging location and waits some time to recharge.

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


### Components diagram ###
The connections among the nodes are described in the following image.

![component_diagram](https://user-images.githubusercontent.com/91679281/203868060-07eac4a6-41d4-48bb-b6a8-51ac289e9a0c.png)

We can see how the `smach_robot`, which implements the behaviour of the robot using a state machine, is the central entity of the entire architecture and is better described after, in the states diagram.
Taking apart the `ARMOR Server`, which is used by `smach_robot` to update and reason about the ontology, the other nodes simulate specific components of the robot:

 - `robot_state`: node in charge of managing the battery level and the pose of the robot. It provides two services for the pose (`GetPose.srv`, `SetPose.srv`), a publisher for the battery level (`Bool.msg`) and a service to recharge the battery (`SetBool.srv`).
 
 - `map_builder`: node in charge of providing the data informations to build the map. It publish in the topic `/map/connections` messages of type `DoorConnection.msg`, which consist in a pair of Door and Location. When it finishes trasmitting all the connections, its job is done and so it exits. 
 
 - `planner`: node in charge of providing a plan, consisting in a `Point` list, given a target location. For doing so, it implements an Action Server that uses `Plan.action`, which interacts with `robot_state`.
 
 - `controller`: node in charge of simulating the motion of the robot. Given a plan, it follows the points path to reach a target location. For doing so, it implements an Action Server that uses `Control.action`, which interacts with `robot-state`.
 

### Sequence diagram ###

![sequence_diagram](https://user-images.githubusercontent.com/91679281/203868067-1aaa2c30-93bb-4eab-866e-c3edf35ddefa.png)

This diagram represents the sequential flow of the architecture. 

In particular, it shows that the nodes always active are `smach_robot`, expectable as it is the central entity that calls the other components, and the `robot_state`, because it always keeps track of the battery level.

Apart from `map_builder`, which once finished the trasmission of connections exits, the other nodes are active only when the `smach_robot` call them:

 - `ARMOR Server`: initially, when we are defining the ontology objects and properties (Abox), and later after every location change of the robot.
- `planner` and `controller`: when the robot has to reach a location.

***Note***: in the diagram, when the `robot_state` notify the battery low, the interactions of the `smach_robot` with the `planner` and the `controller` to reach the charging location before actually recharge have been omitted.


### States diagram ###
![states_diagram drawio](https://user-images.githubusercontent.com/91679281/203871623-20364fd2-2646-4bc9-aca6-3f416d9bb0f7.png)

In this diagram is shown the robot behaviour, implemented through a state machine in the `smach_robot` node. The node relies on the use of `smach_helper` module, which decouples the state machine interface from the computations processes.

After having built the map in the `BUILD MAP` state, it passes to the loop of the Phase 2, which begins with `REACH LOCATION`, that is a sub state machine and its inner states are:
 - `REASONER` : queries the ontology about reachable and urgent locations to decide the next location. If the battery is low, the next location is always the charging one.
 - `PLANNER`: it calls the planner action server to get the plan for the next location.
 - `CONTROLLER`: it calls the controller action server to actuate the plan and simulate the motion of the robot. When it has reached the location, it uploads the ontology.
 
When the motion is finished (i.e. it reached the target location) by the `CONTROLLER` node, there are 3 possible transitions:
 - 'location_urgent_reached': the location should be checked and so it passes to the `CHECK_LOCATION` state.
 - `location_not_urgent_reached': the location shouldn't be checked so it passes to the `REASONER` state to decide next location.
 - `charging_location_reached`: the robot has reached the charging location so it passes to the `CHARGE` state to recharge the battery.
 
 The states `CHECK_LOCATION` and `CHARGE` simply returns to the `REASONER` state when they have terminate their tasks.
 
 Note that, except for the `CHARGE` state, all other states pass to the `REASONER` when the battery is low (`battery_low` transition), leaving their task uncompleted.


---

## Simulation ##

### How to Run ###

This software is developed with a [ROS Noetic](http://wiki.ros.org/noetic) environment and you need to have a ROS workspace initialized in order to run the simulation. Moreover you should have installed:
  - [ARMOR Server](https://github.com/EmaroLab/armor), a ROS integration to manipulate online OWL ontologies, which can be installed by following the instructions in the README.
  - [armor_py_api](https://github.com/EmaroLab/armor_py_api), a library to simplify the python calls to the ARMOR Server, which can be installed by following the instructions in the README.
  - [xterm](https://wiki.archlinux.org/title/Xterm), a terminal simulator, which can be installed by running from the terminal `sudo apt install -y xterm`.
  - [smach](http://wiki.ros.org/smach), a state machine library to simulate the robot behaviour, which can be installed by running from the terminal `sudo apt-get install ros-noetic-smach-ros`

After that, follow those steps:
  1. In the `src` folder of your ROS workspace, clone this repository by running `git clone https://github.com/Mo-AH/surveillance_robot`
  2. Move first to `src/surveillance_robot/scripts` and then to `src/surveillance_robot/utilities/surveillance_robot` and run a `chmod +x <script_name>.py` for each Python module in both folders.
  3. Build the ROS workspace by running `catkin_make` in its root folder.
  4. Now, you can launch the simulation in two different modes:
      - Changing the battery state manually with a terminal interface, by running `roslaunch surveillance_robot manual_batter.launch`.
      - Changing the battery state randomly within the time interval specified in the corrisponding parameter, by running `roslaunch surveillance_robot random_battery.launch`.
      

### Parameters ###

There are some parameters that are setted by default, but they can be changed to meet some specification:

 - `config/environment_size`: It represents the environment boundaries as a list of two float
   numbers, i.e., `[x_max, y_max]`. The environment will have the `x`-th coordinate spanning
   in the interval `[0, x_max)`, while the `y`-th coordinate in `[0, y_max)`.

 - `state/initial_pose`: It represents the initial robot pose as a list of two float numbers, 
   i.e., `[x, y]`. This pose should be within the `environmet_size`.

 - `test/plan_points`: It represents the number of via points in a plan, and it should be
   a list of two integer numbers `[min_n, max_n]`. A random value within such an interval will be chosen to simulate plans of different lengths.

 - `test/plan_time`: It represents the time required to compute the next via point of the 
   plan, and it should be a list of two float numbers, i.e., `[min_time, max_time]` in seconds. 
   A random value within such an interval will be chosen to simulate the time required to 
   compute the next via points.

 - `test/motion_time`: It represents the time required to reach the next via point, and 
   it should be a list of two float numbers, i.e., `[min_time, max_time]` in seconds. A random
   value within such an interval will be chosen to simulate the time required to reach the next 
   via points. 

  - `test/random_sense/active`:  It is a boolean value that defines the battery mode: True for randomly change the state, False to change the state manually.
  
  -  `test/random_sense/battery_time`: It indicates the time passed within battery state changes 
   (i.e., low/high). It should be a list of two float numbers, i.e., `[min_time, max_time]` in 
   seconds, and the time passed between changes in battery levels will be a random value within 
   such an interval.
   
  - `test/checking_time`: It indicates the time required for the check of a location, should be an float number.
  
  - `test/charging_time`:  It indicates the time required to recharge the battery, should be a float number.

Other parameters regarding the ontology, such as starting location, charging location or connections list, can be changed directly in the `architecture_name_mapper.py` module.

### Simulation in action ###

### System limitations and possible improvements ##

---

## Credits ##

The software has been developed starting from the [arch_skeleton](https://github.com/buoncubi/arch_skeleton) repository, created by prof. Luca Buoncompagni. In particular, the following python modules has been used almost as they were, with only some small edit:
  - `robot_state`
  - `planner`
  - `controller`
  - `action_client_helper`



***Author***: Mohammad Al Horany

***Email***: s5271212@studenti.unige.it






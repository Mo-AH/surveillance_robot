#!/usr/bin/env python
"""
.. module:: architecture_name_mapper
  :platform: Unix 
  :synopsis: Python module for the architecture name mapper
.. moduleauthor:: Mohammad Al Horany, 5271212@studenti.unige.it

In this module are defined the names of all parameters, nodes, topics, services and action servers of the architecture.
Also, it stores the data information about the ontology and the map.

"""

import rospy

#---------------------------------------------------------
# PARAMETERS
# ---------------------------------------------------------
# The name of the parameter to define the environment size.
# It should be a list `[max_x, max_y]` such that x:[0, `max_x`) and y:[0, `max_y`).
PARAM_ENVIRONMENT_SIZE = 'config/environment_size'

# The name of a boolean parameter to active random testing.
# If the value is `False` a keyboard-based interface will be used to produce stimulus 
# (i.e., speech, gesture and battery signals). Instead, random stimulus will be generate 
# if `True`. In the latter case, the architecture also requires all the parameters 
# with a the scope `test/random_sense/*`, which are not used if `False`.
PARAM_RANDOM_ACTIVE = 'test/random_sense/active'

# Parameter to set the time required for checking a location(in seconds)
PARAM_CHECKING_TIME = 'test/checking_time'

# Parameter to set the time required for charging the battery (in seconds)
PARAM_CHARGING_TIME = 'test/charging_time'


#---------------------------------------------------------
# ONTOLOGY
# ---------------------------------------------------------
ONTOLOGY_FILENAME = 'topological_map.owl'
ONTOLOGY_IRI = 'http://bnc/exp-rob-lab/2022-23'
ROBOT_NAME = 'Robot1'
STARTING_LOCATION = 'E'
CHARGING_LOCATION = 'E'
URGENCY_THRESHOLD = '15'
CONNECTIONS_LIST = [('E','D6'),
                    ('E','D7'),
                    ('C1','D6'),
                    ('C1','D1'),
                    ('C1','D5'),
                    ('C1','D2'),
                    ('C2','D5'),
                    ('C2','D4'),
                    ('C2','D3'),
                    ('C2','D7'),
                    ('R3','D3'),
                    ('R1','D1'),
                    ('R2','D2'),
                    ('R4','D4')]
TOPIC_CONNECTIONS = 'map/connections'

#---------------------------------------------------------
# NODES NAMES
# ---------------------------------------------------------
# Node representing the shared knowledge required for this scenario.
NODE_ROBOT_STATE = 'robot_state'
# Node implementing the Finite State Machine of the robot
NODE_SMACH_ROBOT = 'smach_robot'
# Node implementing the action server for motion planning
NODE_PLANNER = 'planner'
# Node implementing the action server for motion control
NODE_CONTROLLER = 'controller'
# Node implementing the map builder publisher
NODE_MAP_BUILDER = 'map_builder'

# ---------------------------------------------------------
# POSE
# ---------------------------------------------------------
# Parameter to set the initial robot position
PARAM_INITIAL_POSE = 'state/initial_pose'
# The name of the server to get the current robot pose.
SERVER_GET_POSE = 'state/get_pose'
# The name of the server to set the current robot pose. 
SERVER_SET_POSE = 'state/set_pose'

# ---------------------------------------------------------
#  BATTERY
# ---------------------------------------------------------
# The name of the topic where the battery state is published.
TOPIC_BATTERY_LOW = 'state/battery_level'
# The name of the service to recharge the battery
SERVER_CHARGE = 'state/charge_battery'
# The delay between changes of battery levels, i.e., high/low.
# It should be a list `[min_time, max_time]`, and the battery level change
# will occur after a random number of seconds within such an interval.
PARAM_BATTERY_TIME = 'test/battery_time'


# ---------------------------------------------------------
#   PLANNER
# -------------------------------------------------

# The name of the action server solving the motion planning problem.
ACTION_PLANNER = 'motion/planner'
# The number of points in the plan. It should be a list `[min_n, max_n]`,
# Where the number of points is a random value in the interval [`min_n`, `max_n`).
PARAM_PLANNER_POINTS = 'test/plan_points'
# The delay between the computation of the next via points.
# It should be a list `[min_time, max_time]`, and the computation will 
# last for a random number of seconds in such an interval.
PARAM_PLANNER_TIME = 'test/plan_time'

# -------------------------------------------------
# CONTROLLER
# -------------------------------------------------
# The name of the action server solving the motion control problem.
ACTION_CONTROLLER = 'motion/controller'
# The time required to reach a via points.
# It should be a list `[min_time, max_time]`, and the time to reach a
# via point will be a random number of seconds in such an interval.
PARAM_CONTROLLER_TIME = 'test/motion_time'


# -------------------------------------------------

def tag_log(msg, producer_tag):
    """Function used to label each log with a producer tag.
    
        Args:
            msg (str): message to print
            producer_tag (str): name of the node producer
        Returns:
            string (str): msg tagged by producer
    """
    return '@%s>> %s' % (producer_tag, msg)

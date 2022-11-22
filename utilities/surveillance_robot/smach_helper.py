#! /usr/bin/env python
"""

.. module:: smach_helper
  :platform: Unix 
  :synopsis: Python module for the helper class, used by the module :mod:`smach_robot`
.. moduleauthor:: Mohammad Al Horany, 5271212@studenti.unige.it

This module can be considered as the core of the state machine, by helping it to process its computation.
It implements a specific method for every state plus some other useful method.
It exchanges information with the ontology with the api provided by the `*armor_api* <https://github.com/EmaroLab/armor_py_api>`_ library.
It simulates the robot 2D planning/motion, gets the battery state and build the topological map by interfacing with other modules:
:mod:`planner`, :mod:`controller`, :mod:`robot_state`, :mod:`map_builder`
	
Subscriber of:
	- /state/get_battery
	- /map/connections

Service client of:
	- /state/set_pose
	- /state/charge_battery

Action client of:
	- /motion/planner
	- /motion/controller

"""


import roslib
import rospy
import time
import random
import re
from armor_api.armor_client import ArmorClient
from os.path import dirname, realpath
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolRequest

# Import mutex to manage synchronization
from threading import Lock

# Internal imports
from surveillance_robot import architecture_name_mapper as anm
from surveillance_robot.action_client_helper import ActionClientHelper
from surveillance_robot.msg import Point, DoorConnection, PlanAction, PlanGoal, ControlAction, ControlGoal
from surveillance_robot.srv import SetPose


LOG_TAG = anm.NODE_SMACH_ROBOT
LOOP_SLEEP_TIME = 0.3


class SmachHelper():
	"""This class is the state machine helper. It loads the ontology, initializes the robot location and 2D position, 
		stores useful variables and provides methods to help the state machine in controlling the robot,
		both in 2D Environment and in the ontology. 
	
	"""
	def __init__(self):

		# Get parameters for the simulation
		self.charging_time = rospy.get_param(anm.PARAM_CHARGING_TIME, 3)
		self.checking_time = rospy.get_param(anm.PARAM_CHECKING_TIME, 3)

		#-----------------------#
		# 	BATTERY 						#
		#-----------------------#
		# Initialize the mutex for synchronization
		self.battery_mutex = Lock()

		# Subscribe to the battery low topic and initialize the variable
		rospy.Subscriber(anm.TOPIC_BATTERY_LOW, Bool, self.battery_callback)
		self.battery_low = False

		# Create a service client to recharge the battery
		rospy.wait_for_service(anm.SERVER_CHARGE)
		self.client_recharge = rospy.ServiceProxy(anm.SERVER_CHARGE, SetBool)

		#-----------------------#
		# 	ONTOLOGY 						#
		#-----------------------#
	  # Initialize the mutex for synchronization
		self.map_mutex = Lock()

	  # Start the ArmorClient and loads the ontology file
		path = dirname(realpath(__file__))
		path = path + "/../../ontologies/"
		self.armor_client = ArmorClient("surveillance_robot", "reference")
		self.armor_client.utils.load_ref_from_file(path + anm.ONTOLOGY_FILENAME, anm.ONTOLOGY_IRI, True, "PELLET", True, False)
		self.armor_client.utils.mount_on_ref()
		self.armor_client.utils.set_log_to_terminal(True)

		# Variables of the ontology map
		self.locations_list = []
		self.corridors_list = []
		self.doors_list = []
		self.current_location = anm.STARTING_LOCATION 
		self.target_location = None
		self.urgent_locations = []

		# Subscribe to the map/connections topic to get informations about the map
		rospy.Subscriber(anm.TOPIC_CONNECTIONS, DoorConnection, self.map_callback)
		self.map_built = False
		
		#-----------------------#
		# 	2D ENVIRONMENT		#
		#-----------------------#
		# Get the environment size and set the initial 2D pose of the robot
		self.environment_size = rospy.get_param(anm.PARAM_ENVIRONMENT_SIZE, [10,10])
		self.initial_pose = rospy.get_param(anm.PARAM_INITIAL_POSE, [0, 0])
		self.initial_point = Point()
		self.initial_point.x = self.initial_pose[0]
		self.initial_point.y = self.initial_pose[1]
		self.init_robot_pose(self.initial_point)

    # Define the clients for the the plan and control action servers.
		self.planner_client = ActionClientHelper(anm.ACTION_PLANNER, PlanAction)
		self.controller_client = ActionClientHelper(anm.ACTION_CONTROLLER, ControlAction)


	def build_map(self):
		""" Method paired with BUILD_MAP state.
			It assert the robot initial location, then waits for the map information.
			Once map has been built, it disjoints all the individuals, initializes the timer of locations and the urgency threshold. 
			It uses, from the architecture_name_mapper file:

			- ROBOT_NAME: the name of the robot
			
			- URGENCY_THRESHOLD: the threshold of urgency, after which a location becomes URGENT
		
		"""

		# Assert the robot initial location
		self.armor_client.manipulation.add_objectprop_to_ind('isIn', anm.ROBOT_NAME, anm.STARTING_LOCATION)
		log_msg = (f'[BUILD_MAP] = ROBOT {anm.ROBOT_NAME} spawned in ROOM {anm.STARTING_LOCATION},' +
					' waiting for the information to build the map.')
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))


		# Loop for checking in the map builder has finished publishing connections 
		while not rospy.is_shutdown():
			self.map_mutex.acquire()
			try:

				# Verify if all the connections has been published
				if self.map_built is True:
				
					# Disjoint all the individuals
					self.armor_client.call('DISJOINT', 'IND', '', self.locations_list+self.doors_list+[anm.ROBOT_NAME])

					# Initialize the timer of the locations and of the robot
					starting_time = str(int(time.time()))
					for location in self.locations_list:
						if location is not anm.CHARGING_LOCATION:
							self.armor_client.manipulation.add_dataprop_to_ind("visitedAt", location, "Long", starting_time)

					self.armor_client.manipulation.add_dataprop_to_ind("now", anm.ROBOT_NAME, "Long", starting_time)
					self.last_move_time = starting_time

					# Initialize the urgency threshold of the robot
					self.armor_client.manipulation.add_dataprop_to_ind("urgencyThreshold", anm.ROBOT_NAME, "Long", anm.URGENCY_THRESHOLD)

					# Apply changes and calls the ontology-reasoner
					self.update_ontology()

					# Save the corridors list to avoid future queries
					self.corridors_list = self.format_query(self.armor_client.query.ind_b2_class('CORRIDOR'), 'LOCATION')

					break;
					
			finally:
				self.map_mutex.release()

			rospy.sleep(LOOP_SLEEP_TIME)

		



	def decide_next_location(self):
		""" Method paired with the REASONER state.

			It decides the next location by querying the ontology. The policy of the decision is the following:
				- If battery_low, next_location = charging location
				- If urgent locations nearby, next_location = most urgent reachable location
				- Else, next_location = random CORRIDOR among the reachable

		    Returns:
		    	next_location(str): location to reach

		"""
		# If the battery is low, go to room E
		self.battery_mutex.acquire()
		try:

			if self._battery_low:
				return anm.CHARGING_LOCATION

		finally:
			self.battery_mutex.release()

		# Get reachable rooms
		reachable_locations = self.format_query(self.armor_client.query.objectprop_b2_ind("canReach", anm.ROBOT_NAME), "LOCATION")
		random.shuffle(reachable_locations)

		log_msg = (f'[REACH-LOCATION / REASONER] = From here, I can reach {reachable_locations}.')
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

		# If there are urgent rooms nearby, go to the most urgent
		next_location = self.get_most_urgent(reachable_locations)
		if next_location is not None and next_location is not anm.CHARGING_LOCATION:
			log_msg = (f'[REACH-LOCATION / REASONER] = Urgent room {next_location}!! Ill go there.')
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			return next_location

    # Otherwise go to a corridor
		else:
			for location in reachable_locations:
				if location in self.corridors_list and location is not anm.CHARGING_LOCATION:
					log_msg = (f'[REACH-LOCATION / REASONER] = No urgent locations here, Ill go in CORRIDOR {location}.')
					rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
					return location


	def get_most_urgent(self, reachable_locations):
		""" This method is used by `decide_next_location()` and it returns the most urgent location among the ones given in input, if any, otherwhise it returns False.

		    Args:
		    	reachable_locations (str[]): The list of reachable locations
		  
		    Returns:
		    	most_urgent_location(str): The most urgent location among the reachable ones; if none of them are URGENT,
		    								this is None.
		
		"""

		# Get the urgent locations
		self.urgent_locations = self.format_query(self.armor_client.query.ind_b2_class('URGENT'), "LOCATION")

		# Get the intersection among REACHABLE and URGENT locations
		urgent_reachable_locations = [location for location in self.urgent_locations if location in reachable_locations]

		# If there are not reachable urgent locations urgent, return None 
		if not urgent_reachable_locations:
			return None

		# Return the most urgent among the reachable urgent locations
		last_visit = 0;
		most_urgent_location = urgent_reachable_locations[0]
		for location in urgent_reachable_locations:
				query = int(self.format_query(self.armor_client.query.dataprop_b2_ind("visitedAt", location), "TIMESTAMP")[0])
				if last_visit > query:
					most_urgent_location = location
					last_visit = query
		return most_urgent_location



	def get_plan_to_goal(self, goal_location):
		""" Method paired with the PLANNER state.
			It simulates the planning by calling the `planner` action-server and verifying the battery level in the meanwhile.
		   
		    Args:
		    	goal_location (str): The goal location to reach

		    Returns:
		    	plan_viapoints(Point[]): List of points of the plan; if battery got low, this is simply False.
		
		"""
		# Reset the state of previous stimuli to assure that only the new stimulus are considered.
		self.planner_client.reset_client_states()

    # Call the planning action goal for computing the via-points toward the goal location.
		goal = PlanGoal()
		goal.target = Point(x=random.uniform(0, self.environment_size[0]),
							y=random.uniform(0, self.environment_size[1]))

		self.planner_client.send_goal(goal)
    
    # Wait for the result checking if the battery get low
		while not rospy.is_shutdown():

      # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
			self.battery_mutex.acquire()
			try:

        # If the battery is low, then cancel the planning action server and return False
				if self._battery_low and goal_location is not anm.CHARGING_LOCATION:  # Higher priority
					if self.planner_client.is_running():
						self.planner_client.cancel_goals()
					return False


        # If the planner finishes its computation, then return the viapoints computed
				if self.planner_client.is_done():
					return self.planner_client.get_results().via_points


			finally:

      	# Release the mutex
				self.battery_mutex.release()

      # Wait for a reasonably small amount of time to allow processing stimulus (eventually).
			rospy.sleep(LOOP_SLEEP_TIME)


	def move_robot(self, plan_viapoints, new_location):
		"""Method paired with the CONTROLLER state.
			It simulates the motion by calling the `controller` action-server and verifying the battery level in the meanwhile.
			Once arrived, it updates the robot location in the ontology ("isIn" object prop.) and the last time it moved ("now" data prop.)

		    Args:
					plan_viapoints (Point[]): The planned viapoint to reach the goal location.
		    	goal_location (str): The goal location

		    Returns:
		    	bool(Bool): True if it finishes correctly, False if battery got low
		
		"""

		# Reset the state of previous stimuli to assure that only the new stimulus are considered.
		self.controller_client.reset_client_states()

    # Start the action server for moving the robot through the planned via-points.
		goal = ControlGoal(via_points=plan_viapoints)
		self.controller_client.send_goal(goal)

    # Wait for the action server computation and listen possible incoming stimulus.
		while not rospy.is_shutdown():

      # Acquire the mutex to assure data consistencies
			self.battery_mutex.acquire()
			try:

				# If the battery is low, then cancel the control action goal and return False
				if self._battery_low and new_location is not anm.CHARGING_LOCATION:  # Higher priority
					if self.controller_client.is_running():
						self.controller_client.cancel_goals()
					return False

        # If the controller finishes its computation, then break to update the position in the ontology
				if self.controller_client.is_done():
					break;

			finally:
                
        # Release the mutex
				self.battery_mutex.release()

      # Wait for a reasonably small amount of time to allow the processing of battery state change
			rospy.sleep(LOOP_SLEEP_TIME)

		# Update the robot location in the ontology ("isIn" object prop.) and the last time it moved ("now" data prop.)
		move_time_new = str(int(time.time()))
		self.armor_client.manipulation.replace_objectprop_b2_ind("isIn", anm.ROBOT_NAME, new_location, self.current_location)
		self.current_location = new_location
		self.armor_client.manipulation.replace_dataprop_b2_ind("now", anm.ROBOT_NAME, "Long", move_time_new, self.last_move_time)
		self.last_move_time = move_time_new
		self.update_ontology()

		return True

	
	def check_location(self, location):
		"""Method paired with the CHECK_LOCATION state. It simulates the checking by wasting time and verifying the
			battery level in the meanwhile.

		    Args:
		    	location (str[]): The location to check

		    Returns:
		    	bool (Bool): True if it finishes correctly, False if battery got low
		
		"""

		# Simulate the checking by wasting time and checking the battery in the meanwhile
		for i in range(10):
			# Acquire mutex for synchronization
			self.battery_mutex.acquire()
			try:

				if self._battery_low:
					return False

			finally:
				self.battery_mutex.release()
		
			rospy.sleep(self.checking_time/10)
		
		# Update the "visitedAt" data property of the location when check is finished
		new_visit_time = str(int(time.time()))
		last_visit_time = self.format_query(self.armor_client.query.dataprop_b2_ind('visitedAt', location), "TIMESTAMP")[0]
		self.armor_client.manipulation.replace_dataprop_b2_ind('visitedAt', location, 'Long', new_visit_time, last_visit_time)
		self.update_ontology()

		return True


	def recharge(self):
		"""Method used to send a request to the service that recharges the battery.

		"""
		log_msg = f'[CHARGE] = Recharging...'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

		request = SetBoolRequest()             
		request.data = True                    
		self.client_recharge(request)  

		log_msg = f'[CHARGE] = Robot fully charged!'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

		self.battery_mutex.acquire()
		self._battery_low = False
		self.battery_mutex.release()


	def update_ontology(self):
		"""Method to apply the changes in the buffer and to synchronyze the reasoner.
			It is not protected by mutex because it is called always by the manipulations methods
			that already have the mutex.
		
		"""
		self.armor_client.utils.apply_buffered_changes()
		self.armor_client.utils.sync_buffered_reasoner()


	def battery_callback(self, msg):
		"""The subscriber of the `/state/battery_low/` topic.
		It acquires the changes of states of the battery.
				
				Args:
					msg (Bool) : represents the battery state
		"""
		# Get the battery level and set the relative state variable encoded in this class with the mutex
		self.battery_mutex.acquire()
		self._battery_low = msg.data
		self.battery_mutex.release()


	def map_callback(self, msg):
		"""The subscriber callback of the `/state/battery_low/` topic.
		It receive connection pairs of LOCATION/DOOR, and adds them to the ontology 
				
				Args:
					msg (DoorConnection) : represents the connection between a DOOR and a LOCATION
		"""
		if msg.door:
			# Adds the connection to the ontology
			self.armor_client.manipulation.add_objectprop_to_ind("hasDoor", msg.location, msg.door)
			
			# Store the location and door list
			if msg.location not in self.locations_list:
				self.locations_list.append(msg.location)
			if msg.door not in self.doors_list:	
					self.doors_list.append(msg.door)

			# Log message
			log_msg = f'[BUILD_MAP] = Connection acquired: Location {msg.location} has Door {msg.door} + '
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

		# The empty message represents the end of the acquiring process
		else:

			self.map_mutex.acquire()
			self.map_built = True
			self.map_mutex.release()


	@staticmethod
	def format_query(old_list, type_of_object):
		"""Function to format a list of query strings, by extracting only the meaningful part.
		    
		    Args:
		       old_list (str[]): The list of queries to format.
		       type_of_object (str[]): The query type, that can be 'LOCATION' or 'TIMESTAMP'
		    
		    Returns:
		       formatted_list (str[]): The formatted list of queries.
		
		"""
		start_position = 0
		formatted_list = []
		if type_of_object == 'LOCATION':
			start_position = 32
			end_position = -1
		elif type_of_object == 'TIMESTAMP':
			start_position = 1
			end_position = -11
		for obj in old_list:
			formatted_list.append(obj[start_position:end_position])
		return formatted_list

	@staticmethod
	def init_robot_pose(point):
		"""Function to initialize the robot 2D Pose, by using the service `robot/set_pose`.
		    
		    Args:
		       point (Point): 2D Point representing the initial pose.
	    
	  """
		# Eventually, wait for the server to be initialised.
		rospy.wait_for_service(anm.SERVER_SET_POSE)
		try:
			# Call the service and set the current robot position.
			service = rospy.ServiceProxy(anm.SERVER_SET_POSE, SetPose)
			service(point)  # None that the service `response` is not used.
			log_msg = f'[BUILD_MAP] = Setting initial robot position ({point.x}, {point.y}) to the `{anm.SERVER_SET_POSE}` node.'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		except rospy.ServiceException as e:
			err_msg = f'Cannot set current robot position through `{anm.SERVER_SET_POSE}` server. Error: {e}'
			rospy.logerr(anm.tag_log(err_msg, LOG_TAG))
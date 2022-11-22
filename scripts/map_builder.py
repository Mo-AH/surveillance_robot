#!/usr/bin/env python
"""
.. module:: map_builder
  :platform: Unix 
  :synopsis: Python module for the node that build the map
.. moduleauthor:: Mohammad Al Horany 5271212@studenti.unige.it

This module implements the map_builder node, to simulate a map acquisition process.
It publishes in the `/map/connections` topic every pair DOOR/LOCATION (DoorConnection msg) of the ontological map.

Publishes to:
  /map/connections


"""

import rospy
import random

from surveillance_robot import architecture_name_mapper as anm
from surveillance_robot.msg import DoorConnection

# Tag for identifying logs producer.
LOG_TAG = anm.NODE_MAP_BUILDER


def build_map(publisher):
	"""Method to publish pairs DOOR/LOCATION for the map building process in the `/map/connections` topic.
	
	Args:
	    publisher (rospy.Publisher): publisher to the map topic
	"""
	for location, door in anm.CONNECTIONS_LIST:

		# Create and fill the message
		msg = DoorConnection()
		msg.door = door
		msg.location = location
		publisher.publish(msg)

		log_msg = f' Connection published: Door {msg.door} + Location {msg.location}'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		
		# Waste time to simulate the map acquisition process
		rospy.sleep(0.2)

	# Publish an empty msg to comunicate the end of the building process
	msg = DoorConnection()
	publisher.publish(msg)


if __name__ == '__main__':
	"""Initializes the build_map node.
			After having published all the connections, it exit.
	"""

	rospy.init_node(anm.NODE_MAP_BUILDER, log_level=rospy.INFO)

	publisher = rospy.Publisher(anm.TOPIC_CONNECTIONS, DoorConnection, queue_size=1, latch=True)

	log_msg = f'`{anm.NODE_MAP_BUILDER}` node initialized. It will begin publish connections between DOORs and LOCATIONs in topic `{anm.TOPIC_CONNECTIONS} in 5 seconds.`.'
	rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
	rospy.sleep(5) 

	build_map(publisher)

	log_msg = f'`{anm.NODE_MAP_BUILDER}` node has terminated the map acquiring process. This node will terminate in 5 seconds.'
	rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
	rospy.sleep(5) 

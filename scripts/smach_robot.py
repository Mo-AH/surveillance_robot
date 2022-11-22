#! /usr/bin/env python
"""
.. module:: smach_robot 
  :platform: Unix 
  :synopsis: Python module for the state machine implementation
.. moduleauthor:: Mohammad Al Horany, 5271212@studenti.unige.it

This module implements a node running the state machine of the robot.
It manages the transitions between states, leaving the computation processes to the helper
class of the smach given by the module :mod:`state_machine_helper`.

"""

import roslib
import rospy
import smach
import smach_ros
import time
import random
from armor_api.armor_client import ArmorClient

# Internal imports
from surveillance_robot import architecture_name_mapper as anm
from surveillance_robot.smach_helper import SmachHelper

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_SMACH_ROBOT


class BuildMap(smach.State):
    """This class defines the state BUILD_MAP of the state machine.
    """
    def __init__(self, smach_helper):
        smach.State.__init__(self, 
                             outcomes=['map_built']) 
        self.smach_helper = smach_helper
        
    def execute(self, userdata):
        """Execute method of the state BUILD_MAP
        
            Returns:
                string (str): transition to the next state.
        """

        # Log message
        log_msg = (f'[BUILD_MAP] = Building the map...')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        # Build the map
        self.smach_helper.build_map()

        # Log message
        log_msg = (f'[BUILD_MAP] = Map built!! \n Locations = {self.smach_helper.locations_list} \n Corridors =' +
                    f'{self.smach_helper.corridors_list} \n Charging location = {anm.CHARGING_LOCATION}.')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        rospy.sleep(3)

        return 'map_built'
    

class Reasoner(smach.State):
    """This class defines the inner state REACH_LOCATION / REASONER of the state machine.
    """

    def __init__(self, smach_helper):
        smach.State.__init__(self, 
                             outcomes=['goal_decided'],
                             input_keys=['goal_location'],
                             output_keys=['goal_location'])
        self.smach_helper = smach_helper

    def execute(self, userdata):
        """Execute method of the inner state REACH_LOCATION / REASONER
        
            Returns:
                string (str): transition to the next state.
        """
        # Decide next location and pass it in the planner state
        userdata.goal_location = self.smach_helper.decide_next_location()

        # If it's leading to the charging location, log the message
        if userdata.goal_location == anm.CHARGING_LOCATION:
            log_msg = (f'[REACH-LOCATION / REASONER] = Battery low!! Ill go in ROOM {anm.CHARGING_LOCATION}.')
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        return 'goal_decided'

class Planner(smach.State):
    """This class defines the inner state REACH_LOCATION / PLANNER of the state machine.
    """

    def __init__(self, smach_helper):
        smach.State.__init__(self, 
                             outcomes=['plan_ready', 'battery_low'],
                             input_keys=['plan_to_goal','goal_location'],
                             output_keys=['plan_to_goal', 'goal_location'])
        self.smach_helper = smach_helper

    def execute(self, userdata):
        """Execute method of the inner state REACH_LOCATION / PLANNER
        
            Returns:
                string (str): transition to the next state.
        """
        # Log message
        rospy.loginfo(anm.tag_log(f'[REACH_LOCATION / PLANNER] = Preparing the plan to reach {userdata.goal_location}...', LOG_TAG))

        # Define a plan to follow (set of via points) and pass it to the controller state
        userdata.plan_to_goal = self.smach_helper.get_plan_to_goal(userdata.goal_location)
        
        # If the battery got low while planning, transit to REASONER inner state
        if not userdata.plan_to_goal:
            log_msg = (f'[REACH_LOCATION / PLANNER] = Battery got low while planning !!')
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
            return 'battery_low'

        # Log Message
        log_msg = (f'[REACH_LOCATION / PLANNER] = The plan is ready !')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        return 'plan_ready'


class Controller(smach.State):
    """This class defines the inner state REACH_LOCATION / CONTROLLER of the state machine.
    """
    def __init__(self, smach_helper):
        smach.State.__init__(self, 
                             outcomes=['location_not_urgent_reached',
                                        'location_urgent_reached',
                                        'charging_station_reached',
                                        'battery_low'],
                             input_keys=['plan_to_goal', 'goal_location'],
                             output_keys=['current_location'])
        self.smach_helper = smach_helper

    def execute(self, userdata):
        """Execute method of the inner state REACH_LOCATION / CONTROLLER
        
            Returns:
                string (str): transition to the next state.
        """
        # Log Message
        log_msg = (f'[REACH_LOCATION / CONTROLLER] = Moving to {userdata.goal_location}... ')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        # If the battery got low while moving, transit to REASONER inner state
        if not self.smach_helper.move_robot(userdata.plan_to_goal, userdata.goal_location):
            log_msg = (f'[REACH_LOCATION / CONTROLLER] = Battery got low while moving !!')
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
            return 'battery_low'

        userdata.current_location = userdata.goal_location

        # If it the location reached is the charging one, transit to the CHARGE state
        if userdata.goal_location == anm.CHARGING_LOCATION:
            log_msg = (f'[REACH-LOCATION / CONTROLLER] = Charging station {userdata.goal_location} reached!')
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
            return 'charging_station_reached'

        # If the location reached is not urgent, don't check it
        if userdata.goal_location not in self.smach_helper.urgent_locations:
            log_msg = (f'[REACH-LOCATION / CONTROLLER] = Location {userdata.goal_location} reached. It is not urgent, so I will skip the checking!')
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
            return 'location_not_urgent_reached'

        # Log Message
        log_msg = (f'[REACH-LOCATION / CONTROLLER] = Location {userdata.goal_location} reached!')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        return 'location_urgent_reached'


class CheckLocation(smach.State):
    """This class defines the state CHECK_LOCATION of the state machine.
    """
    def __init__(self,smach_helper):
        smach.State.__init__(self, 
                             outcomes=['battery_low', 'check_complete'],
                             input_keys=['current_location'])
        self.smach_helper = smach_helper


    def execute(self, userdata):
        """Execute method of the state CHECK_LOCATION.

            Returns:
                string (str): transition to the next state.
        """
        # Log Message
        log_msg = (f'[CHECK_LOCATION] = Starting the check of location {userdata.current_location}...')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        # If the battery got low while checking, transit to REACH_LOCATION state
        if not self.smach_helper.check_location(userdata.current_location):
            log_msg = (f'[CHECK_LOCATION] = Battery got low while checking !!')
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
            return 'battery_low'
        
        # Log message
        log_msg = (f'[CHECK_LOCATION] = Location {userdata.current_location} checked! ')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        return 'check_complete'



class Charge(smach.State):
    """This class defines the state CHARGE of the state machine.
    """
    def __init__(self,smach_helper):
        smach.State.__init__(self, outcomes=['charge_complete'])
        self.smach_helper = smach_helper

    def execute(self, userdata):
        """Execute method of the state CHARGE.
        
            Returns:
                string (str): transition to the next state.
        """

        # Call the service to recharge the battery
        self.smach_helper.recharge()

        return 'charge_complete'

def main():
    rospy.init_node('robot_states') #inizializza il nodo

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])


    # Create the smach_helper for the state machine
    smach_helper = SmachHelper();

    # Open the container (the first state is the one that starts)
    with sm:
        
        # Add states to the container
        smach.StateMachine.add('BUILD_MAP',BuildMap(smach_helper),
                               transitions={'map_built':'REACH_LOCATION'})

        # Create a sub state machine and define the inner states
        reach_location_subsm = smach.StateMachine(outcomes=['location_urgent_reached','charging_station_reached'],
                                                output_keys=['current_location'])
        
        with reach_location_subsm:


            smach.StateMachine.add('REASONER', Reasoner(smach_helper),                                
                                   transitions={'goal_decided':'PLANNER'})

            smach.StateMachine.add('PLANNER', Planner(smach_helper), 
                                   transitions={'plan_ready':'CONTROLLER',
                                                'battery_low':'REASONER'})

            smach.StateMachine.add('CONTROLLER', Controller(smach_helper), 
                                   transitions={'charging_station_reached': 'charging_station_reached',
                                                'location_not_urgent_reached' : 'REASONER',
                                                'location_urgent_reached':'location_urgent_reached',
                                                'battery_low':'REASONER'})
        # Add the sub state machine to the container    
        smach.StateMachine.add('REACH_LOCATION', reach_location_subsm,
                               transitions={ 'location_urgent_reached':'CHECK_LOCATION',
                                            'charging_station_reached':'CHARGE'})
        
        smach.StateMachine.add('CHECK_LOCATION', CheckLocation(smach_helper),                                 
                               transitions={'battery_low': 'REACH_LOCATION',
                                            'check_complete': 'REACH_LOCATION'})
        
        smach.StateMachine.add('CHARGE', Charge(smach_helper),                                 
                               transitions={'charge_complete': 'REACH_LOCATION'})        


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()

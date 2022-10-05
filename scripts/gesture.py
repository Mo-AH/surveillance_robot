#!/usr/bin/env python

import random
import rospy
# Import constant name defined to structure the architecture.
import architecture_name_mapper as anm
# Import the message type (and sub-type) to be published.
from arch_skeleton.msg import Point, Gesture


# A tag for identifying logs producer.
LOG_TAG = anm.NODE_GESTURE


# Initialise a new message that this node will publish.
# The published message is of type `Gesture.msg`.
def init_message():
    msg = Gesture()
    msg.stamp = rospy.Time.now()
    msg.coordinate = Point()
    return msg


# Publish a random message with a pointed location as a 2D pose.
# Each message generated by this method are published through the `publisher`  
# input parameter. The random pont generated are within the intervals 
# x: [0, `environment_size[0]`) and y: [0, `environment_size[1]`). Each message 
# is published with a delay random chosen in the range
# [`gesture_timing[0]`, `gesture_timing[1]`) seconds.
def generate_random_gesture(publisher, environment_size, gesture_timing):
    # Generate the random message to be published.
    msg = init_message()
    # Also generate points out of environment (i.e., `+2`) for testing purposes.
    msg.coordinate.x = random.uniform(0, environment_size[0] + 2)  
    msg.coordinate.y = random.uniform(0, environment_size[1] + 2)
    # Publish the message.
    publisher.publish(msg)
    log_msg = 'Publishing random user gesture: (%f, %f).' % (msg.coordinate.x, msg.coordinate.y)
    rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
    # Wait before the next random message is published.
    delay = random.uniform(gesture_timing[0], gesture_timing[1])
    rospy.sleep(delay)


# Allow keyboard interaction to emulate gesture-based user's command.
def generate_manual_gesture(publisher, environment_size):
    try:
        # Generate the message to be published.
        msg = init_message()
        # Wait until Enter is pressed and get the typed text.
        user_input = raw_input(' > ')
        user_input = user_input.lower()
        # Understand the entered text.
        coordinate = [i.strip() for i in user_input.split(',')]
        msg.coordinate.x = float(coordinate[0])
        msg.coordinate.y = float(coordinate[1])
        # Check the correctness of the entered data.
        if len(coordinate) != 2:
            log_msg = 'A not 2D coordinate is given, i.e., `%s`.' % user_input
            rospy.logwarn(anm.tag_log(log_msg, LOG_TAG))
        if msg.coordinate.x < 0 or msg.coordinate.x > environment_size[0] \
                or msg.coordinate.y < 0 or msg.coordinate.y > environment_size[1]:
            log_msg = 'Entered coordinates are out of the environment, i.e., `%s`.' % user_input
            rospy.logwarn(anm.tag_log(log_msg, LOG_TAG))
        # Publish the message.
        publisher.publish(msg)
        log_msg = 'Publishing entered user gesture: (%f, %f).' % (msg.coordinate.x, msg.coordinate.y)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
    except Exception as e:
        # Not understood user's command.
        print('*** USER INPUT ERROR: ' + str(e) + '! Try again:')


# Initialise and run a node that publishes gesture-like commands.
if __name__ == '__main__':
    # Get parameter and initialise this node as well as its publisher.
    rospy.init_node(anm.NODE_GESTURE, log_level=rospy.INFO)
    environment_size = rospy.get_param(anm.PARAM_ENVIRONMENT_SIZE)
    randomness = rospy.get_param(anm.PARAM_RANDOM_ACTIVE, True)
    publisher = rospy.Publisher(anm.TOPIC_GESTURE, Gesture, queue_size=1, latch=True)
    log_msg = 'Initialise node `%s` with topic `%s`.' % (anm.NODE_GESTURE, anm.TOPIC_GESTURE)
    rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

    if randomness:
        # Configure node based on parameters to generate random gesture-based data.
        gesture_timing = rospy.get_param(anm.PARAM_GESTURE_TIME, [2.0, 30.0])
        log_msg = 'Random-based data generation active: a random command with a delay in the range of ' \
                  '[%f, %f) seconds.' % (gesture_timing[0], gesture_timing[1])
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
    else:
        # Explain keyboard-based interaction.
        print('  # Type a pointed location as `x, y` (e.g., `0.4, 5.2`).')
        print('  # Type `cnt+C` and `Enter` to quit.')
        
    while not rospy.is_shutdown():
        if randomness:
            # Generate random gesture-like data.
            generate_random_gesture(publisher, environment_size, gesture_timing)
        else:
            # Allow keyboard interaction to generate gesture-like data.
            generate_manual_gesture(publisher, environment_size)


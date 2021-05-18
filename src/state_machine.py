#!/usr/bin/env python3

import rospy
#import roslib; roslib.load_manifest('smach_tutorials')
import smach
import smach_ros
from std_msgs.msg import String
from uav_control import UAV_Control
# from voice_rec import received_word

class Ready(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['outcome1', 'outcome2'])
        rospy.Subscriber('/voice_recognition', String, self.received_word)
        self.r_word = False

    def received_word(self, msg):
        self.r_word = True

    def execute(self, userdata):
        rospy.loginfo('State: Ready')
        if self.r_word == True:
            return 'outcome2'
        else:
            return 'outcome1'

class Moving(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['outcome1'])

    def execute(self):
        rospy.loginfo('State: Moving')
        uc = UAV_Control()
        UAV_Control.run(uc)

        return 'outcome1'

def main():
    rospy.init_node('state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes = ['outcome1'])

    # Open the container
    with sm:

        # Add states to the container
        smach.StateMachine.add('READY', Ready(), transitions = {'outcome1':'READY', 'outcome2':'MOVING'})
        smach.StateMachine.add('MOVING', Moving(), transitions = {'outcome1':'READY'})

    outcome = sm.execute()

if __name__ == '__main__':
    main()
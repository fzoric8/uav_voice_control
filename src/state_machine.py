#!/usr/bin/env python3

import rospy
#import roslib; roslib.load_manifest('smach_tutorials')
import smach
import smach_ros
from std_msgs.msg import String
from uav_control import UAV_Control
from uav_voice_control.srv import ProvideMove, ProvideMoveResponse
# from voice_rec import received_word

r_word = False

class Ready(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])
        rospy.Subscriber('/voice_recognition', String, self.received_word)
        global r_word

    def received_word(self, msg):
        global r_word
        r_word = True

    def execute(self, userdata):
        rospy.loginfo('State: Ready')
        rospy.sleep(1)
        if r_word == True:
            return 'outcome2'
        else:
            return 'outcome1'

class Moving(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])
        global r_word

    def execute(self, userdata):
        rospy.loginfo('State: Moving')
        rospy.sleep(1)
        # self.r_word = False

        print('1')
        rospy.wait_for_service('/uav/provide_move')
        print('2')
        try:
            self.resp1 = ProvideMoveResponse()
            provide_move = rospy.ServiceProxy('/uav/provide_move', ProvideMove)
            self.resp1 = provide_move(r_word)

        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        if self.resp1 == True:
            return 'outcome1'
        else:
            return 'outcome2'


def main():
    rospy.init_node('state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome1'])

    # Open the container
    with sm:

        # Add states to the container
        smach.StateMachine.add('READY', Ready(), transitions={'outcome1': 'READY', 'outcome2': 'MOVING'})
        smach.StateMachine.add('MOVING', Moving(), transitions={'outcome1': 'READY', 'outcome2': 'MOVING'})

    outcome = sm.execute()
    uc = UAV_Control()
    UAV_Control.run(uc)


if __name__ == '__main__':
    main()
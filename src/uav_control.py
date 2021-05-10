#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


class UAV_Control():

    def __init__(self):
        self.pub_command = rospy.Publisher('/command', Pose, queue_size=10)
        self.detected_word = String()
        self.position = Point()
        self.orientation = Quaternion()
        self.odom = Odometry()
        self.rate = rospy.Rate(10)
        rospy.Subscriber('/voice_recognition', String, self.command_callback)
        self.odom = rospy.Subscriber('/uav/odometry', Odometry, self.odometry_callback)

        self.position.x = 0
        self.position.y = 0
        self.position.z = 0
        self.orientation.x = 0
        self.orientation.y = 0
        self.orientation.z = 0
        self.orientation.w = 0

    def odometry_callback(self, msg):
        self.odom = msg

    def command_callback(self, msg):
        self.detected_word = str(msg.data)
        print(self.detected_word)

        if 'land' in self.detected_word:
            self.position.z = 1
            time.sleep(2)
            self.position.z = 0.5
            time.sleep(2)
            self.position.z = 0

        if 'climb' in self.detected_word:
            if 'one' in self.detected_word:
                self.position.z += 0.1
                print("Penjem se 10cm")
            elif 'two' in self.detected_word:
                self.position.z += 0.2
                print("Penjem se 20cm")
            elif 'three' in self.detected_word:
                self.position.z += 0.3
                print("Penjem se 30cm")
            elif 'ten' in self.detected_word:
                self.position.z += 1
                print("Penjem se 100cm")
            self.orientation.w = 1

        if self.detected_word == 'stop':
            self.position.x = 0
            self.position.y = 0
            self.position.z = 0
            self.orientation.x = 0
            self.orientation.y = 0
            self.orientation.z = 0
            self.orientation.w = 0
            print('uav_control:stop')

        if 'forward' in self.detected_word:
            self.position.x = 1
            self.position.y = 1
            self.position.z = 1
            self.orientation.x = 1
            self.orientation.y = 1
            self.orientation.z = 1
            self.orientation.w = 1
            print('uav_control:forward')

        if 'takeoff' in self.detected_word:
            self.position.z = 0.5
            time.sleep(3)
            self.position.z = 1
            print('uav_control:take_off')
    def run(self):
        while not rospy.is_shutdown():
            self.pub_command.publish(self.position, self.orientation)
            #if self.pub_command.publish(self.position, self.orientation) != 'None':
                #print(self.pub_command.publish(self.position, self.orientation))

if __name__ == '__main__':
    rospy.init_node('uav_control')
    uc = UAV_Control()
    uc.run()


# TODO subscribat na odometriju - uzeti poziciju s odometrije i poslati pola metra
# land -> na 10 cm i onda na nulu
# home - (0,0, 1)
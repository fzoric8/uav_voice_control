#!/usr/bin/env python3

import rospy
import time
import yaml
import math
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


class UAV_Control():

    def __init__(self):

        self.detected_word = String()

        self.rate = rospy.Rate(10)
        self.detected_word = ''

        rospy.Subscriber('/voice_recognition', String, self.command_callback)

        self.l_wing_pub = rospy.Publisher("/cmd_left_wing_deg", Float64, queue_size=1)
        self.r_wing_pub = rospy.Publisher("/cmd_right_wing_deg", Float64, queue_size=1)

        self.r_open_val = Float64(data=90); self.r_close_val = Float64(data=0)
        self.l_open_val = Float64(data=0); self.l_close_val = Float64(data=90)

        rospy.spin()

    def command_callback(self, msg):
        #https://pythonlang.dev/repo/bambocher-pocketsphinx-python/
        self.detected_word = str(msg.data)

        rospy.loginfo("Detected word!")

        if "open" in self.detected_word:
            
            rospy.loginfo("Detected open! ")
            self.r_wing_pub.publish(self.r_open_val); 
            self.l_wing_pub.publish(self.l_open_val);             


        if "close" in self.detected_word: 

            rospy.loginfo("Detected close! ")
            self.r_wing_pub.publish(self.r_close_val); 
            self.l_wing_pub.publish(self.l_close_val); 



if __name__ == '__main__':

    try:
        rospy.init_node('uav_control')
        uc = UAV_Control()

    except Exception as e:
        print(e)

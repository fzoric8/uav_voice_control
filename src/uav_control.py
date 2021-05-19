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
        rospy.Subscriber('/uav/odometry', Odometry, self.odometry_callback)

        self.position.x = self.odom.pose.pose.position.x
        self.position.y = self.odom.pose.pose.position.y
        self.position.z = self.odom.pose.pose.position.z
        self.orientation.x = self.odom.pose.pose.orientation.x
        self.orientation.y = self.odom.pose.pose.orientation.y
        self.orientation.z = self.odom.pose.pose.orientation.z
        self.orientation.w = self.odom.pose.pose.orientation.w

    def odometry_callback(self, msg):
        self.odom = msg

    def command_callback(self, msg):
        self.detected_word = str(msg.data)
        print(self.detected_word)

        ## TODO: LOGIC FOR Sending commands put into run method! Not in callback, we could use while to block command execution and 
        ## Wait for UAV to reach certain reference
        

        ### FROM HERE
        if 'takeoff' in self.detected_word:
            if self.position.z == 0:
                self.position.z = 0.5
                time.sleep(3)
                self.position.z = 1
                print('uav_control: take_off')
            else:
                print('uav: airborne')

        if 'land' in self.detected_word:
            while not self.current_pose.position.z < 0.1:
                
                # TODO: Create cmd_pose variable and initailze it as empty Pose message in constructor, use it for sending commands
                # self.cmd_pose.position.z = self.current_pose.position.z - 0.1 
                # My idea was to use knowledge of current pose(position/orientation) and to decrease height value until we hit ground. 
                
                self.cmd_pose.position.z = self.current_pose.position.z - 0.1
                # INVOKE publish in while loop / CAN BE IN send_command method
                # self.publish....
                # USE WHILE TO


        if 'climb' in self.detected_word:
            epsilon_ = 0.01 
            # Nije tako trivijalno jer treba uzeti u obzir zadanu referencu (self.voice_detected_increment)
            while not abs(self.current_pose.position.z - self.cmd_pose.position.z) < self.voice_detected_increment:
                self.cmd_pose.position.z = self.current_pose.position.z + 0.1 # Assign cmd_pose.position


        if 'right' in self.detected_word:
            if 'one' in self.detected_word:
                self.position.x += 0.1
                print("Idem desno za 10cm")
            elif 'two' in self.detected_word:
                self.position.x += 0.2
                print("Idem desno za 20cm")
            elif 'three' in self.detected_word:
                self.position.x += 0.3
                print("Idem desno za 30cm")
            elif 'ten' in self.detected_word:
                self.position.x += 1
                print("Idem desno za 100cm")

        if 'left' in self.detected_word:
            if 'one' in self.detected_word:
                self.position.x -= 0.1
                print("Idem lijevo za 10cm")
            elif 'two' in self.detected_word:
                self.position.x -= 0.2
                print("Idem lijevo za 20cm")
            elif 'three' in self.detected_word:
                self.position.x -= 0.3
                print("Idem lijevo za 30cm")
            elif 'ten' in self.detected_word:
                self.position.x -= 1
                print("Idem lijevo za 100cm")

        if 'forward' in self.detected_word:
            if 'one' in self.detected_word:
                self.position.y += 0.1
                print("Idem naprijed za 10cm")
            elif 'two' in self.detected_word:
                self.position.y += 0.2
                print("Idem naprijed za 20cm")
            elif 'three' in self.detected_word:
                self.position.y += 0.3
                print("Idem naprijed za 30cm")
            elif 'ten' in self.detected_word:
                self.position.y += 1
                print("Idem naprijed za 100cm")

        if 'backward' in self.detected_word:
            if 'one' in self.detected_word:
                self.position.y -= 0.1
                print("Idem naprijed za 10cm")
            elif 'two' in self.detected_word:
                self.position.y -= 0.2
                print("Idem naprijed za 20cm")
            elif 'three' in self.detected_word:
                self.position.y -= 0.3
                print("Idem naprijed za 30cm")
            elif 'ten' in self.detected_word:
                self.position.y -= 1
                print("Idem naprijed za 100cm")
        
        ########### 
        # TO HERE

        """
        if self.detected_word == 'stop':
            self.position.x = 0
            self.position.y = 0
            self.position.z = 0
            self.orientation.x = 0
            self.orientation.y = 0
            self.orientation.z = 0
            self.orientation.w = 0
            print('uav_control:stop')
        """

    def run(self):
        while not rospy.is_shutdown():

            # TODO: Change variables for sending cmd
            # ONE VARIABLE FOR ODOMETRY READING AND CURRENT POSITION/ORIENTATION READING 
            # ONE VARIABLE FOR CURRENT POSE COMMAND, DO NOT USE SAME VARIABLES FOR CURRENT POSE READING AND CURRENT COMMAND!!!!

            self.pub_command.publish(self.position, self.orientation)
            #if self.pub_command.publish(self.position, self.orientation) != 'None':
                #print(self.pub_command.publish(self.position, self.orientation))

if __name__ == '__main__':
    rospy.init_node('uav_control')
    uc = UAV_Control()
    uc.run()

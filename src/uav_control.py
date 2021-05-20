#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from uav_voice_control.srv import ProvideMove, ProvideMoveResponse


class UAV_Control():

    def __init__(self):
        self.pub_command = rospy.Publisher('/command', Pose, queue_size=10)
        self.detected_word = String()
        self.position = Point()
        self.orientation = Quaternion()
        # self.odom = Odometry()
        self.rate = rospy.Rate(10)
        self.cmd_pose = Pose()
        self.wanted_pose = Pose()
        self.current_pose_odom = Odometry()
        self.var = ''
        rospy.Subscriber('/voice_recognition', String, self.command_callback)
        rospy.Subscriber('/uav/odometry', Odometry, self.odometry_callback)

        try:
            self.s1 = rospy.Service('provide_move', ProvideMove, self.provide_move_callback)
        except rospy.ServiceException as exc:
            print("Service already registred, but I am still working!\n")

    def provide_move_callback(self, rec):
        """For service and communication with smatch"""
        self.r_word = rec.word
        self.resp = ProvideMoveResponse()

        self.resp.response = True

        return self.resp


    def odometry_callback(self, msg):
        """Callback for reading current position via odometry"""
        self.current_pose_odom = msg.pose.pose
        # print(self.current_pose)

    def command_callback(self, msg):
        self.detected_word = str(msg.data)
        print(self.detected_word)

        self.voice_detected_increment = 0

        if 'one' in self.detected_word:
            self.voice_detected_increment = 0.1
        elif 'two' in self.detected_word:
            self.voice_detected_increment = 0.2
        elif 'three' in self.detected_word:
            self.voice_detected_increment = 0.3
        elif 'ten' in self.detected_word:
            self.voice_detected_increment = 1.0
        print(self.voice_detected_increment)


        ## TODO: LOGIC FOR Sending commands put into run method! Not in callback, we could use while to block command execution and 
        ## Wait for UAV to reach certain reference

        ### FROM HERE
        if 'takeoff' in self.detected_word:
            if round(self.current_pose_odom.position.z) == 0:
                self.cmd_pose.position.z = 0.5
                self.cmd_pose.orientation.w = 1
                time.sleep(3)
                self.cmd_pose.position.z = 1
                print('uav_control: take_off')
            else:
                print('uav: airborne')

        if 'stop' in self.detected_word:
            self.cmd_pose = self.current_pose_odom

        if 'land' in self.detected_word:
            self.var = 'land'

        if 'climb' in self.detected_word:
            epsilon_ = 0.01
            print('==============================')
            print('Climbing for ' + str(self.voice_detected_increment))
            #self.wanted_pose.position.z = self.current_pose_odom.position.z + self.voice_detected_increment

            # Nije tako trivijalno jer treba uzeti u obzir zadanu referencu (self.voice_detected_increment)
            while not abs(self.current_pose_odom.position.z - self.cmd_pose.position.z) < 0.05: #self.voice_detected_increment:
                self.cmd_pose.position.z = self.current_pose_odom.position.z + 0.1  # Assign cmd_pose.position
            print('Climbed for ' + str(self.voice_detected_increment))
            print('Current altitude is ' + str(self.current_pose_odom.position.z))

        if 'down' in self.detected_word:
            epsilon_ = 0.01
            self.wanted_pose.position.z = self.current_pose_odom.position.z - self.voice_detected_increment
            while not abs(self.current_pose_odom.position.z - self.wanted_pose.position.z) > self.voice_detected_increment:
                self.cmd_pose.position.z = self.current_pose_odom.position.z - 0.1

        if 'right' in self.detected_word:
            epsilon_ = 0.01
            self.wanted_pose.position.x = self.current_pose_odom.position.x + self.voice_detected_increment
            while not abs(self.current_pose_odom.position.x - self.wanted_pose.position.x) > self.voice_detected_increment:
                self.cmd_pose.position.x = self.current_pose_odom.position.x + 0.1

        if 'left' in self.detected_word:
            epsilon_ = 0.01
            self.wanted_pose.position.x = self.current_pose_odom.position.x - self.voice_detected_increment
            while not abs(self.current_pose_odom.position.x - self.wanted_pose.position.x) > self.voice_detected_increment:
                self.cmd_pose.position.x = self.current_pose_odom.position.x - 0.1

        if 'forward' in self.detected_word:
            epsilon_ = 0.01
            self.wanted_pose.position.y = self.current_pose_odom.position.y + self.voice_detected_increment
            while not abs(self.current_pose_odom.position.y - self.wanted_pose.position.y) > self.voice_detected_increment:
                self.cmd_pose.position.y = self.current_pose_odom.position.y + 0.1

        if 'backward' in self.detected_word:
            epsilon_ = 0.01
            self.wanted_pose.position.y = self.current_pose_odom.position.y - self.voice_detected_increment
            while not abs(self.current_pose_odom.position.y - self.wanted_pose.position.y) > self.voice_detected_increment:
                self.cmd_pose.position.y = self.current_pose_odom.position.y - 0.1

        """while not self.current_pose_odom.position.z < 0.1:
            # TODO: Create cmd_pose variable and initailze it as empty Pose message in constructor, use it for sending commands
            # self.cmd_pose.position.z = self.current_pose.position.z - 0.1
            # My idea was to use knowledge of current pose(position/orientation) and to decrease height value until we hit ground.

            self.cmd_pose.position.z = self.current_pose_odom.position.z - 0.1
            # INVOKE publish in while loop / CAN BE IN send_command method
            # self.publish....
            # USE WHILE TO"""


        """if 'home' in self.detected_word:
            self.wanted_pose.position.x = 0
            self.wanted_pose.position.y = 0
            while not abs(self.current_pose_odom.position.x - 0.1) == self.wanted_pose.position.x:
                if self.current_pose_odom.position.x < 0:
                    self.cmd_pose.position.x = self.current_pose_odom.position.x + 0.1
                elif self.current_pose_odom.position.x > 0:
                    self.cmd_pose.position.x = self.current_pose_odom.position.x - 0.1
                while not abs(self.current_pose_odom.position.y - 0.1) == self.wanted_pose.position.y:
                    if self.current_pose_odom.position.y < 0:
                        self.cmd_pose.position.y = self.current_pose_odom.position.y + 0.1
                    elif self.current_pose_odom.position.y > 0:
                        self.cmd_pose.position.y = self.current_pose_odom.position.y - 0.1"""

        """
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
                print("Idem desno za 100cm")"""

        ###########
        # TO HERE

    #def send_command(self, cmd):



    def run(self):
        while not rospy.is_shutdown():

            # TODO: Change variables for sending cmd
            # ONE VARIABLE FOR ODOMETRY READING AND CURRENT POSITION/ORIENTATION READING 
            # ONE VARIABLE FOR CURRENT POSE COMMAND, DO NOT USE SAME VARIABLES FOR CURRENT POSE READING AND CURRENT COMMAND!!!!

            if self.var == 'land':
                while not self.current_pose_odom.position.z < 0.1:
                    # TODO: Create cmd_pose variable and initailze it as empty Pose message in constructor, use it for sending commands
                    # self.cmd_pose.position.z = self.current_pose.position.z - 0.1
                    # My idea was to use knowledge of current pose(position/orientation) and to decrease height value until we hit ground.

                    self.cmd_pose.position.z = self.current_pose_odom.position.z - 0.1
                    # INVOKE publish in while loop / CAN BE IN send_command method
                    # self.publish....
                    # USE WHILE TO
                    self.pub_command.publish(self.cmd_pose.position, self.cmd_pose.orientation)
                self.var = ''

            """if self.var == 'climb':
                epsilon_ = 0.01
                self.wanted_pose.position.z = self.current_pose_odom.position.z + self.voice_detected_increment
                # Nije tako trivijalno jer treba uzeti u obzir zadanu referencu (self.voice_detected_increment)
                while not abs(self.current_pose_odom.position.z - self.wanted_pose.position.z) >= self.voice_detected_increment:
                    self.cmd_pose.position.z = self.current_pose_odom.position.z + 0.1  # Assign cmd_pose.position
                    self.pub_command.publish(self.cmd_pose.position, self.cmd_pose.orientation)
                self.var = ''"""

            self.pub_command.publish(self.cmd_pose.position, self.cmd_pose.orientation)

if __name__ == '__main__':
    rospy.init_node('uav_control')
    uc = UAV_Control()
    uc.run()

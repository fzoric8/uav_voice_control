#!/usr/bin/env python3

import rospy
import time
import yaml
import math
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


class UAV_Control():

    def __init__(self):

        with open("/home/filip/catkin_ws/src/uav_voice_control/config/config.yml", "r") as ymlfile:
            self.cfg = yaml.load(ymlfile)

        self.detected_word = String()
        self.position = Point()
        self.orientation = Quaternion()

        self.rate = rospy.Rate(10)
        self.cmd_pose = Pose()
        self.wanted_pose = Pose()
        self.last_position = Pose()
        self.current_pose_odom = Odometry()
        self.mode = 'charlie'
        self.cmd_increment = self.cfg["mode"]["charlie"]
        # self.voice_detected_increment = 0
        # self.current_yaw_angle = []
        self.yaw_angle = math.radians(self.cfg["yaw_angle"]["charlie"])
        self.yaw_angle_E = [0.0, 0.0, 0.0]
        self.yaw_angle_Q = [0.0, 0.0, 0.0, 0.0]
        self.var = ''
        self.detected_word = ''

        # Subscribers
        rospy.Subscriber('/voice_recognition', String, self.received_stop_callback)
        rospy.Subscriber('/voice_recognition', String, self.command_callback)
        rospy.Subscriber('/uav/odometry', Odometry, self.odometry_callback)

        # Publishers
        self.pub_command = rospy.Publisher('/uav/pose_ref', Pose, queue_size=10)

        print(f"=====================\nCurrent mode: {self.mode}\n  Reference: {self.cmd_increment}\n  "
              f"Yaw angle reference: {self.yaw_angle}\nStart with begin\n---")

    def odometry_callback(self, msg):
        """Callback for reading current position via odometry"""
        self.current_pose_odom = msg.pose.pose


    def command_callback(self, msg):
        self.detected_word = str(msg.data)
        print(f"Detected word is {self.detected_word}")

        if 'charlie' in self.detected_word:
            self.mode = 'charlie'
            self.cmd_increment = self.cfg["mode"]["charlie"]
            self.yaw_angle = math.radians(self.cfg["yaw_angle"]["charlie"])
            print(f"=====================\nSwitch mode:\n  Current mode: {self.mode}\n  Reference: {self.cmd_increment}"
                  f"\n  Yaw angle reference: {math.degrees(self.yaw_angle)}\n  "
                  f"Only in this mode is possible begin & finish\n---")

        elif 'oscar' in self.detected_word:
            self.mode = 'oscar'
            self.cmd_increment = self.cfg["mode"]["oscar"]
            self.yaw_angle = math.radians(self.cfg["yaw_angle"]["oscar"])
            print(f"=====================\nSwitch mode:\n  Current mode: {self.mode}\n  Reference: {self.cmd_increment}"
                  f"\n  Yaw angle reference: {math.degrees(self.yaw_angle)}\n---")

        elif 'romeo' in self.detected_word:
            self.mode = 'romeo'
            self.cmd_increment = self.cfg["mode"]["romeo"]
            self.yaw_angle = math.radians(self.cfg["yaw_angle"]["romeo"])
            print(f"=====================\nSwitch mode:\n  Current mode: {self.mode}\n  Reference: {self.cmd_increment}"
                  f"\n  Yaw angle reference: {math.degrees(self.yaw_angle)}\n---")

        elif 'sierra' in self.detected_word:
            self.mode = 'sierra'
            self.cmd_increment = self.cfg["mode"]["sierra"]
            self.yaw_angle = math.radians(self.cfg["yaw_angle"]["sierra"])
            print(f"=====================\nSwitch mode:\n  Current mode: {self.mode}\n  Reference: {self.cmd_increment}"
                  f"\n  Yaw angle reference: {math.degrees(self.yaw_angle)}\n---")

        """ Mode: Charlie
                - emergency mode
                - reference: 1 meter
                - yaw: -45deg
                - takeoff & finish only possible in this mode 
            
            Mode: Oscar
                - fine control mode
                - reference: 0.1 meter
                - yaw: 30deg 

            Mode: Romeo
                - reference: 0.3 meter
                - yaw: -30deg 

            Mode: Sierra
                - reference: 0.5 meter
                - yaw: 45deg """

        """ Begin & finish """
        if self.mode == 'charlie':
            if 'begin' in self.detected_word:
                if round(self.current_pose_odom.position.z) == 0:
                    self.cmd_pose.position.z = 0.5
                    #self.cmd_pose.orientation.w = 1
                    time.sleep(3)
                    self.cmd_pose.position.z = 1
                    print('uav_control: take_off\n---')
                else:
                    print('uav: airborne\n---')

            if 'finish' in self.detected_word:
                self.var = 'finish'

        self.last_position = self.current_pose_odom

        if self.mode == 'charlie' or self.mode == 'oscar' or self.mode == 'romeo' or self.mode == 'sierra':
            if 'climb' in self.detected_word:
                self.wanted_pose.position.z = (self.current_pose_odom.position.z + self.cmd_increment)
                while not round(self.current_pose_odom.position.z, 1) == round(self.wanted_pose.position.z, 1):
                    if self.mode == 'charlie' or self.mode == 'sierra':
                        self.cmd_pose.position.z = self.current_pose_odom.position.z + 0.14
                    elif self.mode == 'romeo' or self.mode == 'oscar':
                        self.cmd_pose.position.z = self.current_pose_odom.position.z + 0.07

            if 'down' in self.detected_word:
                self.wanted_pose.position.z = self.current_pose_odom.position.z - self.cmd_increment
                while not round(self.current_pose_odom.position.z, 1) == round(self.wanted_pose.position.z, 1) or \
                        round(self.current_pose_odom.position.z) == 0:
                    print(round(self.current_pose_odom.position.z))
                    if self.mode == 'charlie' or self.mode == 'sierra':
                        self.cmd_pose.position.z = self.current_pose_odom.position.z - 0.14
                    elif self.mode == 'romeo' or self.mode == 'oscar':
                        self.cmd_pose.position.z = self.current_pose_odom.position.z - 0.07

            if 'right' in self.detected_word:
                self.wanted_pose.position.x = self.current_pose_odom.position.x + self.cmd_increment
                while not round(self.current_pose_odom.position.x, 1) == round(self.wanted_pose.position.x, 1):
                    if self.mode == 'charlie' or self.mode == 'sierra':
                        self.cmd_pose.position.x = self.current_pose_odom.position.x + 0.14
                    elif self.mode == 'romeo' or self.mode == 'oscar':
                        self.cmd_pose.position.x = self.current_pose_odom.position.x + 0.07

            if 'left' in self.detected_word:
                self.wanted_pose.position.x = self.current_pose_odom.position.x - self.cmd_increment
                while not round(self.current_pose_odom.position.x, 1) == round(self.wanted_pose.position.x, 1):
                    if self.mode == 'charlie' or self.mode == 'sierra':
                        self.cmd_pose.position.x = self.current_pose_odom.position.x - 0.14
                    elif self.mode == 'romeo' or self.mode == 'oscar':
                        self.cmd_pose.position.x = self.current_pose_odom.position.x - 0.07

            if 'forward' in self.detected_word:
                self.wanted_pose.position.y = self.current_pose_odom.position.y + self.cmd_increment
                while not round(self.current_pose_odom.position.y, 1) == round(self.wanted_pose.position.y, 1):
                    if self.mode == 'charlie' or self.mode == 'sierra':
                        self.cmd_pose.position.y = self.current_pose_odom.position.y + 0.14
                    elif self.mode == 'romeo' or self.mode == 'oscar':
                        self.cmd_pose.position.y = self.current_pose_odom.position.y + 0.07

            if 'backward' in self.detected_word:
                self.wanted_pose.position.y = self.current_pose_odom.position.y - self.cmd_increment
                while not round(self.current_pose_odom.position.y, 1) == round(self.wanted_pose.position.y, 1):
                    if self.mode == 'charlie' or self.mode == 'sierra':
                        self.cmd_pose.position.y = self.current_pose_odom.position.y - 0.14
                    elif self.mode == 'romeo' or self.mode == 'oscar':
                        self.cmd_pose.position.y = self.current_pose_odom.position.y - 0.07

            if 'climb' in self.detected_word or 'down' in self.detected_word or 'right' in self.detected_word or \
                    'left' in self.detected_word or 'forward' in self.detected_word or 'backward' in self.detected_word:
                print(f"Moving for {self.cmd_increment} in direction {self.detected_word}.")
                print(f"Current position:\n{self.current_pose_odom.position}\n---")

            if 'spin' in self.detected_word:
                #self.current_yaw_angle = self.quat_to_eul(self.current_pose_odom.orientation)
                self.yaw_angle_E[2] = self.yaw_angle_E[2] + self.yaw_angle
                self.yaw_angle_Q = self.eul_to_quat(self.yaw_angle_E)
                self.cmd_pose.orientation.x = self.yaw_angle_Q[0]
                self.cmd_pose.orientation.y = self.yaw_angle_Q[1]
                self.cmd_pose.orientation.z = self.yaw_angle_Q[2]
                self.cmd_pose.orientation.w = self.yaw_angle_Q[3]
                if self.yaw_angle > 0:
                    print(f"Rotate CCW for {math.degrees(self.yaw_angle)} degrees.\n---")
                else:
                    print(f"Rotate CW for {abs(math.degrees(self.yaw_angle))} degrees.\n---")


    def eul_to_quat(self, num):
        """ Function for transforming Euler angles to Quaternion"""
        x = 0; y = 0; z = num[2]

        qx = math.sin(x/2) * math.cos(y/2) * math.cos(z/2) - math.cos(x/2) * math.sin(y/2) * math.sin(y/2)
        qy = math.cos(x/2) * math.sin(y/2) * math.cos(z/2) + math.sin(x/2) * math.cos(y/2) * math.sin(z/2)
        qz = math.cos(x/2) * math.cos(y/2) * math.sin(z/2) - math.sin(x/2) * math.sin(y/2) * math.cos(z/2)
        qw = math.cos(x/2) * math.cos(y/2) * math.cos(z/2) + math.sin(x/2) * math.sin(y/2) * math.sin(z/2)

        return [qx, qy, qz, qw]

    def quat_to_eul(self, orientation):
        """ Function for transforming Quaternion to Euler angles"""
        x = orientation.x; y = orientation.y; z = orientation.z; w = orientation.w

        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        eul_x = math.atan2(t0, t1)

        t2 = 2.0 * (w * y - z * x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        eul_y = math.asin(t2)

        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        eul_z = math.atan2(t3, t4)

        return [eul_x, eul_y, eul_z]

    def run(self):
        while not rospy.is_shutdown():

            if self.var == 'finish':
                while not self.current_pose_odom.position.z < 0.1:
                    self.cmd_pose.position.z = self.current_pose_odom.position.z - 0.1
                    self.pub_command.publish(self.cmd_pose.position, self.cmd_pose.orientation)
                self.var = ''

            self.pub_command.publish(self.cmd_pose.position, self.cmd_pose.orientation)


if __name__ == '__main__':

    try:
        rospy.init_node('uav_control')
        uc = UAV_Control()
        uc.run()

    except Exception as e:
        print(e)

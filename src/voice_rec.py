#!/usr/bin/env python3

import rospy
from pocketsphinx import LiveSpeech
from std_msgs.msg import String


def voice_recog(voice):
    voice_rec = String()
    voice_rec.data = str(voice)
    print(voice_rec)
    pub.publish(voice_rec.data)


if __name__ == '__main__':
    pub = rospy.Publisher('voice_recognition', String, queue_size=10)
    rospy.init_node('voice_publish')
    speech = LiveSpeech(lm=False,
                        kws='/home/filip/catkin_ws/src/uav_voice_control/src/key.list', verbose=False,
                        no_search=False, full_utt=False, buffer_size=2048, sampling_rate=16000)

    while not rospy.is_shutdown():
        for phrase in speech:
            voice_recog(phrase)
            rospy.sleep(1.0)
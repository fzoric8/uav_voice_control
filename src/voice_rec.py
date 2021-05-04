#!/usr/bin/env python

import rospy
import pyaudio 
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
    
    # Check which microphone to use 
    p = pyaudio.PyAudio()
    print("===================================================")
    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        print("Device index {} name is: {}".format(i, info["name"]))
        print("Device info by index: ", info)
    
    # Instantiate LiveSpeech class 
    speech = LiveSpeech(lm=False,
			# Set relative path here for key.list -> should work on any PC
                        kws='/home/developer/catkin_ws/src/uav_voice_control/src/key.list', verbose=False,
                        no_search=False, full_utt=False, buffer_size=2048, sampling_rate=16000, audio_device="dmix")
			# If you don't pass any argument while creating an instance of Pockesphinx, AudioFile or 
                        # LiveSpeech class, it will use next default values: https://github.com/bambocher/pocketsphinx-python#default-config

    while not rospy.is_shutdown():
        for phrase in speech:
            voice_recog(phrase)
            rospy.sleep(1.0)

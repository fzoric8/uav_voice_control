#!/usr/bin/env python3

import rospy
import pyaudio
from os import environ, path
from pocketsphinx import LiveSpeech
from sphinxbase import *
from std_msgs.msg import String


def voice_recog(voice):
    #if not rospy.is_shutdown():
    voice_rec = String()
    voice_rec.data = str(voice)
    print("checkpoint")
    print(voice_rec)
    print(voice_rec.data)
    pub.publish(voice_rec.data)
    #rospy.sleep(1.0)
		#rospy.spin()


if __name__ == '__main__':
    pub = rospy.Publisher('voice_recognition', String, queue_size=10)
    rospy.init_node('voice_publish')
    #speech = LiveSpeech(lm=False, keyphrase='up', kws_threshold=1e-20)
    #speech1 = LiveSpeech(lm=False, keyphrase='up', kws_threshold=1e-10)

#for phrase in speech:
    while not rospy.is_shutdown():
        for phrase in LiveSpeech():
            v = phrase
            print(type(phrase))
            print(dir(phrase))
        #v = phrase.segments(detailed=True)
            print(v)
            voice_recog(v)
            rospy.sleep(1.0)



        #try:
            #while not rospy.is_shutdown():
                #voice_recog(v)
        #except rospy.ROSInterruptException:
            #print("palo")

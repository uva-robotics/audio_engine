#! /usr/bin/env python
import rospy
import numpy as np
from naoqi_bridge_msgs.msg import AudioBuffer

from util import MonoConvert, StereoConvert, LocalizeAudio

NAOQI_AUDIO_TOPIC = '/pepper_robot/audio'

class AudioEngine():
    def __init__(self):
        self.channels = 4
        self.sample_rate = 48000
        self.mono_convert = MonoConvert(self.sample_rate, self.channels)
        self.stereo_convert = StereoConvert(self.channels)
        self.localize = LocalizeAudio(self.channels)
        self.sub = rospy.Subscriber(NAOQI_AUDIO_TOPIC, AudioBuffer, self.audio_cb)

    def audio_cb(self, data):
        data = data.data
        self.localize.process(data)
        self.mono_convert.convert(data)
        self.stereo_convert.convert(data)
        
    
if __name__=="__main__":
    rospy.init_node('audio_engine')
    
    engine = AudioEngine()
    rospy.spin()

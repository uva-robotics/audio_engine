#! /usr/bin/env python

import rospy
import numpy as np
from naoqi_bridge_msgs.msg import AudioBuffer

import pyaudio
from std_msgs.msg import Float32MultiArray, String
import scipy.io.wavfile as wav

from util import MonoConvert, StereoConvert, LocalizeAudio


AUDIO_TOPIC = '/converted_audio'
TEST_AUDIO_TOPIC = '/audio_test'

class AudioTest():
    def __init__(self):
        rospy.init_node('audio_test')
        self.sample_rate = 48000
        self.channels = 2
        self.CHUNKSIZE = 512

        self.mono_convert = MonoConvert(self.sample_rate, self.channels)
        self.stereo_convert = StereoConvert(self.channels)
        self.localize = LocalizeAudio(self.channels)

        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=pyaudio.paInt16, 
            channels=self.channels, 
            rate=self.sample_rate, 
            input=True, 
            frames_per_buffer=self.CHUNKSIZE)

        # write np_data to publisher with CHUNKSIZE.
        while not rospy.is_shutdown():
            frames = []
            for _ in range(0, int(self.sample_rate / self.CHUNKSIZE)):
                data = self.stream.read(self.CHUNKSIZE)
                # self.send_test.publish(data)
                frames.append(np.fromstring(data, dtype=np.int16))
            
            # self.localize.process(frames)
            self.mono_convert.convert(frames)
            self.stereo_convert.convert(frames)
            # m = Float32MultiA1rray()
            # m.data = np.hstack(frames)
            # self.audio_topic.publish(m)

        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()

        
        # wav.write('./test.wav', self.desired_sample_rate, np_data)

if __name__ == '__main__':
    test = AudioTest()
    rospy.spin()
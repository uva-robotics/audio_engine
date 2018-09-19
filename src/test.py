#! /usr/bin/env python

import rospy
import numpy as np
from naoqi_bridge_msgs.msg import AudioBuffer

import pyaudio
from std_msgs.msg import Float32MultiArray, String
import scipy.io.wavfile as wav

AUDIO_TOPIC = '/converted_audio'
TEST_AUDIO_TOPIC = '/audio_test'

class AudioTest():
    def __init__(self):
        rospy.init_node('audio_test')
        self.desired_sample_rate = 16000
        self.desired_channels = 1
        self.CHUNKSIZE = 2048
        self.audio_topic = rospy.Publisher(AUDIO_TOPIC, Float32MultiArray, queue_size=10)
        
        self.send_test = rospy.Publisher(TEST_AUDIO_TOPIC, String, queue_size=10)

        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=pyaudio.paInt16, 
            channels=self.desired_channels, 
            rate=self.desired_sample_rate, 
            input=True, 
            frames_per_buffer=self.CHUNKSIZE)

        # write np_data to publisher with CHUNKSIZE.
        while not rospy.is_shutdown():
            frames = []
            for _ in range(0, int(self.desired_sample_rate / self.CHUNKSIZE)):
                data = self.stream.read(self.CHUNKSIZE)
                self.send_test.publish(data)
                frames.append(np.fromstring(data, dtype=np.int16))
            m = Float32MultiArray()
            m.data = np.hstack(frames)
            self.audio_topic.publish(m)

        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()

        
        # wav.write('./test.wav', self.desired_sample_rate, np_data)

if __name__ == '__main__':
    test = AudioTest()
    rospy.spin()
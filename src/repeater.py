#!/usr/bin/python
import rospy
import pyaudio
import numpy as np

import sounddevice as sd

from std_msgs.msg import Float32MultiArray

AUDIO_TOPIC = '/converted_audio'

class AudioRepeater():

    def __init__(self):
        rospy.init_node('audio_engine_repeater')

        self.desired_sample_rate = 16000
        self.desired_channels = 1
        self.CHUNKSIZE = 1024

        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=pyaudio.paInt16, 
            channels=self.desired_channels, 
            rate=self.desired_sample_rate, 
            output=True,
            output_device_index=7,
            frames_per_buffer=self.CHUNKSIZE)

        rospy.Subscriber(AUDIO_TOPIC, Float32MultiArray, self.audio_cb)



    def audio_cb(self, data):
        audiodata = np.asarray(data.data)
        data = audiodata.astype(np.int16).tostring()
        self.stream.write(data)

if __name__ == '__main__':
    print(sd.query_devices())
    repeater = AudioRepeater()
    rospy.spin()
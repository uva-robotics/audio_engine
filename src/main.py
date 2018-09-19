#! /usr/bin/env python

import rospy
import numpy as np
from naoqi_bridge_msgs.msg import AudioBuffer

from std_msgs.msg import Float32MultiArray
from scipy.signal import butter, lfilter


NAOQI_AUDIO_TOPIC = '/pepper_robot/audio'
CONVERTED_AUDIO_TOPIC = '/converted_audio'


class AudioEngine():
    def __init__(self):
        self.desired_sample_rate = 16000
        self.desired_channels = 1
        self.convert = rospy.Publisher(CONVERTED_AUDIO_TOPIC, Float32MultiArray, queue_size=10)
        self.buffer = []
        self.buffer_size = 512

        self.sub = rospy.Subscriber(NAOQI_AUDIO_TOPIC, AudioBuffer, self.audio_cb)


    def downsample(self, audio, n):
        """Downsample audio. Keep only every n'th element"""
        return audio[::n]

    def butter_bandpass(self, lowcut, highcut, fs, order=5):
        nyq = 0.5 * fs
        low = lowcut / nyq
        high = highcut / nyq
        b, a = butter(order, [low, high], btype='band')
        return b, a


    def butter_bandpass_filter(self, data, lowcut, highcut, fs, order=5):
        b, a = self.butter_bandpass(lowcut, highcut, fs, order=order)
        y = lfilter(b, a, data)
        return y
        
    def audio_cb(self, data):
        frequency = data.frequency
        total_channels = len(data.channelMap)
        
        audio = list(data.data)

        # 4-channel to mono
        audiodata = np.asarray(audio)
        reshape = audiodata.reshape(-1, total_channels/self.desired_channels)
        audiodata = np.mean(reshape, axis=1)

        # Downsample the audio
        audiodata = self.downsample(audiodata, frequency / self.desired_sample_rate)

        # Voice frequency between 300 and 3500
        audiodata = self.butter_bandpass_filter(audiodata, 300, 3400, self.desired_sample_rate)

        self.buffer.extend(audiodata)

        ms_in_buffer = len(self.buffer)/float(self.desired_sample_rate) * 1000
        print(ms_in_buffer)

        if ms_in_buffer >= self.buffer_size:
            m = Float32MultiArray()
            m.data = self.buffer
            self.convert.publish(m)
            self.buffer = []
    
if __name__=="__main__":
    rospy.init_node('audio_engine')
    
    engine = AudioEngine()
    rospy.spin()

import rospy
import numpy as np
import collections

from scipy.signal import butter, lfilter

from std_msgs.msg import Float32MultiArray


CONVERTED_AUDIO_TOPIC = '/converted_audio'
AUDIO_LOCALIZATION_TOPIC = '/audio_localization'
STEREO_AUDIO_TOPIC = '/stereo_audio'


def calc_rms(audio):
    return np.sqrt(np.mean(audio**2))

class LocalizeAudio():

    def __init__(self, channels):
        self.channels = channels
        self.localization = rospy.Publisher(AUDIO_LOCALIZATION_TOPIC, Float32MultiArray, queue_size=10)

    def process(self, data):

        # uint8 CHANNEL_FRONT_LEFT=0
        # uint8 CHANNEL_FRONT_RIGHT=2
        # uint8 CHANNEL_REAR_LEFT=3
        # uint8 CHANNEL_REAR_RIGHT=5
        samples_per_channel = int(len(data) / self.channels)

        channel_data = np.reshape(data, (self.channels, samples_per_channel), 'F')

        rms_data = [calc_rms(i) for i in channel_data]

        m = Float32MultiArray()
        m.data = rms_data
        self.localization.publish(m)

class StereoConvert():
    def __init__(self, channels):

        self.channels = channels
        self.desired_sample_rate = 48000
        self.desired_channels = 2
        self.stereo_audio = rospy.Publisher(STEREO_AUDIO_TOPIC, Float32MultiArray, queue_size=10)
        self.buffer = []
        self.buffer_size = 512 # ms

        self.CHUNK_SIZE = int(self.desired_sample_rate * self.buffer_size / 1000.)
        self.ring_buffer = collections.deque(maxlen=self.CHUNK_SIZE)

    def convert(self, data):        
        # 4-channel to stereo
        audiodata = np.asarray(data)
        reshape = audiodata.reshape(-1, self.channels/self.desired_channels)
        audiodata = np.mean(reshape, axis=1)

        # self.buffer.extend(audiodata)
        self.ring_buffer.extend(audiodata)
        ms_in_buffer = len(self.ring_buffer)/float(self.desired_sample_rate) * 1000

        if ms_in_buffer == self.buffer_size:
            m = Float32MultiArray()
            m.data = self.ring_buffer
            self.stereo_audio.publish(m)
            self.buffer = []
            self.ring_buffer.clear()

class MonoConvert():

    def __init__(self, sample_rate, channels):

        self.sample_rate = sample_rate
        self.channels = channels
        
        self.desired_sample_rate = 16000
        self.desired_channels = 1
        self.mono_audio = rospy.Publisher(CONVERTED_AUDIO_TOPIC, Float32MultiArray, queue_size=10)
        self.buffer = []
        self.buffer_size = 512 # ms

        self.CHUNK_SIZE = int(self.desired_sample_rate * self.buffer_size / 1000.)
        self.ring_buffer = collections.deque(maxlen=self.CHUNK_SIZE)


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


    def convert(self, data):
        
        # 4-channel to mono
        audiodata = np.asarray(data)
        reshape = audiodata.reshape(-1, self.channels/self.desired_channels)
        audiodata = np.mean(reshape, axis=1)

        # Downsample the audio
        audiodata = self.downsample(audiodata, self.sample_rate / self.desired_sample_rate)

        # Voice frequency between 300 and 3500
        audiodata = self.butter_bandpass_filter(audiodata, 250, 4000, self.desired_sample_rate)

        # self.buffer.extend(audiodata)
        self.ring_buffer.extend(audiodata)
        ms_in_buffer = len(self.ring_buffer)/float(self.desired_sample_rate) * 1000

        if ms_in_buffer == self.buffer_size:
            m = Float32MultiArray()
            m.data = self.ring_buffer
            self.mono_audio.publish(m)
            self.ring_buffer.clear()


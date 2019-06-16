"""
Code related to audio processing.
"""
from .dll_lib import *
import math


class AudioFrame:
    """
    A frame of Audio
    """
    def __init__(self, beam_angle, beam_conf, samples):
        """
        Create a body from raw body/joint data.

        Note:
            Should not be called by user.
            Use `kinect.get_audio_frames()`.
        """
        self.beam_angle = beam_angle
        self.beam_conf = beam_conf
        self.data = samples

    def __repr__(self):
        degs = round(math.degrees(self.beam_angle), 1)
        return '<AudioFrame [{}°]>'.format(degs)
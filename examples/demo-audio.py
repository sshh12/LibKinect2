"""
Demonstrating basic usage with audio data.
"""
from libkinect2 import Kinect2
from libkinect2.utils import merge_audio_frames
import numpy as np
import time
import cv2

# Init kinect w/all visual sensors
kinect = Kinect2(use_sensors=['color', 'audio'])
kinect.connect()
kinect.wait_for_worker()

# Playback
## import pyaudio
## p = pyaudio.PyAudio()
## stream = p.open(format=pyaudio.paFloat32,
##                 channels=1,
##                 rate=16000,
##                 output=True)


# A super simple (less accurate) method to map beam angle to position.
# Using linear regression calibrated w/random test data.
def beam_angle_to_pos(angle):
    return int(angle * 1419.2 + 23.898 + 1920 / 2)


# Average angle of audio beams
rolling_avg_angle = 0

for _, color_img, audio_frames in kinect.iter_frames():

    if audio_frames:

        # Combine raw audio samples from frames
        full_frame = merge_audio_frames(audio_frames)

        # Update beam angle
        avg_angle = full_frame.beam_angle
        rolling_avg_angle = rolling_avg_angle * 0.8 + avg_angle * 0.2

        ## stream.write(full_frame.data)

    # Plot the audio direction
    sound_x = beam_angle_to_pos(rolling_avg_angle)
    cv2.putText(color_img, "Audio Beam", (sound_x - 90, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv2.LINE_AA)
    color_img[:, sound_x-5:sound_x+5, 0] += 100
    color_img[:, sound_x-5:sound_x+5, 1] -= 100
    color_img[:, sound_x-5:sound_x+5, 2] -= 100

    cv2.imshow('sensors', color_img)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

## stream.close()  
kinect.disconnect()
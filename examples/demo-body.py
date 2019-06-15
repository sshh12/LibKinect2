"""
Demonstrating basic usage with bodies.
"""
from libkinect2 import Kinect2
from libkinect2.utils import draw_skeleton, dist
import numpy as np
import cv2

# Init kinect w/all visual sensors
kinect = Kinect2(use_sensors=['color', 'body'])
kinect.connect()
kinect.wait_for_worker()


def draw_hand(color_img, hand, wrist):
    if hand.tracking == 'tracked' and wrist.tracking == 'tracked':
        size = dist(hand.color_pos, wrist.color_pos)
        if hand.state == 'closed':
            cv2.circle(color_img, hand.color_pos, size, (0, 0, 255), -1)
        else:
            cv2.circle(color_img, hand.color_pos, size, (0, 255, 0), 2)


def draw_face(color_img, face):
    if face is not None:
        for i in range(68):
            x, y = face.points[i]
            cv2.circle(color_img, (x, y), 2, (255, 255, 255), -1)


for _, color_img, bodies in kinect.iter_frames():

    for body in bodies:
        face = body.get_face(color_img)
        draw_skeleton(color_img, body)
        draw_hand(color_img, body['hand_left'], body['wrist_left'])
        draw_hand(color_img, body['hand_right'], body['wrist_right'])
        draw_face(color_img, face)
    
    cv2.imshow('sensors', color_img)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

kinect.disconnect()
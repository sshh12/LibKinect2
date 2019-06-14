"""
Helpful functions.
"""
import cv2


## Adapted from:
## https://github.com/Kinect/PyKinect2/blob/c86d575175edf5fc0834590acdfb82b55b5ccd96/examples/PyKinectBodyGame.py#L76
BODY_EDGES = [
    # Torso
    ('head', 'neck'),
    ('neck', 'spine_shoulder'),
    ('spine_shoulder', 'spine_mid'),
    ('spine_mid', 'spine_base'),
    ('spine_shoulder', 'shoulder_right'),
    ('spine_shoulder', 'shoulder_left'),
    ('spine_base', 'hip_right'),
    ('spine_base', 'hip_left'),
    # Right Arm
    ('shoulder_right', 'elbow_right'),
    ('elbow_right', 'wrist_right'),
    ('wrist_right', 'hand_right'),
    ('hand_right', 'hand_right_tip'),
    ('wrist_right', 'thumb_right'),
    # Left Arm
    ('shoulder_left', 'elbow_left'),
    ('elbow_left', 'wrist_left'),
    ('wrist_left', 'hand_left'),
    ('hand_left', 'hand_left_tip'),
    ('wrist_left', 'thumb_left'),
    # Right Leg
    ('hip_right', 'knee_right'),
    ('knee_right', 'ankle_right'),
    ('ankle_right', 'foot_right'),
    # Left Leg
    ('hip_left', 'knee_left'),
    ('knee_left', 'ankle_left'),
    ('ankle_left', 'foot_left')
]


def draw_skeleton(color_img, body, color=(0, 255, 0)):
    for part_a, part_b in BODY_EDGES:
        cv2.line(color_img, body[part_a].color_pos, body[part_b].color_pos, color, 2)
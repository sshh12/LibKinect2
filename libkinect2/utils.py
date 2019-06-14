"""
Helpful functions.
"""
import numpy as np
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
    """
    Draw skeleton onto `color_img` (an array of shape (height, width, colors))
    using joints from `body`.
    """
    for part_a, part_b in BODY_EDGES:
        cv2.line(color_img, body[part_a].color_pos, body[part_b].color_pos, color, 2)


def depth_map_to_image(depth_map):
    """
    Convert `depth_map` to a multicolor image
    for visualization.
    """
    h, w, _ = depth_map.shape
    img = np.empty((h, w, 3))
    normalized_map = (depth_map[:, :, 0] / 8000.0)
    img[:, :, 0] = normalized_map * 180
    img[:, :, 1] = 150 + normalized_map * 100
    img[:, :, 2] = 150 + normalized_map * 100
    return cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_HSV2BGR)

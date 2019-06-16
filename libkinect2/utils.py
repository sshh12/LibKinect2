"""
Helpful functions.
"""
from .audio import AudioFrame
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


def draw_skeleton(color_img, body, color=(0, 255, 0), allow_inferred=False):
    """
    Draw skeleton onto `color_img` (an array of shape (height, width, colors))
    using joints from `body`.
    """
    for part_a, part_b in BODY_EDGES:
        joint_a = body[part_a]
        joint_b = body[part_b]
        if allow_inferred or (joint_a.tracking == 'tracked' and joint_b.tracking == 'tracked'):
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


def ir_to_image(ir_image):
    """
    Convert `ir_image` to a multicolor image
    for visualization.
    """
    h, w, _ = ir_image.shape
    img = np.empty((h, w, 3))
    normalized_img = (ir_image[:, :, 0] / 65535.0)
    img[:, :, 0] = normalized_img * 255
    img[:, :, 1] = normalized_img * 255
    img[:, :, 2] = normalized_img * 255
    return img.astype(np.uint8)


def dist(pos_a, pos_b):
    """
    Distance between two points
    """
    return int(((pos_a[0] - pos_b[0])**2 + (pos_a[1] - pos_b[1])**2)**0.5)


def merge_audio_frames(audio_frames):
    """
    Merge `AudioFrame`s into a single `AudioFrame`
    by combining / averaging data.
    """
    data = np.concatenate([frame.data for frame in audio_frames])
    beam_angle = np.mean([frame.beam_angle for frame in audio_frames])
    beam_conf = np.mean([frame.beam_conf for frame in audio_frames])
    return AudioFrame(beam_angle, beam_conf, data)
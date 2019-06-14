"""
Code for interfacing with the compiled library.
"""
from pkg_resources import resource_filename
import numpy as np
import ctypes

### Constants ###
F_SENSOR_COLOR    = 0x00000001
F_SENSOR_DEPTH    = 0x00000010
F_SENSOR_IR       = 0x00000100
F_SENSOR_BODY     = 0x00001000
F_SENSOR_MULTI    = 0x00001111
F_SENSOR_AUDIO    = 0x00010000
F_MAP_COLOR_CAM   = 0x00000002
F_MAP_DEPTH_CAM   = 0x00000020
F_MAP_DEPTH_COLOR = 0x00000200
F_MAP_COLOR_DEPTH = 0x00002000
COLOR_WIDTH       = 1920
COLOR_HEIGHT      = 1080
COLOR_CHANNELS    = 4
DEPTH_WIDTH       = 512
DEPTH_HEIGHT      = 424
IR_WIDTH          = 512
IR_HEIGHT         = 424
MAX_SUBFRAMES     = 8
SUBFRAME_SIZE     = 256
AUDIO_BUF_LEN     = 512
SUBFRAME_SIZE     = 256
MAX_BODIES        = 6
BODY_PROPS        = 15
MAX_JOINTS        = 25
JOINT_PROPS       = 9
FLOAT_MULT        = 100000

### Enum Mappings ###
JOINT_MAP = {
    'spine_base':     0,
    'spine_mid':      1,
    'neck':           2,
    'head':           3,
    'shoulder_left':  4,
    'elbow_left':     5,
    'wrist_left':     6,
    'hand_left':      7,
    'shoulder_right': 8,
    'elbow_right':    9,
    'wrist_right':    10,
    'hand_right':     11,
    'hip_left':       12,
    'knee_left':      13,
    'ankle_left':     14,
    'foot_left':      15,
    'hip_right':      16,
    'knee_right':     17,
    'ankle_right':    18,
    'foot_right':     19,
    'spine_shoulder': 20,
    'hand_left_tip':  21,
    'thumb_left':     22,
    'hand_right_tip': 23,
    'thumb_right':    24,
    'eye_left':       -1,
    'eye_right':      -1,
    'mouth':          -1
}
TRACKING_MAP = [None, 'inferred', 'tracked']
HIGH_CONFIDENCE_MAP = [False, True]
HAND_MAP = ['unk', None, 'open', 'closed', 'lasso']
DETECTION_MAP = ['unk', None, 'maybe', 'yes']


def init_lib(dll_path=None):
    """
    Load the dll and add arg/return types.
    """
    if dll_path is None:
        dll_path = resource_filename(__name__, 'data/Kinect2-API.dll')

    kinectDLL = ctypes.cdll.LoadLibrary(dll_path)

    kinectDLL.init_kinect.argtypes = [ctypes.c_int, ctypes.c_int]
    kinectDLL.init_kinect.restype = ctypes.c_bool

    kinectDLL.close_kinect.argtypes = []
    kinectDLL.close_kinect.restype = None

    kinectDLL.pause_worker.argtypes = []
    kinectDLL.pause_worker.restype = None

    kinectDLL.resume_worker.argtypes = []
    kinectDLL.resume_worker.restype = None

    kinectDLL.get_tick.argtypes = []
    kinectDLL.get_tick.restype = ctypes.c_int32

    kinectDLL.get_color_data.argtypes = [np.ctypeslib.ndpointer(dtype=np.uint8)]
    kinectDLL.get_color_data.restype = ctypes.c_bool

    kinectDLL.get_ir_data.argtypes = [np.ctypeslib.ndpointer(dtype=np.uint16)]
    kinectDLL.get_ir_data.restype = ctypes.c_bool

    kinectDLL.get_depth_data.argtypes = [np.ctypeslib.ndpointer(dtype=np.uint16)]
    kinectDLL.get_depth_data.restype = ctypes.c_bool

    kinectDLL.get_body_data.argtypes = [np.ctypeslib.ndpointer(dtype=np.uint8), np.ctypeslib.ndpointer(dtype=np.int32)]
    kinectDLL.get_body_data.restype = ctypes.c_bool

    kinectDLL.get_audio_data.argtypes = [np.ctypeslib.ndpointer(dtype=np.float32), np.ctypeslib.ndpointer(dtype=np.float32)]
    kinectDLL.get_audio_data.restype = ctypes.c_int32

    kinectDLL.get_map_color_to_camera.argtypes = [np.ctypeslib.ndpointer(dtype=np.float32)]
    kinectDLL.get_map_color_to_camera.restype = ctypes.c_bool

    kinectDLL.get_map_depth_to_camera.argtypes = [np.ctypeslib.ndpointer(dtype=np.float32)]
    kinectDLL.get_map_depth_to_camera.restype = ctypes.c_bool

    kinectDLL.get_map_depth_to_color.argtypes = [np.ctypeslib.ndpointer(dtype=np.float32)]
    kinectDLL.get_map_depth_to_color.restype = ctypes.c_bool

    kinectDLL.get_map_color_depth.argtypes = [np.ctypeslib.ndpointer(dtype=np.float32)]
    kinectDLL.get_map_color_depth.restype = ctypes.c_bool

    return kinectDLL

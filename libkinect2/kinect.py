from pkg_resources import resource_filename
import numpy as np
import ctypes


F_SENSOR_COLOR  = 0x00000001
F_SENSOR_DEPTH  = 0x00000010
F_SENSOR_IR     = 0x00000100
F_SENSOR_BODY   = 0x00001000
F_SENSOR_MULTI  = 0x00001111
F_SENSOR_AUDIO  = 0x00010000
COLOR_WIDTH     = 1920
COLOR_HEIGHT    = 1080
COLOR_CHANNELS  = 4
DEPTH_WIDTH     = 512
DEPTH_HEIGHT    = 424
IR_WIDTH        = 512
IR_HEIGHT       = 424
MAX_SUBFRAMES   = 8
SUBFRAME_SIZE   = 256
AUDIO_BUF_LEN   = 512
SUBFRAME_SIZE   = 256
MAX_BODIES      = 6
BODY_PROPS      = 7
MAX_JOINTS      = 25
JOINT_PROPS     = 5


JOINT_MAP = {
    'spine_base':     0,
    'spine_mid':      1,
    'neck':           2,
    'head':           3,
    'shoulder_left':  3,
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
    'thumb_right':    24
}

TRACKING_MAP = [False, 'inferred', 'tracked']
HIGH_CONFIDENCE_MAP = [False, True]
HAND_MAP = ['unk', False, 'open', 'closed', 'lasso']



kinectDLL = ctypes.cdll.LoadLibrary(resource_filename(__name__, 'data/Kinect2-API.dll'))

kinectDLL.init_kinect.argtypes = [ctypes.c_int]
kinectDLL.init_kinect.restype = ctypes.c_bool

kinectDLL.close_kinect.argtypes = []
kinectDLL.close_kinect.restype = None

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


class Kinect2:

    def __init__(self, use_sensors=['color']):
        self.sensor_flags = 0
        if 'color' in use_sensors:
            self.sensor_flags |= F_SENSOR_COLOR
        if 'depth' in use_sensors:
            self.sensor_flags |= F_SENSOR_DEPTH
        if 'ir' in use_sensors:
            self.sensor_flags |= F_SENSOR_IR
        if 'body' in use_sensors:
            self.sensor_flags |= F_SENSOR_BODY
        if 'audio' in use_sensors:
            self.sensor_flags |= F_SENSOR_AUDIO

    def connect(self):
        if not kinectDLL.init_kinect(self.sensor_flags):
            raise IOError('Unable to init Kinect2 Sensor.')
        return True

    def disconnect(self):
        kinectDLL.close_kinect()

    def get_color_image(self):
        color_ary = np.empty((COLOR_HEIGHT, COLOR_WIDTH, COLOR_CHANNELS), np.uint8)
        if kinectDLL.get_color_data(color_ary):
            return color_ary
        return None

    def get_ir_image(self):
        ir_ary = np.empty((IR_HEIGHT, IR_WIDTH, 1), np.uint16)
        if kinectDLL.get_ir_data(ir_ary):
            return ir_ary
        return None

    def get_depth_map(self):
        depth_ary = np.empty((DEPTH_HEIGHT, DEPTH_WIDTH, 1), np.uint16)
        if kinectDLL.get_depth_data(depth_ary):
            return depth_ary
        return None

    def _process_bodies(self, body_ary, joint_ary):
        bodies = []
        for i in range(MAX_BODIES):
            if body_ary[i, 0]:
                bodies.append(Body(i, body_ary[i], joint_ary[i]))
                print(joint_ary[i])
        return bodies

    def get_bodies(self):
        body_ary = np.empty((MAX_BODIES, BODY_PROPS), np.uint8)
        joint_ary = np.empty((MAX_BODIES, MAX_JOINTS, JOINT_PROPS), np.int32)
        if kinectDLL.get_body_data(body_ary, joint_ary):
            return self._process_bodies(body_ary, joint_ary)
        return None

    def _process_audio(self, frame_cnt, audio_ary, meta_ary):
        frames = []
        for i in range(frame_cnt):
            beam_angle = meta_ary[i*2]
            beam_conf = meta_ary[i*2+1]
            samples = audio_ary[i*SUBFRAME_SIZE:(i+1)*SUBFRAME_SIZE]
            frames.append((beam_angle, beam_conf, samples))
        return frames

    def get_audio_frames(self):
        audio_ary = np.empty((AUDIO_BUF_LEN * SUBFRAME_SIZE,), np.float32)
        meta_ary = np.empty((AUDIO_BUF_LEN * 2,), np.float32)
        frame_cnt = kinectDLL.get_audio_data(audio_ary, meta_ary)
        if frame_cnt:
            return self._process_audio(frame_cnt, audio_ary, meta_ary)
        return []


class Body:

    def __init__(self, idx, body_ary, joint_ary):
        self.idx = idx
        self.tracked = body_ary[0]
        self._body_ary = body_ary
        self._joint_ary = joint_ary
        self._load_props()

    def _load_props(self):
        self.engaged = self._body_ary[1]
        self.restricted = self._body_ary[2]

    def keys(self):
        return JOINT_MAP.keys()

    def __getitem__(self, key):
        if isinstance(key, tuple):
            joint_name, pos_type = key
        else:
            joint_name, pos_type = key, 'color'
        joint_data = self._joint_ary[JOINT_MAP[joint_name]]
        if pos_type == 'color':
            track_data = (TRACKING_MAP[joint_data[0]], joint_data[1], joint_data[2])
        else:
            track_data = (TRACKING_MAP[joint_data[0]], joint_data[3], joint_data[4])
        if joint_name == 'hand_left':
            track_data += (HIGH_CONFIDENCE_MAP[self._body_ary[3]], HAND_MAP[self._body_ary[4]])
        elif joint_name == 'hand_right':
            track_data += (HIGH_CONFIDENCE_MAP[self._body_ary[5]], HAND_MAP[self._body_ary[6]])
        return track_data

    def __repr__(self):
        if self.tracked:
            state = ' [Tracked]'
        else:
            state = ''
        return '<Body ({}){}>'.format(self.idx, state)
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
BODY_PROPS      = 15
MAX_JOINTS      = 25
JOINT_PROPS     = 9
FLOAT_MULT      = 100000


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


def init_lib():

    kinectDLL = ctypes.cdll.LoadLibrary(resource_filename(__name__, 'data/Kinect2-API.dll'))

    kinectDLL.init_kinect.argtypes = [ctypes.c_int]
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

    return kinectDLL


class Kinect2:

    def __init__(self, use_sensors=['color']):
        self._kinect = init_lib()
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
        if not self._kinect.init_kinect(self.sensor_flags):
            raise IOError('Unable to init Kinect2 Sensor.')
        return True

    def disconnect(self):
        self._kinect.close_kinect()

    def get_color_image(self):
        color_ary = np.empty((COLOR_HEIGHT, COLOR_WIDTH, COLOR_CHANNELS), np.uint8)
        if self._kinect.get_color_data(color_ary):
            return color_ary
        return None

    def get_ir_image(self):
        ir_ary = np.empty((IR_HEIGHT, IR_WIDTH, 1), np.uint16)
        if self._kinect.get_ir_data(ir_ary):
            return ir_ary
        return None

    def get_depth_map(self):
        depth_ary = np.empty((DEPTH_HEIGHT, DEPTH_WIDTH, 1), np.uint16)
        if self._kinect.get_depth_data(depth_ary):
            return depth_ary
        return None

    def _get_raw_bodies(self):
        body_ary = np.empty((MAX_BODIES, BODY_PROPS), np.uint8)
        joint_ary = np.empty((MAX_BODIES, MAX_JOINTS, JOINT_PROPS), np.int32)
        if self._kinect.get_body_data(body_ary, joint_ary):
            return body_ary, joint_ary
        return None, None

    def get_bodies(self):
        body_ary, joint_ary = self._get_raw_bodies()
        bodies = []
        if body_ary is not None:
            for i in range(MAX_BODIES):
                if body_ary[i, 0]:
                    bodies.append(Body(i, body_ary[i], joint_ary[i]))
        return bodies

    def _get_raw_audio(self):
        audio_ary = np.empty((AUDIO_BUF_LEN * SUBFRAME_SIZE,), np.float32)
        meta_ary = np.empty((AUDIO_BUF_LEN * 2,), np.float32)
        frame_cnt = self._kinect.get_audio_data(audio_ary, meta_ary)
        return frame_cnt, audio_ary, meta_ary

    def get_audio_frames(self):
        frame_cnt, audio_ary, meta_ary = self._get_raw_audio()
        frames = []
        for i in range(frame_cnt):
            beam_angle = meta_ary[i*2]
            beam_conf = meta_ary[i*2+1]
            samples = audio_ary[i*SUBFRAME_SIZE:(i+1)*SUBFRAME_SIZE]
            frames.append((beam_angle, beam_conf, samples))
        return frames


class Body:

    def __init__(self, idx, body_ary, joints_ary):
        self.idx = idx
        self._body_ary = body_ary
        self._joints_ary = joints_ary
        self._load_props()

    def _load_props(self):
        self.tracked = self._body_ary[0]
        self.engaged = DETECTION_MAP[self._body_ary[1]]
        self.restricted = bool(self._body_ary[2])
        self.neutral = DETECTION_MAP[self._body_ary[7]]
        self.happy = DETECTION_MAP[self._body_ary[8]]
        self.looking_away = DETECTION_MAP[self._body_ary[13]]
        self.glasses = DETECTION_MAP[self._body_ary[14]]

    def keys(self):
        return JOINT_MAP.keys()

    def __getitem__(self, joint_name):
        joint_idx = JOINT_MAP[joint_name.lower()]
        if joint_idx == -1:
            raise NotImplementedError()
        return Joint(joint_name, self._body_ary, self._joints_ary[joint_idx])

    def __repr__(self):
        if self.tracked:
            state = ' [Tracked]'
        else:
            state = ''
        return '<Body ({}){}>'.format(self.idx, state)


class Joint:

    def __init__(self, joint_name, body_ary, joint_ary):
        self.name = joint_name
        self._body_ary = body_ary
        self._joint_ary = joint_ary
        self._load_props()
    
    def _load_props(self):
        self.tracking = TRACKING_MAP[self._joint_ary[0]]
        self.color_pos = (self._joint_ary[1], self._joint_ary[2])
        self.depth_pos = (self._joint_ary[3], self._joint_ary[4])
        self.orientation = (
            self._joint_ary[5] / FLOAT_MULT,
            self._joint_ary[6] / FLOAT_MULT,
            self._joint_ary[7] / FLOAT_MULT,
            self._joint_ary[8] / FLOAT_MULT
        )
        if self.name == 'hand_left':
            self.confidence = HIGH_CONFIDENCE_MAP[self._body_ary[3]]
            self.state = HAND_MAP[self._body_ary[4]]
        elif self.name == 'hand_right':
            self.confidence = HIGH_CONFIDENCE_MAP[self._body_ary[5]]
            self.state = HAND_MAP[self._body_ary[6]]
        else:
            self.confidence = None
            self.state = None

    def __repr__(self):
        if self.state:
            return '<Joint {} [{}] [{}]>'.format(self.name.title(), self.state, self.tracking)
        else:
            return '<Joint {} [{}]>'.format(self.name.title(), self.tracking)
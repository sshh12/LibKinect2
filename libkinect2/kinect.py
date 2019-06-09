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
JOINT_PROPS     = 1


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

    def get_bodies(self):
        body_ary = np.empty((MAX_BODIES, BODY_PROPS), np.uint8)
        joint_ary = np.empty((MAX_BODIES, MAX_JOINTS, JOINT_PROPS), np.int32)
        if kinectDLL.get_body_data(body_ary, joint_ary):
            return body_ary, joint_ary
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
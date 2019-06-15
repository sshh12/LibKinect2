"""
Code related to body tracking.
"""
from pkg_resources import resource_filename
from .dll_lib import *
from .utils import dist
import cv2

try:
    import dlib
    face_detector = dlib.get_frontal_face_detector()
    face_feat_detector = dlib.shape_predictor(
        resource_filename(__name__, 'data/shape_predictor_68_face_landmarks.dat'))
    DLIB_LOADED = True
except ImportError:
    DLIB_LOADED = False


class Body:
    """
    A body tracked by the Kinect.

    Attributes:
        idx: The tracking index
        tracked: If this body is tracked
        engaged: State of person's engagement
        restricted: If the body is restricted
    """
    def __init__(self, idx, body_ary, joints_ary):
        """
        Create a body from raw body/joint data.

        Note:
            Should not be called by user.
            Use `kinect.get_bodies()`.
        """
        self.idx = idx
        self._body_ary = body_ary
        self._joints_ary = joints_ary
        self._joints_cache = {}
        self._load_props()

    def _load_props(self):
        self.tracked = bool(self._body_ary[0])
        ## These are not yet supported by Kinect2 )':
        ## self.engaged = DETECTION_MAP[self._body_ary[1]]
        ## self.restricted = bool(self._body_ary[2])
        ## self.neutral = DETECTION_MAP[self._body_ary[7]]
        ## self.happy = DETECTION_MAP[self._body_ary[8]]
        ## self.looking_away = DETECTION_MAP[self._body_ary[13]]
        ## self.glasses = DETECTION_MAP[self._body_ary[14]]
        
    def get_face(self, color_img):
        """
        Use body data (self) and color_img to
        extract their face.
        """
        if not DLIB_LOADED:
            raise Exception('Dlib is required to use this method.')
        head = self.__getitem__('head')
        neck = self.__getitem__('neck')
        face = Face(color_img, head, neck)
        if face.exists:
            return face
        return None

    def keys(self):
        """
        Return a list of keys (joints).

        Returns:
            list of joint names
        """
        return JOINT_MAP.keys()

    def __getitem__(self, joint_name):
        """
        Get a joint of this body by name.

        Returns:
            `Joint`

        Note:
            Use `body.keys()` for list of joints.
        """
        joint_name = joint_name.lower()

        if joint_name in self._joints_cache:
            return self._joints_cache[joint_name]

        joint_idx = JOINT_MAP[joint_name.lower()]
        if joint_idx == -1:
            raise NotImplementedError()

        joint = Joint(joint_name, self._body_ary, self._joints_ary[joint_idx])
        self._joints_cache[joint_name] = joint
        return joint

    def __repr__(self):
        if self.tracked:
            state = ' [Tracked]'
        else:
            state = ''
        return '<Body ({}){}>'.format(self.idx, state)


class Joint:
    """
    A joint.

    Attributes:
        name: Name of the joint
        tracking: The current tracking state
        color_pos: Position in the color camera space - (x, y)
        depth_pos: Position in the depth sensor space - (x, y)
        orientation: Orientation as (w, x, y, z)
        state: The state of the joint if provided by Kinect API
    """
    def __init__(self, joint_name, body_ary, joint_ary):
        """
        Create a joint from raw body/joint data.

        Note:
            Should not be called by user.
            Use `body[joint_name]`.
        """
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


class Face:
    """
    A person's face.
    """
    def __init__(self, color_img, head, neck):
        """
        Create face from an image and head/neck joints.

        Note:
            Should not be called by user.
            Use `body.get_face()`.
        """
        self.color_img = color_img
        self.head = head
        self.pos = head.color_pos
        self.neck = neck
        self.exists = False
        self.rect = None
        self.points = None
        self._find()

    def _find(self):
        
        head_x, head_y = self.pos
        if self.head.tracking != 'tracked' or self.neck.tracking != 'tracked' or min(head_x, head_y) <= 0:
            return

        radius = int(dist(self.head.color_pos, self.neck.color_pos) * 1.5)
        radius = min([head_x, head_y, radius])
        x1, x2 = head_x-radius, head_x+radius
        y1, y2 = head_y-radius, head_y+radius
        face_img = self.color_img[y1:y2, x1:x2, :]

        if 0 in face_img.shape:
            return

        self.face_img = np.copy(face_img)
        rects = face_detector(self.face_img, 1)

        if len(rects) == 0:
            return

        self.exists = True
        self.rect = rects[0]
        self.points = np.zeros((68, 2), dtype=np.int)

        shape = face_feat_detector(self.face_img, self.rect)
        for i in range(68):
            self.points[i] = (shape.part(i).x + x1, shape.part(i).y + y1)

    def __repr__(self):
        if self.exists:
            return '<Face [Valid @ {}]>'.format(self.pos)
        else:
            return '<Face [Invalid]>'
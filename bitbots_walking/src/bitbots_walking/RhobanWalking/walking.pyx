# -*- encoding: utf8 -*-
from bitbots_common.pose.pose cimport Pose as CPose
from bitbots_common.pose.pypose cimport PyPose, wrap_pose
from libcpp cimport bool
from libcpp.string cimport string
from bitbots_common.eigen cimport Vector3f
from bitbots_common.eigen cimport Vector2d
from bitbots_common.eigen cimport Vector3d
from cython.operator cimport dereference as deref, address as ref


cdef class ZMPWalkingEngine:

    def __cinit__(self):
        self.thisptr = new _RhobanWalk()

    def __dealloc__(self):
        del self.thisptr

    cpdef start(self):
        self.thisptr.start()

    cpdef stop(self):
        self.thisptr.stop()

    cpdef process(self):
        cdef unsigned char phase
        phase = self.thisptr.update()

        return int(phase)

    cpdef setParams(self, object config):
        print("Params Setzen")
        print(config["trunkXOffset"])
        self.parameters = ref(self.thisptr.getParams())
        self.parameters.trunkXOffset = config["trunkXOffset"]
        #self.thisptr.getParams().trunkXOffset = config["trunkXOffset"]
        #print  parameters.trunkXOffset


    cpdef set_active(self, bool active):
        pass


    cpdef stance_reset(self):
        pass

    cpdef set_velocity(self, float x, float y, float z):
        self.thisptr.set_velocity(x/3.0, y/5.0, z/5.0) #TODO

    cpdef set_gyro(self, float x, float y, float z):
        self.thisptr.set_gyro(x, y, z)

    property running:
        def __get__(self):
            return self.thisptr.is_active()

    property velocity:
        def __set__(self, object val):
            cdef float x, y, z
            x, y, z = val
            self.thisptr.set_velocity(x, y, z)

    property pose:
        def __get__(self):
            cdef PyPose p = wrap_pose(self.thisptr.get_pose())
            return p

    property frequenzy:
        def __set__(self, value):
            self.thisptr.set_frequenzy(value)

    cpdef create_walkready_pose(self, dict config={}, duration=1):
        """
            Creates a new temporary ZMPWalkingengine and does on iteration with it.
            Afterwards the pose which derives of the walking iteration, will be extracted
            and transfered to an animation.
        """
        walking = ZMPWalkingEngine()
        # parame walking
        walking.set_active(True)
        walking.process()
        walking.process()
        walking.process()
        pose = walking.pose
        #for joint in pose:
        #    joint.set_speed((joint.goal - joint.position)/duration)

        # An animation is a dict with additional information and a list of keyframes.
        # A keyframe is a dict with a duration and a list of motor goals
        animation = {"name": "walkready",
            "keyframes": [{"duration":duration, "goals":{joint[0]:joint[1].goal for joint in pose}}]}
        print("walk: test")
        return animation

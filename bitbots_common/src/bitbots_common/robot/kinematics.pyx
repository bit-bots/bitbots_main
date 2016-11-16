from libcpp cimport bool
from libcpp.vector cimport vector
from libcpp.map cimport map
from libcpp.string cimport string
from cython.operator cimport address as ref
from cython.operator cimport dereference as deref

import time

from bitbots_common.eigen cimport *
import numpy as np
cimport numpy as np
# Nötig, wenn man Numpy ordentlich mit Cython nutzen will
np.import_array()

from bitbots_common.util import get_config
from bitbots_common.pose.pose cimport Pose
from bitbots_common.pose.pypose cimport PyPose

def rfoot():
    return RFoot
def lfoot():
    return LFoot

# Important the implementation of the class Robot is in this included file robot.pxi
include "robot.pxi"

cdef class Joint:

    def __cinit__(self, string name):
        self.name = name
        self.has_value = False

    def __dealloc__(self):
        del self.joint

    cdef set_joint(self, const _Joint& joint):
        if self.has_value is True:
            del self.joint
        self.joint = new _Joint(joint)
    cdef init_from_joint(self, const _Joint& joint):
        if self.has_value is True:
            del self.joint
        self.joint = new _Joint(joint)

    cpdef get_angle(self):
        return self.joint.get_angle()

    cpdef update_angle(self, float degree):
        self.joint.update_angle(degree)

    cpdef np.ndarray get_chain_matrix(self, bool inverse=False):
        cdef Affine3d cm = self.joint.get_chain_matrix_inverse() if inverse else self.joint.get_chain_matrix()
        cdef np.ndarray mat = matrix4d_to_numpy(cm.matrix())
        mat[0:3,3] = mat[0:3,3] * 1000
        return mat

    cpdef np.ndarray get_transform(self, bool inverse=False):
        cdef Affine3d tr = self.joint.get_inverse_transform() if inverse else self.joint.get_transform()
        cdef np.ndarray mat = matrix4d_to_numpy(tr.matrix())
        mat[0:3,3] = mat[0:3,3] * 1000
        return mat

    cpdef np.ndarray get_centre_of_gravity(self, bool with_mass=False):
        cdef Vector4d cog
        cog = self.joint.get_centre_of_gravity() if with_mass else self.joint.get_normalized_centre_of_gravity()
        cdef np.ndarray vec = np.array((cog.x(), cog.y(), cog.z()),dtype=np.float32)
        return vec * 1000

    cpdef np.ndarray get_chain_masspoint(self):
        cdef Vector4d cmp = self.joint.get_chain_masspoint()
        cdef np.ndarray vec = np.array((cmp.x(), cmp.y(), cmp.z()),dtype=np.float32)
        return vec * 1000

    cpdef np.ndarray get(self):
        cdef Matrix4d tr = self.joint.get_transform().matrix()
        cdef np.ndarray mat = matrix4d_to_numpy(tr)
        mat[0:3,3] = mat[0:3,3] * 1000
        return mat

    cpdef np.ndarray get_endpoint(self):
        cdef Vector4d ep = self.joint.get_endpoint()
        return np.array((ep.x(), ep.y(), ep.z()), dtype=np.float32) * 1000

    cdef const _Joint* get_joint(self):
        return self.joint

    cpdef int get_id(self):
        return self.joint.get_id()

    cdef string get_name(self):
        return str(self.name)


cdef class ChainHolder:

    def __cinit__(self):
        pass
    def __dealloc__(self):
        pass

    cdef _ChainHolder get(self):
        return deref(self.chain)

    cdef set(self, _ChainHolder chain):
        self.chain = new _ChainHolder(chain)

    cpdef Joint get_joint(self, int idx):
        cdef Joint rvalue = Joint("")
        rvalue.set_joint(self.chain.get_joint(idx))
        return rvalue

    cpdef int size(self):
        return self.chain.size() if self.chain is not NULL else 0


cdef class KinematicTarget:

    def __cinit__(self, int axis, int fromm, int to, float error, object target):
        self.axis = axis
        self.fromm = fromm
        self.to = to
        self.error = error
        self.target = Vector3d(target[0], target[1], target[2])

    property target:
        def __get__(self):
            return (self.target.at(0), self.target.at(1), self.target.at(2))

    property axis:
        def __get__(self):
            return self.axis
    property error:
        def __get__(self):
            return self.error
    property fromm:
        def __get__(self):
            return self.fromm
    property to:
        def __get__(self):
            return self.to

cdef class KinematicTask:

    def __cinit__(self, Robot robot):
        self.robot = robot.robot
        self.task = new _KinematicTask(deref(robot.robot))
        self.targets = []
        self.executable = False
        self.subtask = None

    cpdef reset(self):
        self.targets = []
        self.executable = False

    cpdef set_subtask(self, KinematicTask subtask):
        self.subtask = subtask
        self.task.set_subtask(self.subtask.task)
        #raise NotImplementedError("The current version of the kinemaic framework is not capable of calculating subtakst")

    cpdef add_target(self, int fromm, int to, int axis, float error, object target):
        if axis == 0:
            self.add_x_orientation(fromm, to, error, target)
        elif axis == 1:
            self.add_y_orientation(fromm, to, error, target)
        elif axis == 2:
            self.add_z_orientation(fromm, to, error, target)
        elif axis == 3:
            self.add_position(fromm, to, error, target)
        else:
            raise ValueError("This axis does not exist % d" % axis)

    cpdef add_position(self, int fromm, int to, float error, object target):
        self.targets.append(KinematicTarget(3, fromm, to, error / 1000, target))
    cpdef add_x_orientation(self, int fromm, int to, float error, object target):
        self.targets.append(KinematicTarget(0, fromm, to, error, target))
    cpdef add_y_orientation(self, int fromm, int to, float error, object target):
        self.targets.append(KinematicTarget(1, fromm, to, error, target))
    cpdef add_z_orientation(self, int fromm, int to, float error, object target):
        self.targets.append(KinematicTarget(2, fromm, to, error, target))

    cpdef execute(self, int iterations):
        if not self.executable:
            self.prepare()
        cdef int it = self.task.execute(iterations)
        print "Benötigte %d Iterationen" % (iterations - it)

    cpdef prepare(self):
        if not self.targets:
            return
            # empty list, nothing to do
        if self.subtask and not self.subtask.executable:
            self.subtask.prepare()
        cdef MAxisType axx = MAxisType(len(self.targets), 1)
        cdef MTargetType t = VectorXd(3 * len(self.targets), 1)
        cdef MErrorType e  = VectorXd(len(self.targets), 1)
        for i in range(len(self.targets)):
            axx.block(i,0,1,1).insert(int_to_axis(self.targets[i].axis))
            t.block(3 * i, 0, 3, 1).insert(Vector3d(self.targets[i].target[0], self.targets[i].target[1], self.targets[i].target[2]))
            if(self.targets[i].axis == 3):
                t.block(3 * i, 0, 3, 1).insert(t.block(3 * i, 0, 3, 1) / 1000)
            e.block(0,i,1,1).insert(<double>self.targets[i].error)
        self.task.set_target_values(axx, int_to_JointID(self.targets[0].fromm), int_to_JointID(self.targets[0].to), t, e)
        self.executable = True

    cpdef update_target(self, target, number=0):
        cdef KinematicTarget kt = self.targets[number]
        kt.target = Vector3d(target[0], target[1], target[2])
        cdef MTargetType t = VectorXd(3 * len(self.targets), 1)
        for i in range(len(self.targets)):
            t.block(3 * i, 0, 3, 1).insert(Vector3d(self.targets[i].target[0], self.targets[i].target[1], self.targets[i].target[2]))
            if(self.targets[i].axis == 3):
                t.block(3 * i, 0, 3, 1).insert(t.block(3 * i, 0, 3, 1) / 1000)
        if not self.executable:
            self.prepare()
        self.task.update_target(t)

    def __dealloc__(self):
        #del self.robot
        del self.task
        # When giving the subtask to a kinematic task, it takes care of it,
        # So at this point, the subtask task is already freed
        if self.subtask:
            self.subtask.task = <_KinematicTask*>0

    cpdef ChainHolder create_chain(self, int fromm, int to):
        cdef ChainHolder rvalue = ChainHolder()
        cdef JointID from_id = int_to_JointID(fromm)
        cdef JointID to_id = int_to_JointID(to)

        del self.task
        self.task = new _KinematicTask(deref(self.robot))
        rvalue.set(self.task.create_chain(from_id, to_id,))
        return rvalue

    cpdef ChainHolder create_cog_chain(self, int chain_id):
        ###_ChainHolder create_centre_of_gravity_chain(ChainIds chain_id, set[int] ignore_joints, _DisableType disable_chain)
        cdef ChainHolder rvalue = ChainHolder()
        cdef ChainID chain = int_to_ChainID(chain_id)
        cdef _ChainHolder c = self.task.create_centre_of_gravity_chain(chain)
        rvalue.set(c)
        return rvalue

    cpdef update_chain(self, ChainHolder chain, int flags):
        self.task.update_robot_chain(deref(chain.chain), flags)

from bitbots_common.pose.pose cimport Pose, Joint, get_num_joints
from libcpp cimport bool

from libcpp.string cimport string

cdef int num_joints = get_num_joints()

cpdef int num_pose_joints()

cdef class PyJoint:
    cdef PyPose pose
    cdef Joint* joint

    cdef set_joint(self, PyPose pose, Joint* joint)

    cdef set_active(self, bool active)
    cdef reset(self)
    cdef set_goal(self, float goal)
    cdef set_speed(self, float speed)
    cdef set_position(self, float position)
    cdef set_load(self, float load)
    cdef set_maximum(self, float maximum)
    cdef set_minimum(self, float minimum)
    cdef set_p(self, int p)
    cdef set_i(self, int i)
    cdef set_d(self, int d)
    cdef bool is_active(self)
    cdef bool has_changed(self)
    cdef float get_goal(self)
    cdef float get_speed(self)
    cdef float get_position(self)
    cdef float get_load(self)
    cdef float get_maximum(self)
    cdef float get_minimum(self)

    cdef int get_p(self)
    cdef int get_i(self)
    cdef int get_d(self)

    cdef int get_cid(self)

cdef class PyPose:
    cdef bool is_reference
    cdef Pose* pose

    cdef set_c_pose(self, Pose& pose)
    cdef set_c_pose_ref(self, Pose& pose)
    cdef Pose* get_pose_ptr(self)

    cpdef PyJoint get_joint(self, bytes name)
    cpdef PyJoint get_joint_by_cid(self, int cid)
    cpdef string get_joint_name(self, int cid)

    cpdef list get_positions(self)
    cpdef list get_speeds(self)

    cpdef reset(self)
    cpdef update(self, PyPose other)
    cpdef update_positions(self, PyPose other)

cdef PyPose wrap_pose(Pose& pose)
cdef PyPose wrap_pose_obj(Pose pose)
cdef PyPose wrap_pose_ref(Pose& pose)


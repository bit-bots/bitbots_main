from bitbots_common.pose.pose cimport Pose, Joint, get_num_joints
from libcpp cimport bool

from libcpp.string cimport string
from libcpp.vector cimport vector

cdef int num_joints = get_num_joints()

cpdef int num_pose_joints()

cdef class PyJoint:
    cdef PyPose pose
    cdef Joint* joint

    cdef set_joint(self, PyPose pose, Joint* joint)

    cpdef set_active(self, bool active)
    cpdef reset(self)
    cpdef set_goal(self, float goal)
    cpdef set_speed(self, float speed)
    cpdef set_position(self, float position)
    cpdef set_load(self, float load)
    cpdef set_temperature(self, float temp)
    cpdef set_voltage(self, float volt)
    cpdef set_maximum(self, float maximum)
    cpdef set_minimum(self, float minimum)
    cpdef set_p(self, int p)
    cpdef set_i(self, int i)
    cpdef set_d(self, int d)
    cpdef bool is_active(self)
    cpdef bool has_changed(self)
    cpdef float get_goal(self)
    cpdef float get_speed(self)
    cpdef float get_position(self)
    cpdef float get_load(self)
    cpdef float get_temperature(self)
    cpdef float get_voltage(self)
    cpdef float get_maximum(self)
    cpdef float get_minimum(self)

    cpdef int get_p(self)
    cpdef int get_i(self)
    cpdef int get_d(self)

    cpdef int get_cid(self)

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
    cpdef list get_positions_rad(self)
    cpdef list get_speeds(self)
    cpdef list get_goals(self)
    cpdef list get_loads(self)
    cpdef set_positions(self, list names, list positions)
    cpdef set_speeds(self, list names, list speeds)
    cpdef set_goals(self, list names, list speeds)
    cpdef set_loads(self, list names, list loads)


    cpdef reset(self)
    cpdef update(self, PyPose other)
    cpdef update_positions(self, PyPose other)

cdef PyPose wrap_pose(Pose& pose)
cdef PyPose wrap_pose_obj(Pose pose)
cdef PyPose wrap_pose_ref(Pose& pose)
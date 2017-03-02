from libcpp cimport bool
from libcpp.vector cimport vector
from libcpp.set cimport set
from libcpp.map cimport map
from libcpp.string cimport string
from cython.operator cimport dereference as deref
from bitbots_common.eigen cimport *

cimport numpy as np
from bitbots_common.boost cimport reference_wrapper

from bitbots_common.pose.pose cimport Pose
from bitbots_common.pose.pypose cimport PyPose

cdef inline np.ndarray matrix4d_to_numpy(const_Matrix4d& c):
    cdef np.ndarray py = np.empty((4, 4), dtype=np.float32)

    cdef int i
    for i in range(4):
        py[i] = (c.at(i, 0), c.at(i, 1), c.at(i, 2), c.at(i, 3))

    return py

cdef extern from "jointids.hpp" namespace "Robot::Kinematics::JointIds":
    cdef enum JointIds:
        Root, RShoulderPitch, LShoulderPitch, RShoulderRoll, LShoulderRoll, RElbow, LElbow, RHipYaw, LHipYaw, RHipRoll, LHipRoll, RHipPitch, LHipPitch,
        RKnee, LKnee, RAnklePitch, LAnklePitch, RAnkleRoll, LAnkleRoll, Neck, Camera, RFootEndpoint, LFootEndpoint, LArmEndpoint, RArmEndpoint
cdef extern from "jointids.hpp" namespace "Robot::Kinematics::ChainIds":
    cdef enum ChainIds:
        HeadChain, RArmChain, LArmChain, RLegChain, LLegChain

cdef extern from "jointids.hpp" namespace "Robot::Kinematics":
    JointIds int_to_JointID(int id)
    ChainIds int_to_ChainID(int id)
    ctypedef JointIds JointID "::Robot::Kinematics::JointIds"
    ctypedef ChainIds ChainID "::Robot::Kinematics::ChainIds"

cdef int    Center=Root, RShoulderP=RShoulderPitch, LShoulderP=LShoulderPitch, RShoulderR=RShoulderRoll, LShoulderR=LShoulderRoll, RElbo=RElbow, LElbo=LElbow, \
            RHipY=RHipYaw, LHipY=LHipYaw, RHipR=RHipRoll, LHipR=LHipRoll, RHipP=RHipPitch, LHipP=LHipPitch, RKne=RKnee, LKne=LKnee, RAnkleP=RAnklePitch, LAnkleP=LAnklePitch, \
            RAnkleR=RAnkleRoll, LAnkleR=LAnkleRoll, RFoot=RFootEndpoint, Cam=Camera, LFoot=LFootEndpoint, RArm=RArmEndpoint, LArm=LArmEndpoint

cdef int    HeadC = HeadChain, RArmC = RArmChain, LArmC = LArmChain, RLegC = RLegChain, LLegC = LLegChain

cdef extern from "kinematic_joint.hpp" namespace "Robot::Kinematics":
    cppclass _Joint "::Robot::Kinematics::KJoint":
        _Joint()
        _Joint(const _Joint& other)
        _Joint(Vector3d transform, Vector3d rotations, vectorForEigen4d masses, Vector3d angles, int id)
        const Affine3d& get()
        const Vector4d& get_endpoint()
        const Affine3d& get_transform()
        const Affine3d& get_inverse_transform()
        const Affine3d& get_chain_matrix()
        const Vector4d& get_centre_of_gravity()
        const Vector4d& get_normalized_centre_of_gravity()
        const Vector4d& get_chain_masspoint()
        double get_angle()
        void update_angle(double degree)
        Affine3d get_chain_matrix_inverse()
        int get_id()

    cdef enum AxisType "::Robot::Kinematics::KJoint::AxisType":
        XAxis, YAxis, ZAxis, Position

    AxisType int_to_axis(int a)

    ctypedef AxisType AxisT "::Robot::Kinematics::KJoint::AxisType"

cdef extern from "vector":
    cppclass jointvector "std::vector<Robot::Kinematics::KJoint, Eigen::aligned_allocator<Robot::Kinematics::KJoint> >":
        jointvector()
        void push_back(_Joint next)

    cppclass vectorForEigen4d "std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >":
        vectorForEigen4d()
        void push_back(Vector4d next)

cdef extern from "kinematic_robot.hpp":

    cdef enum _UpdateChainFlags "Robot::Kinematics::KRobot::UpdateChainFlags":
        both

    ctypedef _Joint const_Joint "const Robot::Kinematics::KJoint"

    # Hübsch ist was anderes... Danke Cython :/
    ctypedef vector[reference_wrapper[_Joint]] Chain "Robot::Kinematics::KRobot::Chain"
    ctypedef Chain const_Chain "const Robot::Kinematics::KRobot::Chain"

    ctypedef vector[reference_wrapper[_Joint]].iterator ChainIterator "Robot::Kinematics::KRobot::Chain::iterator"
    ctypedef ChainIterator const_ChainIterator "Robot::Kinematics::KRobot::Chain::const_iterator"
    ctypedef Matrix[AxisType] MAxisType "Robot::Kinematics::KRobot::MultipleAxisType"
    ctypedef Matrix[double] MTargetType "Robot::Kinematics::KRobot::MultipleTargetType"
    ctypedef Matrix[double] MErrorType "Robot::Kinematics::KRobot::MultipleErrorType"

    cppclass _Robot "::Robot::Kinematics::KRobot":
        _Robot()
        _Robot(const map[string, int]& idMapping, const map[string, int]& chainMapping, \
            const jointvector& joints, const vector[vector[int]]& chains_template, int max_id)
        void update(VectorXd&)
        void update(const Pose& pose)
        void update_robot_masses()
        void set_initial_angles(const Vector3d& rpy)
        void set_initial_angles_rad(const Vector3d& rpy)
        void reset_initial_angles()
        void set_angles_to_pose(Pose& pose, int chain_id, float time)
        const _Joint& get_joint_by_id(int id)
        const _Joint& get_joint_by_name(string name)
        Chain& get_chain_by_name(string name)
        Chain& get_chain_by_id(int id)
        Vector4d get_centre_of_gravity()
        Vector4d get_centre_of_gravity(const Matrix4d& offset)
        double get_mass()
        void print_robot_data_for_debug()
        int inverse_chain(int id, Vector3d target)
        int inverse_chain(int id, Vector3d target, double error, int max_it)
        int inverse_chain(int id, Vector3d target, double error, int max_it, AxisType axis)
        int inverse_chain(int id, VectorXd target, Vector2d error, int max_it, MAxisType axis)
        int inverse_chain(Chain& chain, Vector3d& target, double error, int max_it, AxisType axis)
        int inverse_chain(Chain& chain, VectorXd& target, Vector2d error, int max_it, MAxisType axis)

cdef _UpdateChainFlags updateFlag = both
ctypedef unsigned int unit

cdef extern from "kinematic_task.hpp":
    cppclass _ChainHolder "Robot::Kinematics::KinematicTask::ChainHolder":
        _ChainHolder()
        _ChainHolder(_ChainHolder)
        _Joint& get_joint(int idx)
        int size()


    cppclass _KinematicTask "Robot::Kinematics::KinematicTask":

        _KinematicTask(const _Robot&)
        void perform_centre_of_gravity_task(ChainIds fromm, const Vector3d& target, set[int] ignore_joints, double error, int max_it)
        _ChainHolder create_centre_of_gravity_chain(ChainIds)
        _ChainHolder create_chain(JointIds, JointIds)
        void update_robot_chain(_ChainHolder chain)
        void update_robot_chain(_ChainHolder chain, unsigned int update_flags)
        void set_target_values(MAxisType axis, JointIds fromm, JointIds to, VectorXd& target_position, VectorXd& error)
        void update_target(VectorXd& target)
        int execute(int it)
        void set_subtask(_KinematicTask* subtask)

cdef class Joint:
    cdef _Joint* joint
    cdef bool has_value
    cdef object name

    cdef set_joint(self, const _Joint& joint)
    cdef init_from_joint(self, const _Joint& joint)
    cpdef get_angle(self)
    cpdef update_angle(self, float degree)
    cpdef np.ndarray get(self)
    cpdef np.ndarray get_endpoint(self)
    cpdef np.ndarray get_centre_of_gravity(self, bool with_mass=?)
    cpdef np.ndarray get_chain_masspoint(self)
    cpdef np.ndarray get_chain_matrix(self, bool inverse=?)
    cpdef np.ndarray get_transform(self, bool inverse=?)
    cdef const _Joint* get_joint(self)
    cpdef int get_id(self)
    cdef string get_name(self)

cdef class Robot:
    cdef _Robot* robot
    cdef bool is_initialized_empty

    cdef const _Joint* get_c_joint_by_id(self, int id)
    cdef const _Joint* get_c_joint_by_name(self, string name)
    cpdef Joint get_joint_by_id(self, int id)
    cpdef Joint get_joint_by_name(self, string name)
    cpdef debugprint(self)
    cpdef update(self, PyPose pose)
    cpdef update_masses(self)
    cpdef np.ndarray get_centre_of_gravity(self, bool with_mass=?)
    cpdef np.ndarray get_centre_of_gravity_with_offset(self, np.ndarray offset)
    cpdef float get_mass(self)
    cpdef PyPose set_angles_to_pose(self, PyPose pose, int chain_id=?, float time=?)
    cpdef set_initial_angles(self, float roll, float pitch, float yaw)
    cpdef reset_initial_angles(self)

    ###Kompatibilität zum bestehenden Code
    cpdef update_with_matrix(self, np.ndarray ndarr)
    cpdef Joint get_l_foot_endpoint(self)
    cpdef Joint get_r_foot_endpoint(self)
    cpdef Joint get_l_arm_endpoint(self)
    cpdef Joint get_r_arm_endpoint(self)
    cpdef Joint get_camera(self)
    cpdef Joint get_center(self)
    cdef list chain_to_list(self, const_Chain& chain)
    cpdef list get_l_arm_chain(self)
    cpdef list get_l_leg_chain(self)
    cpdef list get_r_arm_chain(self)
    cpdef list get_r_leg_chain(self)
    cpdef list get_head_chain(self)

cdef class ChainHolder:
    cdef _ChainHolder* chain

    cdef _ChainHolder get(self)
    cdef set(self, _ChainHolder chain)
    cpdef Joint get_joint(self, int idx)
    cpdef int size(self)

cdef class KinematicTarget:
    cdef int axis, fromm, to
    cdef double error
    cdef Vector3d target

cdef class KinematicTask:
    cdef _KinematicTask* task
    cdef _Robot* robot
    cdef list targets
    cdef KinematicTask subtask
    cdef bool executable

    cpdef reset(self)
    cpdef prepare(self)
    cpdef execute(self, int iterations)
    cpdef update_target(self, target, number=?)
    cpdef add_target(self, int fromm, int to, int axis, float error, object target)
    cpdef add_position(self, int fromm, int to, float error, object target)
    cpdef add_x_orientation(self, int fromm, int to, float error, object target)
    cpdef add_y_orientation(self, int fromm, int to, float error, object target)
    cpdef add_z_orientation(self, int fromm, int to, float error, object target)
    cpdef set_subtask(self, KinematicTask subtask)

    cpdef ChainHolder create_chain(self, int fromm, int to)
    cpdef ChainHolder create_cog_chain(self, int chain_id)
    cpdef update_chain(self, ChainHolder chain, int flags)

cdef inline Joint init_joint_from_config(dict joint_config):
    cdef int id = joint_config["id"]
    cdef Vector3d transform, rotations, angles
    cdef vectorForEigen4d masses = vectorForEigen4d()
    cdef object id_string = "Current Joint %d has malformed config." % id

    cdef float mass
    cdef float off_x, off_y, off_z
    cdef dict offsets = joint_config["mass_offsets"]

    try:
        for part in offsets:
            mass = offsets[part]["mass"]
            off_x, off_y, off_z = offsets[part]["offset"]
            masses.push_back(Vector4d(off_x / 1000.0, off_y / 1000.0, off_z / 1000.0, mass))
    except Exception, e:
        print(e)
        raise Exception(e, "Error while filling offsets", id_string)

    cdef float x, y, z, r, p, yaw
    try:
        x, y, z = joint_config["transform"]
        transform = Vector3d(x, y, z) / 1000
        r, p, yaw = joint_config["rpy"]
        rotations = Vector3d(r, p, yaw)
    except Exception, e:
        print(e)
        raise Exception(e, "Error while creating transform", id_string)

    cdef int default, min , max
    try:
        default, min ,max = joint_config["def_min_max_angles"]
        #angles = Vector3d(0, -180, 180)
        angles = Vector3d(int(default), int(min), int(max))
    except Exception, e:
        print(e)
        raise Exception(e, "Error while setting angles", id_string)

    cdef Joint joint = Joint(joint_config["name"])
    cdef _Joint c_joint = _Joint(transform, rotations, masses, angles, id)
    joint.set_joint(c_joint)

    return joint

cdef inline set_joint_config_limits(dict joint, list joints_angles, int max):
    cdef int joint_id = int(joint["id"])
    if int(joint["id"]) <= max:
        limits = joints_angles[joint_id - 1]["limits"]
        joint["def_min_max_angles"] = [ limits["default"], \
                                        limits["min"], \
                                        limits["max"] ]
    else:
        joint["def_min_max_angles"] = [0, 0, 0]


cdef inline _Robot* create_robot_from_config(object config):
    cdef dict robot
    robot = config[config["RobotTypeName"]]
    cdef list joint_specialization = config["JointOverride"]
    cdef list joints_angles = config["joints"]
    cdef dict joints = {}
    cdef int max_id = robot["max_joint_id"]

    #add default angles to root config, because the angles are provided in a extra config file
    robot["Root"]["def_min_max_angles"] = [0, 0, 0]
    #insert Root _Joint
    joints[int(robot["Root"]["id"])] = init_joint_from_config(robot["Root"])
    cdef list chains = robot["ChainNames"]
    #insert all joints in dict
    cdef dict limits
    for chain in chains:
        for joint in robot[str(chain)]:
            set_joint_config_limits(joint, joints_angles, max_id)
            joints[int(joint["id"])] = init_joint_from_config(joint)
    #Adding "Virtual joints to the jointvector, those that are unused by the robot"
    for joint in robot["Virtual"]:
        set_joint_config_limits(joint, joints_angles, max_id)
        joints[int(joint["id"])] = init_joint_from_config(joint)
    #allow overriding joint e.g. Camera, for the Robots
    if joint_specialization is not None:
        for joint in joint_specialization:
            set_joint_config_limits(joint, joints_angles, max_id)
            #some prints here, because I needed to test this
            #print "old joint: %d \n %s" % (joints[int(joint["id"])].get_id(), (joints[int(joint["id"])].get()) )
            joints[int(joint["id"])] = init_joint_from_config(joint)
            #print "new joint: %d \n %s" % (joints[int(joint["id"])].get_id(), (joints[int(joint["id"])].get()) )
            print("Set specialized configuration for %s \n %s" % (joint["name"], joint))

    #init the chain template with joint ids
    cdef vector[vector[int]] chain_template = vector[vector[int] ]()
    cdef vector[int] chain_ids
    for chain in chains:
        chain_ids = vector[int]()
        for joint in robot[str(chain)]:
            chain_ids.push_back(int(joint["id"]))
        chain_template.push_back(vector[int](chain_ids))

    #init the joint vector
    cdef jointvector joint_vector
    cdef Joint temp_joint
    cdef int idx = 0
    for idx in range(len(joints)):
        temp_joint = joints.__getitem__(idx)
        joint_vector.push_back(deref(temp_joint.get_joint()))

    #init _Joint Id Mapping
    cdef map[string, int] joint_mapping = map[string, int](), chain_mapping = map[string, int]()
    idx = 0
    for idx in range(len(joints)):
        temp_joint = joints[idx]
        joint_mapping[temp_joint.get_name()] = temp_joint.get_id()

    #init Chain Id Mapping
    idx = 0
    for chain in robot["ChainNames"]:
        chain_mapping[chain] = idx
        idx = idx + 1

    return new _Robot(joint_mapping, chain_mapping, joint_vector, chain_template, max_id)

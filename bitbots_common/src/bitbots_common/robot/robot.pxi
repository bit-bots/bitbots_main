
cdef class Robot:
    def __cinit__(self, bool initialize_empty=False):
        self.is_initialized_empty = initialize_empty
        if initialize_empty is True:
            self.robot = NULL
            return
        cdef object config = get_config()
        try:
            self.robot = create_robot_from_config(config)
        except BaseException, e:
            print e
            #exit()
            raise e

        # Mit einer leeren Pose initialisieren
        cdef Pose zero
        self.robot.update(zero)

    def __dealloc__(self):
        if not self.is_initialized_empty:
            del self.robot

    cdef const _Joint* get_c_joint_by_id(self, int id):
        return ref(self.robot.get_joint_by_id(id))
    cdef const _Joint* get_c_joint_by_name(self, string name):
        return ref(self.robot.get_joint_by_name(name))
    cpdef Joint get_joint_by_id(self, int id):
        cdef Joint joint = Joint(str(id))
        joint.set_joint(self.robot.get_joint_by_id(id))
        return joint
    cpdef Joint get_joint_by_name(self, string name):
        cdef Joint joint = Joint(name)
        joint.set_joint(self.robot.get_joint_by_name(name))
        return joint

    cpdef debugprint(self):
        self.robot.print_robot_data_for_debug()

    cpdef update(self, PyPose pose):
        self.robot.update(deref(pose.pose))

    cpdef update_masses(self):
        self.robot.update_robot_masses()

    cpdef np.ndarray get_centre_of_gravity(self, bool with_mass=False):
        cdef Vector4d ep = self.robot.get_centre_of_gravity()
        cdef np.ndarray vec = np.array((ep.x(), ep.y(), ep.z(), ep.at(3)), dtype=np.float32)
        vec[0:3] = vec[0:3] * 1000
        if not with_mass:
            vec[3] = 1
        return vec

    cpdef np.ndarray get_centre_of_gravity_with_offset(self, np.ndarray of):
        of.reshape(4,4)
        cdef Matrix4d offset_m

        offset_m.insert(<double>of[0,0]).add(<double>of[0,1]).add(<double>of[0,2]).add(<double>of[0,3]) \
                   .add(<double>of[1,0]).add(<double>of[1,1]).add(<double>of[1,2]).add(<double>of[1,3]) \
                   .add(<double>of[2,0]).add(<double>of[2,1]).add(<double>of[2,2]).add(<double>of[2,3]) \
                   .add(<double>of[3,0]).add(<double>of[3,1]).add(<double>of[3,2]).add(<double>of[3,3])
        cdef Vector4d ep = self.robot.get_centre_of_gravity(offset_m)
        cdef np.ndarray vec = np.array((ep.x(), ep.y(), ep.z(), ep.at(3)), dtype=np.float32)
        vec[0:3] = vec[0:3] * 1000
        return vec

    cpdef float get_mass(self):
        return self.robot.get_mass()

    cpdef update_with_matrix(self, np.ndarray ndarr):
        if ndarr.ndim != 1 or ndarr.shape[0] != 20 or ndarr.dtype != np.float32:
            raise ValueError("Invalid joint Matrix")

        cdef arr = np.array(ndarr, dtype=np.float32)
        cdef double *data = <double*>np.PyArray_BYTES(arr)
        self.robot.update(<VectorXd>MapVectorXd(data, 20))

    cpdef PyPose set_angles_to_pose(self, PyPose pose, int chain_id=-1, float time=0):
        self.robot.set_angles_to_pose(deref(pose.pose), chain_id, time)
        return pose

    cpdef set_initial_angles(self, float roll, float pitch, float yaw):
        self.robot.set_initial_angles(Vector3d(roll, pitch, yaw))

    cpdef reset_initial_angles(self):
        self.robot.reset_initial_angles()

    cpdef Joint get_l_foot_endpoint(self):
        cdef Joint joint = Joint("LFootEndpoint")
        joint.set_joint(self.robot.get_joint_by_id(LFoot))
        return joint

    cpdef Joint get_r_foot_endpoint(self):
        cdef Joint joint = Joint("RFootEndpoint")
        joint.set_joint(self.robot.get_joint_by_id(RFoot))
        return joint

    cpdef Joint get_l_arm_endpoint(self):
        cdef Joint joint = Joint("LArmEndpoint")
        joint.set_joint(self.robot.get_joint_by_id(LArm))
        return joint

    cpdef Joint get_r_arm_endpoint(self):
        cdef Joint joint = Joint("RArmEndpoint")
        joint.set_joint(self.robot.get_joint_by_id(RArm))
        return joint

    cpdef Joint get_camera(self):
        cdef Joint joint = Joint("Camera")
        joint.set_joint(self.robot.get_joint_by_id(Cam))
        return joint

    cpdef Joint get_center(self):
        cdef Joint joint = Joint("Root")
        joint.set_joint(self.robot.get_joint_by_id(Center))
        return joint

    cdef list chain_to_list(self, const_Chain& chain):
        cdef list result = []
        cdef Joint joint
        cdef const_ChainIterator it = chain.begin()

        while it != chain.end():
            joint = Joint("")
            joint.set_joint(deref(it).get())
            result.append(joint)
            it += 1

        return result

    cpdef list get_l_arm_chain(self):
        return self.chain_to_list(self.robot.get_chain_by_id(LArmC))

    cpdef list get_l_leg_chain(self):
        return self.chain_to_list(self.robot.get_chain_by_id(LLegC))

    cpdef list get_r_arm_chain(self):
        return self.chain_to_list(self.robot.get_chain_by_id(RLegC))

    cpdef list get_r_leg_chain(self):
        return self.chain_to_list(self.robot.get_chain_by_id(LLegC))

    cpdef list get_head_chain(self):
        return self.chain_to_list(self.robot.get_chain_by_id(HeadC))

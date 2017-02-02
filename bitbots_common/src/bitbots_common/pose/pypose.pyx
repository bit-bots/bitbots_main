import math
import rospy
from cython.operator cimport address as ref
from cython.operator cimport dereference as deref

from libcpp.string cimport string
from libcpp.vector cimport vector

cpdef int num_pose_joints():
    return get_num_joints()

cdef class PyJoint:
    cdef set_joint(self, PyPose pose, Joint* joint):
        self.pose = pose
        self.joint = joint

    property position:
        """
        Aktueller Winkel des Gelenks. Ändert man diesen, ändert sich nicht
        der wert von :attr:`changed`.
        """
        def __get__(self): return self.joint.get_position()
        def __set__(self, value):
            self.joint.set_position(value)

    property speed:
        """
        Geschwindigkeit des Gelenks ins Grad pro Sekunde, mit der
        das Ziel :attr:`goal` angefahren werden soll.

        Setzt auch :attr:`changed` auf *true*.
        """
        def __get__(self): return self.joint.get_speed()
        def __set__(self, value): self.joint.set_speed(value)

    property goal:
        """
        Zielwinkel des Gelenks. Setzt das Gelenk automatisch auf Aktiv.

        Setzen eines neuen Ziel-Wertes führt dazu, dass :attr:`changed`
        und :attr:`active` den Wert *true* bekommen.
        """
        def __get__(self): return self.joint.get_goal()
        def __set__(self, value):
            """
            Beachtenb das es den Setter unten noc hmal als funktion gibt!
            """
            if self.joint.set_goal(value):
                return
            raise ValueError("Invalid Vaulue %d for Motor %d: Possible range: %d to %d" % (
                int(value),
                self.cid,
                self.joint.get_minimum(),
                self.joint.get_maximum()))

    property active:
        """
        Gibt an, ob der Motor des Gelenks aktiviert werden
        soll oder nicht. Er ist locker, wenn er nicht aktiviert ist.
        """
        def __get__(self): return self.joint.is_active()
        def __set__(self, value): self.joint.set_active(value)

    property changed:
        """
        Dieses Property gibt an, ob der Gelenkwinkel geändert wurde.
        Es kann aus Python heraus nicht gesetzt werden. Soll dieses Flag
        doch geändert werden, so muss auf die entsprechende
        Cython-Methode :func:`reset` zurückgegriffen werden.
        """
        def __get__(self): return self.joint.has_changed()

    property load:
        """
        Gibt an, ob Last auf dem Motor ist. Der Wert ist nicht zuverlässig,
        es sollte nur das Vorzeichen beachtet werden.
        """
        def __get__(self): return self.joint.get_load()
        def __set__(self, value): self.joint.set_load(value)

    property maximum:
        """
        Maximalwinkel des Motors
        """
        def __get__(self): return self.joint.get_maximum()
        def __set__(self, value): self.joint.set_maximum(value)

    property minimum:
        """
        Minimalwinkel des Motors
        """
        def __get__(self): return self.joint.get_minimum()
        def __set__(self, value): self.joint.set_minimum(value)

    property p:
        """
        Minimalwinkel des Motors
        """
        def __get__(self): return self.joint.get_p()
        def __set__(self, value): self.joint.set_p(value)

    property i:
        """
        Minimalwinkel des Motors
        """
        def __get__(self): return self.joint.get_i()
        def __set__(self, value): self.joint.set_i(value)

    property d:
        """
        Minimalwinkel des Motors
        """
        def __get__(self): return self.joint.get_d()
        def __set__(self, value): self.joint.set_d(value)

    property cid:
        """ Die ID des Gelenks. """
        def __get__(self): return self.joint.get_cid()

    property reset:
        """
        wenn True resettet die gelenke
        Leider notwendiger ugly hack da reset() aus der src
        nicht aufgeruffen werden kann
        """
        def __set__(self,value):
            if value:
                self.joint.reset()

    def __repr__(self):
        return "<Joint cid=%d, active=%s, pos=%1.1f°, goal=%1.1f°, speed=%1.1f>°" % (
            self.cid,
            "yes" if self.active else "no",
            self.position, self.goal, self.speed)

    cpdef set_active(self, bool active):
        self.joint.set_active(active)

    cpdef reset(self):
        self.joint.reset()

    cpdef set_goal(self, float goal):
        """
        Beachtenb das es den Setter unten noc hmal als funktion gibt!
        """
        if self.joint.set_goal(goal):
            return
        raise ValueError("Invalid Vaulue %f for Motor %d" % (goal, self.cid))

    cpdef set_speed(self, float speed):
        self.joint.set_speed(speed)

    cpdef set_position(self, float position):
        self.joint.set_position(position)

    cpdef set_load(self, float load):
        self.joint.set_load(load)

    cpdef set_temperature(self, float temp):
        self.joint.set_temperature(temp)

    cpdef set_voltage(self, float volt):
        self.joint.set_voltage(volt)

    cpdef set_minimum(self, float minimum):
        self.joint.set_minimum(minimum)

    cpdef set_maximum(self, float maximum):
        self.joint.set_maximum(maximum)

    cpdef set_p(self, int p):
        self.joint.set_p(p)

    cpdef set_i(self, int i):
        self.joint.set_i(i)

    cpdef set_d(self, int d):
        self.joint.set_d(d)

    cpdef bool is_active(self):
        return self.joint.is_active()

    cpdef bool has_changed(self):
        return self.joint.has_changed()

    cpdef float get_goal(self):
        return self.joint.get_goal()

    cpdef float get_speed(self):
        return self.joint.get_speed()

    cpdef float get_position(self):
        return self.joint.get_position()

    cpdef float get_load(self):
        return self.joint.get_load()

    cpdef float get_temperature(self):
        return self.joint.get_temperature()

    cpdef float get_voltage(self):
        return self.joint.get_voltage()

    cpdef float get_maximum(self):
        return self.joint.get_maximum()

    cpdef float get_minimum(self):
        return self.joint.get_minimum()

    cpdef int get_p(self):
        return self.joint.get_p()

    cpdef int get_i(self):
        return self.joint.get_i()

    cpdef int get_d(self):
        return self.joint.get_d()

    cpdef int get_cid(self):
        return self.joint.get_cid()


cdef PyJoint wrap_joint(PyPose pose, Joint& j):
    cdef PyJoint joint = PyJoint()
    joint.set_joint(pose, ref(j))
    return joint

cdef PyJoint wrap_joint_ptr(PyPose pose, Joint* j):
    cdef PyJoint joint = PyJoint()
    joint.set_joint(pose, j)
    return joint

cdef class PyPose:
    def __cinit__(self):
        self.is_reference = False
        self.pose = new Pose()

    def __dealloc__(self):
        if not self.is_reference:
            del self.pose

    cdef set_c_pose(self, Pose& pose):
        if self.is_reference:
            self.pose = new Pose()
            self.is_reference = False

        self.pose.copy(pose)

    cdef set_c_pose_ref(self, Pose& pose):
        if not self.is_reference:
            del self.pose

        self.pose = ref(pose)
        self.is_reference = True

    cdef Pose* get_pose_ptr(self):
        return self.pose

    cpdef PyJoint get_joint(self, bytes pyname):
        return wrap_joint_ptr(self, self.pose.get_joint_ptr(string(<char*>pyname)))

    cpdef PyJoint get_joint_by_cid(self, int cid):
        return wrap_joint_ptr(self, self.pose.get_joint_by_cid(cid))

    cpdef list get_positions(self):
        cdef vector[string] names = self.pose.get_joint_names()
        cdef list result =[]
        for name in names:
            result.append(self.get_joint(name).get_position())
        return result

    cpdef list get_positions_rad(self):
        cdef vector[string] names = self.pose.get_joint_names()
        cdef list result =[]
        for name in names:
            result.append(math.radians(self.get_joint(name).get_position()))
        return result

    cpdef list get_speeds(self):
        cdef vector[string] names = self.pose.get_joint_names()
        cdef list result =[]
        for name in names:
            result.append(self.get_joint(name).get_speed())
        return result

    cpdef list get_goals(self):
        cdef int i = 0
        cdef vector[string] names = self.pose.get_joint_names()
        cdef list result =[]
        for name in names:
            result.append(self.get_joint(name).get_goal())
        return result

    cpdef list get_loads(self):
        cdef int i = 0
        cdef vector[string] names = self.pose.get_joint_names()
        cdef list result =[]
        for name in names:
            result.append(self.get_joint(name).get_load())
        return result

    cpdef set_positions(self, list names, list positions):
        cdef int i = 0
        cdef list result =[]
        cdef string name
        if len(names) != len(positions):
            raise AssertionError("Length of names and positions does not match.")
        for name in names:
            self.get_joint(name).set_position(positions[i])
            i += 1

    cpdef set_speeds(self, list names, list speeds):
        cdef int i = 0
        cdef list result =[]
        cdef string name
        if len(names) != len(speeds):
            raise AssertionError("Length of names and speeds does not match.")
        for name in names:
            self.get_joint(name).set_speed(speeds[i])
            i += 1

    cpdef set_goals(self, list names, list goals):
        cdef int i = 0
        cdef list result =[]
        cdef string name
        if len(names) != len(goals):
            raise AssertionError("Length of names and goals does not match.")
        for name in names:
            self.get_joint(name).set_goal(goals[i])
            i += 1

    cpdef set_loads(self, list names, list loads):
        cdef int i = 0
        cdef list result =[]
        cdef string name
        if len(names) != len(loads):
            raise AssertionError("Length of names and goals does not match.")
        for name in names:
            self.get_joint(name).set_load(loads[i])
            i += 1

    def __getitem__(self, name):
        return self.get_joint(name)

    def __iter__(self):
        return iter(self.joints)

    def copy(self):
        """ Erzeugt eine Kopie dieser Pose """
        return wrap_pose(deref(self.pose))

    def get_joint_names(self):
        cdef int i
        cdef vector[string] names = self.pose.get_joint_names()

        cdef list result = []
        for i in range(names.size()):
            result.append(<bytes>names[i].c_str())

        return result

    cpdef string get_joint_name(self, int cid):
        cdef vector[string] names = self.pose.get_joint_names()
        return names[cid]

    property names:
        """ Liste mit den Namen aller Gelenke. """
        def __get__(self):
            return self.get_joint_names()

    property joints:
        def __get__(self):
            cdef int i
            cdef vector[string] names = self.pose.get_joint_names()
            cdef vector[Joint*] joints = self.pose.get_joints()

            cdef list result = []
            for i in range(names.size()):
                tup = (<bytes>names[i].c_str(), wrap_joint_ptr(self, joints[i]))
                result.append(tup)

            return result

    property positions:
        def __get__(self):
            cdef int i
            cdef vector[string] names = self.pose.get_joint_names()
            cdef vector[Joint*] joints = self.pose.get_joints()

            cdef list result = []
            for i in range(names.size()):
                tup = (<bytes>names[i].c_str(), joints[i].get_position())
                result.append(tup)

            return result

        def __set__(self, values):
            cdef bytes name
            cdef float value
            for name, value in values:
                self.pose.get_joint_ptr(string(<char*>name)).set_position(value)

    property goals:
        def __get__(self):
            cdef int i
            cdef vector[string] names = self.pose.get_joint_names()
            cdef vector[Joint*] joints = self.pose.get_joints()

            cdef list result = []
            for i in range(names.size()):
                tup = (<bytes>names[i].c_str(), joints[i].get_goal())
                result.append(tup)

            return result

        def __set__(self, values):
            cdef bytes name
            cdef float value
            for name, value in values:
                self.pose.get_joint_ptr(string(<char*>name)).set_goal(value)

    def set_active(self, bool ena):
        """ Setzt das :attr:`~PyJoint.active`-Attribute aller
        Gelenke auf *true*.

        Methode um alle Gelenke auf einmal zu aktivieren oder zu deaktivieren.
        Nur die :attr:`~PyJoint.goal`-Werte von aktivierten Gelenken werden von den
        Motoren beachtet.

        Ein Gelenk wird jedoch auch automatisch auf *active* gestellt, wenn
        sein :attr:`~PyJoint.goal`-Wert geändert wird.
        """
        cdef vector[Joint*] joints = self.pose.get_joints()
        for i in range(joints.size()):
            joints[i].set_active(ena)

    cpdef reset(self):
        self.pose.reset()

    cpdef update(self, PyPose other):
        """ Übernimmt die :attr:`~PyJoint.goal`, :attr:`~PyJoint.speed` und
            :attr:`~PyJoint.active` Werte aller Gelenke aus der anderen
            Pose, für die :attr:`~PyJoint.changed` gesetzt ist.
        """
        self.pose.update(deref(other.pose))

    cpdef update_positions(self, PyPose other):
        """ Updated die Werte von :attr:`~PyJoint.position`
            und :attr:`~PyJoint.load`.
        """
        self.pose.update_positions(deref(other.pose))

    property r_shoulder_pitch:
        def __get__(self):
            return wrap_joint(self, self.pose.get_r_shoulder_pitch())

    property l_shoulder_pitch:
        def __get__(self):
            return wrap_joint(self, self.pose.get_l_shoulder_pitch())

    property r_shoulder_roll:
        def __get__(self):
            return wrap_joint(self, self.pose.get_r_shoulder_roll())

    property l_shoulder_roll:
        def __get__(self):
            return wrap_joint(self, self.pose.get_l_shoulder_roll())

    property r_elbow:
        def __get__(self):
            return wrap_joint(self, self.pose.get_r_elbow())

    property l_elbow:
        def __get__(self):
            return wrap_joint(self, self.pose.get_l_elbow())

    property r_hip_yaw:
        def __get__(self):
            return wrap_joint(self, self.pose.get_r_hip_yaw())

    property l_hip_yaw:
        def __get__(self):
            return wrap_joint(self, self.pose.get_l_hip_yaw())

    property r_hip_roll:
        def __get__(self):
            return wrap_joint(self, self.pose.get_r_hip_roll())

    property l_hip_roll:
        def __get__(self):
            return wrap_joint(self, self.pose.get_l_hip_roll())

    property r_hip_pitch:
        def __get__(self):
            return wrap_joint(self, self.pose.get_r_hip_pitch())

    property l_hip_pitch:
        def __get__(self):
            return wrap_joint(self, self.pose.get_l_hip_pitch())

    property r_knee:
        def __get__(self):
            return wrap_joint(self, self.pose.get_r_knee())

    property l_knee:
        def __get__(self):
            return wrap_joint(self, self.pose.get_l_knee())

    property r_ankle_pitch:
        def __get__(self):
            return wrap_joint(self, self.pose.get_r_ankle_pitch())

    property l_ankle_pitch:
        def __get__(self):
            return wrap_joint(self, self.pose.get_l_ankle_pitch())

    property r_ankle_roll:
        def __get__(self):
            return wrap_joint(self, self.pose.get_r_ankle_roll())

    property l_ankle_roll:
        def __get__(self):
            return wrap_joint(self, self.pose.get_l_ankle_roll())

    property head_pan:
        def __get__(self):
            return wrap_joint(self, self.pose.get_head_pan())

    property head_tilt:
        def __get__(self):
            return wrap_joint(self, self.pose.get_head_tilt())

    property l_hand:
        def __get__(self):
            return wrap_joint(self, self.pose.get_l_hand())

    property r_hand:
        def __get__(self):
            return wrap_joint(self, self.pose.get_r_hand())

    property l_elbow_roll:
        def __get__(self):
            return wrap_joint(self, self.pose.get_l_elbow_roll())

    property r_shoulder_yaw:
        def __get__(self):
            return wrap_joint(self, self.pose.get_r_shoulder_yaw())

    property l_shoulder_yaw:
        def __get__(self):
            return wrap_joint(self, self.pose.get_l_shoulder_yaw())

    property belly_roll:
        def __get__(self):
            return wrap_joint(self, self.pose.get_belly_roll())

    property belly_pitch:
        def __get__(self):
            return wrap_joint(self, self.pose.get_belly_pitch())

    property r_toe:
        def __get__(self):
            return wrap_joint(self, self.pose.get_belly_pitch())

    property l_toe:
        def __get__(self):
            return wrap_joint(self, self.pose.get_belly_pitch())

cdef PyPose wrap_pose(Pose& p):
    cdef PyPose pose = PyPose()
    pose.set_c_pose(p)
    return pose

cdef PyPose wrap_pose_obj(Pose p):
    cdef PyPose pose = PyPose()
    pose.set_c_pose(p)
    return pose

cdef PyPose wrap_pose_ref(Pose& p):
    cdef PyPose pose = PyPose()
    pose.set_c_pose_ref(p)
    return pose
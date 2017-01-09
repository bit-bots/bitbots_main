cdef enum:
    R_SHOULDER_PITCH    = 0
    L_SHOULDER_PITCH    = 1
    R_SHOULDER_ROLL     = 2
    L_SHOULDER_ROLL     = 3
    R_ELBOW             = 4
    L_ELBOW             = 5
    R_HIP_YAW           = 6
    L_HIP_YAW           = 7
    R_HIP_ROLL          = 8
    L_HIP_ROLL          = 9
    R_HIP_PITCH         = 10
    L_HIP_PITCH         = 11
    R_KNEE              = 12
    L_KNEE              = 13
    R_ANKLE_PITCH       = 14
    L_ANKLE_PITCH       = 15
    R_ANKLE_ROLL        = 16
    L_ANKLE_ROLL        = 17
    HEAD_YAW            = 18
    HEAD_PITCH          = 19
    NUMBER_OF_SERVOS    = 20


cpdef berechneInverseKinematics(PyPose pose, list Left, list Right, list Oberkoerper):
    cdef _Coordinates LLeft
    LLeft.x = float(Left[0])
    LLeft.y = float(Left[1])
    LLeft.z = float(Left[2])
    LLeft.alpha = float(Left[3])
    LLeft.beta = float(Left[4])
    LLeft.gamma = float(Left[5])
    cdef _Coordinates LRight
    LRight.x = float(Right[0])
    LRight.y = float(Right[1])
    LRight.z = float(Right[2])
    LRight.alpha = float(Right[3])
    LRight.beta = float(Right[4])
    LRight.gamma = float(Right[5])
    cdef _Coordinates LOberkoerper
    LOberkoerper.x = float(Oberkoerper[0])
    LOberkoerper.y = float(Oberkoerper[1])
    LOberkoerper.z = float(Oberkoerper[2])
    LOberkoerper.alpha = float(Oberkoerper[3])
    LOberkoerper.beta = float(Oberkoerper[4])
    LOberkoerper.gamma = float(Oberkoerper[5])
    cdef vector[float] motors
    motors = berechneIK(LLeft,LRight,LOberkoerper)

    pose.r_hip_yaw.goal = motors[R_HIP_YAW] / 3.1415926535 * 180
    pose.l_hip_yaw.goal = motors[L_HIP_YAW]/ 3.1415926535 * 180
    pose.r_hip_roll.goal = - motors[R_HIP_ROLL]/ 3.1415926535 * 180
    pose.l_hip_roll.goal = - motors[L_HIP_ROLL]/ 3.1415926535 * 180
    pose.r_hip_pitch.goal = motors[R_HIP_PITCH]/ 3.1415926535 * 180
    pose.l_hip_pitch.goal = - motors[L_HIP_PITCH]/ 3.1415926535 * 180
    pose.r_knee.goal =  motors[R_KNEE]/ 3.1415926535 * 180
    pose.l_knee.goal = - motors[L_KNEE]/ 3.1415926535 * 180
    pose.r_ankle_pitch.goal = -motors[R_ANKLE_PITCH]/ 3.1415926535 * 180
    pose.l_ankle_pitch.goal = motors[L_ANKLE_PITCH]/ 3.1415926535 * 180
    pose.l_ankle_roll.goal = motors[L_ANKLE_ROLL]/ 3.1415926535 * 180
    pose.r_ankle_roll.goal = motors[R_ANKLE_ROLL]/ 3.1415926535 * 180


cpdef berechneDirekteKinematic(PyPose pose):
    cdef  vector[float] motors

    for i in range(30):
        motors.push_back(0)

    motors[R_HIP_YAW] = pose.r_hip_yaw.goal / 180.0 * 3.1415926535
    motors[L_HIP_YAW] = pose.l_hip_yaw.goal / 180.0 * 3.1415926535
    motors[R_HIP_ROLL] = - pose.r_hip_roll.goal / 180.0 * 3.1415926535
    motors[L_HIP_ROLL] = - pose.l_hip_roll.goal / 180.0 * 3.1415926535
    motors[R_HIP_PITCH] = pose.r_hip_pitch.goal / 180.0 * 3.1415926535
    motors[L_HIP_PITCH] = - pose.l_hip_pitch.goal / 180.0 * 3.1415926535
    motors[R_KNEE] =  pose.r_knee.goal / 180.0 * 3.1415926535
    motors[L_KNEE] = -  pose.l_knee.goal / 180.0 * 3.1415926535
    motors[R_ANKLE_PITCH] = -  pose.r_ankle_pitch.goal / 180.0 * 3.1415926535
    motors[L_ANKLE_PITCH] = pose.l_ankle_pitch.goal / 180.0 * 3.1415926535
    motors[L_ANKLE_ROLL] = pose.l_ankle_roll.goal / 180.0 * 3.1415926535
    motors[R_ANKLE_ROLL] = pose.r_ankle_roll.goal / 180.0 * 3.1415926535

    cdef  _Coordinates coordL
    coordL = calculateLegDirektKinematics(motors, True)
    cdef  _Coordinates coordR

    coordR = calculateLegDirektKinematics(motors, False)

    return [coordL.x, coordL.y, coordL.z, coordL.alpha, coordL.beta, coordL.gamma],[coordR.x, coordR.y, coordR.z, coordR.alpha, coordR.beta, coordR.gamma]
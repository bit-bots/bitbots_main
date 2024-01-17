import math
from bitbots_moveit_bindings import get_bioik_ik, check_collision
from bio_ik_msgs.msg import IKRequest, LookAtGoal
from sensor_msgs.msg import JointState


def test_get_ik():
    r = IKRequest()
    r.group_name = 'Head'
    r.timeout.sec = 1
    r.approximate = True
    r.look_at_goals.append(LookAtGoal())
    r.look_at_goals[0].link_name = 'camera'
    r.look_at_goals[0].weight = 1.0
    r.look_at_goals[0].axis.x = 1.0
    r.look_at_goals[0].target.x = 0.5
    r.look_at_goals[0].target.y = 0.3
    r.look_at_goals[0].target.z = -0.4
    bio_ik_response = get_bioik_ik(r)
    names = bio_ik_response.ik_response.solution.joint_state.name
    positions = bio_ik_response.ik_response.solution.joint_state.position
    pan = positions[names.index("HeadPan")]
    tilt = positions[names.index("HeadTilt")]
    assert math.isclose(pan, 0.5321544636603255)
    assert math.isclose(tilt, -0.9626794731747034)


def test_check_collision():
    x = JointState()
    x.name = [
            "HeadPan",
            "HeadTilt",
            "LShoulderPitch",
            "LShoulderRoll",
            "LElbow",
            "RShoulderPitch",
            "RShoulderRoll",
            "RElbow",
            "LHipYaw",
            "LHipRoll",
            "LHipPitch",
            "LKnee",
            "LAnklePitch",
            "LAnkleRoll",
            "RHipYaw",
            "RHipRoll",
            "RHipPitch",
            "RKnee",
            "RAnklePitch",
            "RAnkleRoll"
        ]
    x.position = [
                0.0,  # HeadPan
                0.0,  # HeadTilt
                0.0,  # LShoulderPitch
                0.0,  # LShoulderRoll
                0.79,  # LElbow
                0.0,  # RShoulderPitch
                0.0,  # RShoulderRoll
                -0.79,  # RElbow
                -0.0112,  # LHipYaw
                -0.0615,  # LHipRoll
                0.4732,  # LHipPitch
                1.0058,  # LKnee
                -0.4512,  # LAnklePitch
                0.0625,  # LAnkleRoll
                0.0112,  # RHipYaw
                -0.0615,  # RHipRoll
                -0.4732,  # RHipPitch
                -1.0059,  # RKnee
                0.4512,  # RAnklePitch
                -0.0625,  # RAnkleRoll
            ]

    assert check_collision(x) == False
    x.position[x.name.index("RHipRoll")] = 1
    assert check_collision(x) == True
    x.position[x.name.index("RHipRoll")] = -1
    assert check_collision(x) == False

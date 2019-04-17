#!/usr/bin/env python2.7

import rospy
from geometry_msgs.msg import WrenchStamped, Vector3
from sensor_msgs.msg import JointState

rospy.init_node("torque_viz")

# information from URDF
frame_ids = { "RHipYaw": "r_hip_1",
              "RHipRoll": "r_hip_2",
              "RHipPitch": "r_upper_leg",
              "RKnee": "r_lower_leg",
              "RAnklePitch": "r_ankle",
              "RAnkleRoll": "r_foot",

              "LHipYaw": "l_hip_1",
              "LHipRoll": "l_hip_2",
              "LHipPitch": "l_upper_leg",
              "LKnee": "l_lower_leg",
              "LAnklePitch": "l_ankle",
              "LAnkleRoll": "l_foot",

              "RShoulderPitch": "r_shoulder",
              "RShoulderRoll": "r_upper_arm",
              "RElbow": "r_lower_arm",

              "LShoulderPitch": "l_shoulder",
              "LShoulderRoll": "l_upper_arm",
              "LElbow": "l_lower_arm",

              "HeadPan": "neck",
              "HeadTilt": "head"
}


publisher = {}
for k in frame_ids:
    publisher[k] = rospy.Publisher("/wrenches/" + k, WrenchStamped, queue_size=1)

# information from URDF
axis = { "RHipYaw": [0.0, 0.0, -1.0],
         "RHipRoll": [1.0, 0.0, 0.0],
         "RHipPitch": [0.0, 1.0, 0.0],
         "RKnee": [0.0, 1.0, 0.0],
         "RAnklePitch": [0.0, -1.0, 0.0],
         "RAnkleRoll": [-1.0, 0.0, 0.0],

         "LHipYaw": [0.0, 0.0, -1.0],
         "LHipRoll": [1.0, 0.0, 0.0],
         "LHipPitch": [0.0, -1.0, 0.0],
         "LKnee": [0.0, -1.0, 0.0],
         "LAnklePitch": [0.0, 1.0, 0.0],
         "LAnkleRoll": [-1.0, 0.0, 0.0],

         "RShoulderPitch": [0.0, -1.0, 0.0],
         "RShoulderRoll": [-1.0, 0.0, 0.0],
         "RElbow": [0.0, 1.0, 0.0],

         "LShoulderPitch": [0.0, 1.0, 0.0],
         "LShoulderRoll": [-1.0, 0.0, 0.0],
         "LElbow": [0.0, -1.0, 0.0],

         "HeadPan": [0.0, 0.0, 1.0],
         "HeadTilt": [0.0, -1.0, 0.0]
}


def publish_torques(js_msg):
    """
    publishes rotational (torque) and linear force (linear currently set to 0)
    :param js_msg: JointState message (contains: names, effort, velocity, position)
    :return:
    """
    i = 0
    joints = {}
    for n in js_msg.name:
        joints[n] = [js_msg.position[i], js_msg.velocity[i], js_msg.effort[i]]
        i = i+1

    for key, value in joints.items():
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = rospy.Time.now()
        wrench_msg.header.frame_id = frame_ids[key]

        j_pos = value[0] # not needed at the moment
        j_vel = value[1] # not needed at the moment
        j_eff = value[2]

        a = axis[key]

        vec3_pos = [
            j_pos * a[0],
            j_pos * a[1],
            j_pos * a[2]
        ]

        vec3_eff = [
            j_eff * a[0],
            j_eff * a[1],
            j_eff * a[2]
        ]

        torque = vec3_eff

        vec3_tor = Vector3()
        vec3_tor.x = torque[0]
        vec3_tor.y = torque[1]
        vec3_tor.z = torque[2]

        force = [0, 0, 0]  # set to 0, because not relevant at the moment

        vec3_for = Vector3()
        vec3_for.x = force[0]
        vec3_for.y = force[1]
        vec3_for.z = force[2]

        wrench_msg.wrench.torque = vec3_tor
        wrench_msg.wrench.force = vec3_for

        print(wrench_msg)
        publisher[key].publish(wrench_msg)
    return


rospy.Subscriber("/joint_states", JointState, publish_torques)

rospy.spin()
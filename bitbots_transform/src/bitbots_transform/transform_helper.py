import math

import image_geometry
import rospy
import tf2_ros
import tf_conversions


def transf(x, y, caminfo):
    tfbuffer = tf2_ros.Buffer(cache_time=rospy.Duration(6))
    tfl = tf2_ros.TransformListener(tfbuffer)

    pit = rospy.Time(0)
    trans = tfbuffer.lookup_transform("base_link", "L_CAMERA", pit, rospy.Duration(0.1))


    # Setup camerainfos
    cam = image_geometry.PinholeCameraModel()
    cam.fromCameraInfo(caminfo)
    quaternion = (
        trans.transform.rotation.x,
        trans.transform.rotation.y,
        trans.transform.rotation.z,
        trans.transform.rotation.w)
    euler = tf_conversions.transformations.euler_from_quaternion(quaternion)
    cam_pan = euler[1]
    cam_tilt = euler[2]

    y = y - 600

    angle_horizontal = (x - cam.width / 2.0) / cam.width * 72.0

    angle_vertical = -(y - cam.height / 2.0) / cam.height * 72.0

    angle = cam_pan + angle_horizontal
    distance = 0.70 * abs(math.tan(math.radians(cam_tilt + angle_vertical)) / math.cos(math.radians(angle)))

    return distance * math.cos(math.radians(angle)), distance * math.sin(math.radians(angle)), 0
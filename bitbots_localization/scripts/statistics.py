#!/usr/bin/env python
import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion
from bitbots_localization.msg import Evaluation
import csv
import sys
import math

rospy.init_node('statistics')

tfBuffer = tf2_ros.Buffer(rospy.Duration(30))
listener = tf2_ros.TransformListener(tfBuffer)


def write_header():
    global i
    global name
    header = ['step',
              'resample',
              'line points', 'goal posts', 'field boundary points', 'corners', 'tcrossings', 'crosses',
              'gtx', 'gty', 'gtt',
              'x0.2', 'y0.2', 't0.2',
              'x5', 'y5', 't5',
              'x10', 'y10', 't10',
              'x20', 'y20', 't20',
              'meanx', 'meany', 'meant',
              'cov xx 5', 'cov xy 5', 'cov yy 5', 'cov tt 5',
              'cov xx 10', 'cov xy 10', 'cov yy 10', 'cov tt 10',
              'cov xx 20', 'cov xy 20', 'cov yy 20', 'cov tt 20',
              'cov xx m', 'cov xy m', 'cov yy m', 'cov tt m',
              'dx0.2', 'dy0.2', 'dt0.2',
              'dx5', 'dy5', 'dt5',
              'dx10', 'dy10', 'dt10',
              'dx20', 'dy20', 'dt20',
              'dxmean', 'dymean', 'dtmean']

    with open((name +'_' + str(i)) + '.csv', 'w') as f:
        csv_writer = csv.writer(f)

        csv_writer.writerow(header)  # write header


def write_row():
    global pose
    global name
    try:

        transGT = tfBuffer.lookup_transform('map', 'base_footprint', pose.header.stamp, timeout=rospy.Duration(0.2))
        transB = tfBuffer.lookup_transform('map', 'best_estimate', pose.header.stamp, timeout=rospy.Duration(0.2))
        trans5 = tfBuffer.lookup_transform('map', 'best_estimate_5', pose.header.stamp, timeout=rospy.Duration(0.2))
        transE = tfBuffer.lookup_transform('map', 'localization_estimate', pose.header.stamp, timeout=rospy.Duration(0.2))
        trans20 = tfBuffer.lookup_transform('map', 'best_estimate_20', pose.header.stamp, timeout=rospy.Duration(0.2))
        transM = tfBuffer.lookup_transform('map', 'mean', pose.header.stamp, timeout=rospy.Duration(0.2))

        quaternionGT = (
            transGT.transform.rotation.x,
            transGT.transform.rotation.y,
            transGT.transform.rotation.z,
            transGT.transform.rotation.w)
        eulerGT = euler_from_quaternion(quaternionGT)

        quaternionB = (
            transB.transform.rotation.x,
            transB.transform.rotation.y,
            transB.transform.rotation.z,
            transB.transform.rotation.w)
        eulerB = euler_from_quaternion(quaternionB)

        quaternion5 = (
            trans5.transform.rotation.x,
            trans5.transform.rotation.y,
            trans5.transform.rotation.z,
            trans5.transform.rotation.w)
        euler5 = euler_from_quaternion(quaternion5)

        quaternionE = (
            transE.transform.rotation.x,
            transE.transform.rotation.y,
            transE.transform.rotation.z,
            transE.transform.rotation.w)
        eulerE = euler_from_quaternion(quaternionE)

        quaternion20 = (
            trans20.transform.rotation.x,
            trans20.transform.rotation.y,
            trans20.transform.rotation.z,
            trans20.transform.rotation.w)
        euler20 = euler_from_quaternion(quaternion20)

        quaternionM = (
            transM.transform.rotation.x,
            transM.transform.rotation.y,
            transM.transform.rotation.z,
            transM.transform.rotation.w)
        eulerM = euler_from_quaternion(quaternionM)


        transDxB = transB.transform.translation.x - transGT.transform.translation.x
        transDyB = transB.transform.translation.y - transGT.transform.translation.y
        transDtB = math.atan2(math.sin(eulerB[2] - eulerGT[2]), math.cos(eulerB[2] - eulerGT[2]))

        transDx5 = trans5.transform.translation.x - transGT.transform.translation.x
        transDy5 = trans5.transform.translation.y - transGT.transform.translation.y
        transDt5 = math.atan2(math.sin(euler5[2] - eulerGT[2]), math.cos(euler5[2] - eulerGT[2]))

        transDx = transE.transform.translation.x - transGT.transform.translation.x
        transDy = transE.transform.translation.y - transGT.transform.translation.y
        transDt = math.atan2(math.sin(eulerE[2] - eulerGT[2]), math.cos(eulerE[2] - eulerGT[2]))

        transDx20 = trans20.transform.translation.x - transGT.transform.translation.x
        transDy20 = trans20.transform.translation.y - transGT.transform.translation.y
        transDt20 = math.atan2(math.sin(euler20[2] - eulerGT[2]), math.cos(euler20[2] - eulerGT[2]))

        transDxmean = transM.transform.translation.x - transGT.transform.translation.x
        transDymean = transM.transform.translation.y - transGT.transform.translation.y
        transDtmean = math.atan2(math.sin(eulerM[2] - eulerGT[2]), math.cos(eulerM[2] - eulerGT[2]))



        with open((name + '_' + str(i)) + '.csv', 'a+') as f:
            csv_writer = csv.writer(f)
            row = [pose.header.seq,
                   pose.resampled,
                   pose.lines, pose.goals, pose.fb_points, pose.corners, pose.tcrossings, pose.crosses,
                   round(transGT.transform.translation.x, 2), round(transGT.transform.translation.y, 2), round(eulerGT[2], 2),
                   round(transB.transform.translation.x, 2), round(transB.transform.translation.y, 2), round(eulerB[2], 2),
                   round(trans5.transform.translation.x, 2), round(trans5.transform.translation.y, 2), round(euler5[2], 2),
                   round(transE.transform.translation.x, 2), round(transE.transform.translation.y, 2), round(eulerE[2], 2),
                   round(trans20.transform.translation.x, 2), round(trans20.transform.translation.y, 2), round(euler20[2], 2),
                   round(transM.transform.translation.x, 2), round(transM.transform.translation.y, 2), round(eulerM[2], 2),
                   pose.cov_estimate_5[0], pose.cov_estimate_5[1], pose.cov_estimate_5[7], pose.cov_estimate_5[35],
                   pose.cov_estimate_10[0], pose.cov_estimate_10[1], pose.cov_estimate_10[7], pose.cov_estimate_10[35],
                   pose.cov_estimate_20[0], pose.cov_estimate_20[1], pose.cov_estimate_20[7], pose.cov_estimate_20[35],
                   pose.cov_mean[0], pose.cov_mean[1], pose.cov_mean[7], pose.cov_mean[35],
                   round(transDxB, 2), round(transDyB, 2), round(transDtB, 2),
                   round(transDx5, 2), round(transDy5, 2), round(transDt5, 2),
                   round(transDx, 2), round(transDy, 2), round(transDt, 2),
                   round(transDx20, 2), round(transDy20, 2), round(transDt20, 2),
                   round(transDxmean, 2), round(transDymean, 2), round(transDtmean, 2)]

            csv_writer.writerow(row)


    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("could not look up transform at sequence %s", pose.header.seq)



def pose_cb(msg):
    global pose
    pose = msg
    write_row()


if __name__ == '__main__':

    pose_sub = rospy.Subscriber("/pose", Evaluation, pose_cb, queue_size=20)
    global i
    i = int(sys.argv[1])
    name = '/homes/12hartfil/Dokumente/data/experiment_walking_3/lines_goals_fb'


    while not rospy.is_shutdown():
        try:
            write_header()
            rospy.spin()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            rospy.logwarn(
                "We moved backwards in time. I hope you just resetted the simulation. If not there is something wrong")
        except rospy.exceptions.ROSInterruptException:
            exit()

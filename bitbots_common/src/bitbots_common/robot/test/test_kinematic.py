import numpy
from bitbots.robot.kinematics import Robot
from bitbots.ipc.ipc import SharedMemoryIPC

ipc = SharedMemoryIPC()
robot = Robot()

robot.update(ipc.get_pose())
if False:
    """
    print 31
    print numpy.dot(robot.get_joint_by_id(34).get_chain_matrix(inverse=True) , robot.get_joint_by_id(31).get_chain_matrix() )
    print 36
    print numpy.dot(robot.get_joint_by_id(34).get_chain_matrix(inverse=True) , robot.get_joint_by_id(36).get_chain_matrix() )
    """
    print 7
    print robot.get_joint_by_id(7).get_chain_matrix()
    print 9
    print robot.get_joint_by_id(9).get_chain_matrix()
    print 11
    print robot.get_joint_by_id(11).get_chain_matrix()
    print 13
    print robot.get_joint_by_id(13).get_chain_matrix()
    print 15
    print robot.get_joint_by_id(15).get_chain_matrix()
    print 17
    print robot.get_joint_by_id(17).get_chain_matrix()
    print 34
    print robot.get_joint_by_id(34).get_chain_matrix()

    print 19
    print robot.get_joint_by_id(19).get_chain_matrix()
    print 20
    print robot.get_joint_by_id(20).get_chain_matrix()
    print 31
    print robot.get_joint_by_id(31).get_chain_matrix()
    print "Gobal"
    print 19
    print numpy.dot(robot.get_joint_by_id(34).get_chain_matrix(inverse=True),
                    robot.get_joint_by_id(19).get_chain_matrix())
    print 20
    print numpy.dot(robot.get_joint_by_id(34).get_chain_matrix(inverse=True),
                    robot.get_joint_by_id(20).get_chain_matrix())
    print 31
    print numpy.dot(robot.get_joint_by_id(34).get_chain_matrix(inverse=True),
                    robot.get_joint_by_id(31).get_chain_matrix())

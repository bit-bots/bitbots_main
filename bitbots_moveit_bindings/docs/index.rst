Welcome to |project|'s documentation!
================================================

This package allows to make calls to the MoveIt IK and FK from Python.
Here is an example code snippet for using the IK:

  .. code-block:: Python

    from bitbots_moveit_bindings import get_position_ik
    from moveit_msgs.srv import GetPositionIKRequest, GetPositionIKResponse
    from tf.transformations import quaternion_from_euler

    # create request
    request = GetPositionIKRequest()
    # configure request
    request.ik_request.timeout = rospy.Time.from_seconds(0.01)
    request.ik_request.attempts = 1
    request.ik_request.avoid_collisions = False

    # specify the goal
    request.ik_request.group_name = "LeftLeg"
    request.ik_request.ik_link_name = "l_sole"
    request.ik_request.pose_stamped.pose.position = Point(0,0,-0.35)
    request.ik_request.pose_stamped.pose.orientation = Quaternion(*quaternion_from_euler(0,0,0))

    # call the IK
    ik_result = get_position_ik(request, approximate=False)

    # you get the results as a joint state message
    results_as_joint_state_msg = ik_result.solution.joint_state

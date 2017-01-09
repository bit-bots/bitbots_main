def set_joint_state_on_pose(joint_state, pose):
    pose.setPositions(joint_state.positions)
    pose.setSpeed(joint_state.velocities)
    return pose

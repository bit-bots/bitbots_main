def set_joint_state_on_pose(joint_state, pose):
    pose.set_positions(joint_state.positions)
    pose.set_speeds(joint_state.velocities)
    return pose

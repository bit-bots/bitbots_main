#include <bitbots_odometry/odometry_fuser.h>

/*
odom -> baselink
walking (X, Y, Z, rZ)
imu (rX, rY)
*/


// TODO Doku

OdometryFuser::OdometryFuser() : tf_listener_(tf_buffer_), support_state_cache_(100) {
  ros::NodeHandle n("");
  ros::NodeHandle pnh("~");
  pnh.param<std::string>("base_link_frame", base_link_frame_, "base_link");
  pnh.param<std::string>("r_sole_frame", r_sole_frame_, "r_sole");
  pnh.param<std::string>("l_sole_frame", l_sole_frame_, "l_sole");
  pnh.param<std::string>("odom_frame", odom_frame_, "odom");
  pnh.param<std::string>("rotation_frame", rotation_frame_, "rotation");
  pnh.param<std::string>("imu_frame", imu_frame_, "imu_frame");

  auto callback = [&](const bitbots_msgs::SupportState::ConstPtr &msg){support_state_cache_.add(msg);};

  ros::Subscriber walk_support_state_sub = n.subscribe<bitbots_msgs::SupportState>(
    "walk_support_state", 1, callback);

  ros::Subscriber kick_support_state_sub = n.subscribe<bitbots_msgs::SupportState>(
    "dynamic_kick_support_state", 1, callback);

  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(n, "imu/data", 1);
  message_filters::Subscriber<nav_msgs::Odometry> motion_odom_sub(n, "motion_odometry", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, nav_msgs::Odometry> SyncPolicy;

  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(50), imu_sub, motion_odom_sub);

  sync.registerCallback(&OdometryFuser::imuCallback, this);

  geometry_msgs::TransformStamped tf;

  static tf2_ros::TransformBroadcaster br;

  // This specifies the throttle of error messages
  float msg_rate = 10.0;
  // wait till connection with publishers has been established
  // so we do not immediately blast something the log output
  ros::Duration(0.5).sleep();

  // wait for transforms from joints
  while (!tf_buffer_.canTransform(l_sole_frame_, base_link_frame_, ros::Time(0), ros::Duration(1)) && ros::ok()) {
    ROS_WARN_THROTTLE(30, "Waiting for transforms from robot joints");
  }

  ros::Rate r(500.0);
  ros::Time last_time_stamp;
  while (ros::ok()) {
    ros::spinOnce();

    // in simulation, the time does not always advance between loop iteration
    // in that case, we do not want to republish the transform
    if (fused_time_ == last_time_stamp) {
      r.sleep();
      continue;
    }

    // get roll an pitch from imu
    tf2::Quaternion imu_orientation;
    tf2::fromMsg(imu_data_.orientation, imu_orientation);

    // get motion_odom transform
    tf2::Transform motion_odometry;
    tf2::fromMsg(odom_data_.pose.pose, motion_odometry);

    // combine orientations to new quaternion if IMU is active, use purely odom otherwise
    tf2::Transform fused_odometry;

    // compute the point of rotation (in base_link frame)
    tf2::Transform rotation_point_in_base = getCurrentRotationPoint();
    // publish rotation point as debug
    geometry_msgs::TransformStamped rotation_point_msg;
    rotation_point_msg.header.stamp = fused_time_;
    rotation_point_msg.header.frame_id = base_link_frame_;
    rotation_point_msg.child_frame_id = rotation_frame_;
    geometry_msgs::Transform rotation_point_transform_msg;
    rotation_point_transform_msg = tf2::toMsg(rotation_point_in_base);
    rotation_point_msg.transform = rotation_point_transform_msg;
    br.sendTransform(rotation_point_msg);
    // get base_link in rotation point frame
    tf2::Transform base_link_in_rotation_point = rotation_point_in_base.inverse();

    // get only translation and yaw from motion odometry
    tf2::Quaternion odom_orientation_yaw = getCurrentMotionOdomYaw(
      motion_odometry.getRotation());
    tf2::Transform motion_odometry_yaw;
    motion_odometry_yaw.setRotation(odom_orientation_yaw);
    motion_odometry_yaw.setOrigin(motion_odometry.getOrigin());

    // Get the rotation offset between the IMU and the baselink
    tf2::Transform imu_mounting_offset;
    try {
      geometry_msgs::TransformStamped imu_mounting_transform = tf_buffer_.lookupTransform(
        base_link_frame_, imu_frame_, fused_time_);
      fromMsg(imu_mounting_transform.transform, imu_mounting_offset);
    } catch (tf2::TransformException &ex) {
      ROS_ERROR("Not able to use the IMU%s", ex.what());
    }

    // get imu transform without yaw
    tf2::Quaternion imu_orientation_without_yaw_component = getCurrentImuRotationWithoutYaw(
      imu_orientation * imu_mounting_offset.getRotation());
    tf2::Transform imu_without_yaw_component;
    imu_without_yaw_component.setRotation(imu_orientation_without_yaw_component);
    imu_without_yaw_component.setOrigin({0, 0, 0});

    // transformation chain to get correctly rotated odom frame
    // go to the rotation point in the odom frame. rotate the transform to the base link at this point
    fused_odometry = motion_odometry_yaw * rotation_point_in_base * imu_without_yaw_component * base_link_in_rotation_point;

    // combine it all into a tf
    tf.header.stamp = fused_time_;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_link_frame_;
    geometry_msgs::Transform fused_odom_msg;
    fused_odom_msg = toMsg(fused_odometry);
    tf.transform = fused_odom_msg;
    br.sendTransform(tf);

    last_time_stamp = fused_time_;

    r.sleep();
  }
}

tf2::Quaternion OdometryFuser::getCurrentMotionOdomYaw(tf2::Quaternion motion_odom_rotation)
{
  // Convert tf to eigen quaternion
  Eigen::Quaterniond eigen_quat, out;
  tf2::convert(motion_odom_rotation, eigen_quat);

  // Extract yaw rotation
  double yaw = rot_conv::EYawOfQuat(eigen_quat);

  tf2::Quaternion odom_orientation_yaw;
  odom_orientation_yaw.setRPY(0, 0, yaw);

  return odom_orientation_yaw;
}

tf2::Quaternion OdometryFuser::getCurrentImuRotationWithoutYaw(tf2::Quaternion imu_rotation)
{
  // Calculate robot orientation vector
  tf2::Vector3 robot_vector = tf2::Vector3(0,0,1);
  tf2::Vector3 robot_vector_rotated = robot_vector.rotate(imu_rotation.getAxis(), imu_rotation.getAngle()).normalize();

  // Check if the robots orientation is near the yaw singularity
  if (robot_vector_rotated.z() < 0.2)
  {
    // Use ony a IMU offset during the singularity
    return imu_rotation;
  }

  // Convert tf to eigen quaternion
  Eigen::Quaterniond eigen_quat, eigen_quat_out;
  tf2::convert(imu_rotation, eigen_quat);

  // Remove yaw from quaternion
  rot_conv::QuatNoEYaw(eigen_quat, eigen_quat_out);

  // Convert eigen to tf quaternion
  tf2::Quaternion tf_quat_out;

  tf2::convert(eigen_quat_out, tf_quat_out);

  return tf_quat_out;
}

tf2::Transform OdometryFuser::getCurrentRotationPoint() {


  geometry_msgs::TransformStamped rotation_point;
  tf2::Transform rotation_point_tf;

  char current_support_state = bitbots_msgs::SupportState::DOUBLE;

  bitbots_msgs::SupportState::ConstPtr current_support_state_msg = support_state_cache_.getElemBeforeTime(fused_time_);

  if (current_support_state_msg) {
    current_support_state = current_support_state_msg->state;
  }

  // Wait for the forward kinematics of both legs (simplified by transforming from one to the other) to be avalible for the current fusing operation
  tf_buffer_.canTransform(r_sole_frame_, l_sole_frame_, fused_time_, ros::Duration(0.1));

  // otherwise point of rotation is current support foot sole or center point of the soles if double support
  if (current_support_state == bitbots_msgs::SupportState::RIGHT || current_support_state == bitbots_msgs::SupportState::LEFT) {
    try {
      std::string support_frame;
      if (current_support_state == bitbots_msgs::SupportState::RIGHT)
        support_frame = r_sole_frame_;
      else
        support_frame = l_sole_frame_;
      rotation_point = tf_buffer_.lookupTransform(base_link_frame_, support_frame,
                                                  fused_time_);
      fromMsg(rotation_point.transform, rotation_point_tf);
    } catch (tf2::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
    }
  } else if (current_support_state == bitbots_msgs::SupportState::DOUBLE) {
    // use point between soles if double support or unknown support
    geometry_msgs::TransformStamped base_to_l_sole;
    base_to_l_sole = tf_buffer_.lookupTransform(base_link_frame_, l_sole_frame_, fused_time_);
    geometry_msgs::TransformStamped l_to_r_sole;
    l_to_r_sole = tf_buffer_.lookupTransform(l_sole_frame_, r_sole_frame_, fused_time_);
    tf2::Transform base_to_l_sole_tf;
    tf2::fromMsg(base_to_l_sole.transform, base_to_l_sole_tf);
    tf2::Transform l_to_r_sole_tf;
    tf2::fromMsg(l_to_r_sole.transform, l_to_r_sole_tf);

    // we only want to have the half transform to get the point between the feet
    tf2::Transform l_to_center_tf;
    l_to_center_tf
        .setOrigin({l_to_r_sole_tf.getOrigin().x() /2, l_to_r_sole_tf.getOrigin().y() /2, l_to_r_sole_tf.getOrigin().z() /2});

    // Set to zero rotation, because the rotation measurement is done by the imu
    tf2::Quaternion zero_rotation;
    zero_rotation.setRPY(0,0,0);
    l_to_center_tf.setRotation(zero_rotation);

    rotation_point_tf = base_to_l_sole_tf * l_to_center_tf;
    rotation_point_tf.setRotation(zero_rotation);
  } else {
    ROS_ERROR_THROTTLE(2, "cop not available and unknown support state %c", current_support_state);
  }
  return rotation_point_tf;
}

void OdometryFuser::imuCallback(
    const sensor_msgs::Imu::ConstPtr &imu_msg,
    const nav_msgs::Odometry::ConstPtr &motion_odom_msg) {
  imu_data_ = *imu_msg;
  odom_data_ = *motion_odom_msg;
  // Use the time of the imu as a baseline to do transforms and stuff because it is more timecritical than the walking odometry.
  // The walking odom stamp is also close to this timestamp due to the Synchronizer policy.
  fused_time_ = imu_data_.header.stamp;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "bitbots_odometry");

  OdometryFuser o;
}


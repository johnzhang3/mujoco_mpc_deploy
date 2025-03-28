#include <mocap_optitrack/velocity_estimator.h>

namespace mocap_optitrack {

VelocityEstimator::VelocityEstimator(double alpha)
    : alpha_(alpha), has_previous_state_(false),
      filtered_linear_velocity_(Eigen::Vector3d::Zero()),
      filtered_angular_velocity_(Eigen::Vector3d::Zero()) {
}

geometry_msgs::TwistStamped VelocityEstimator::getRosVelocities(RigidBody const& body, double delta_time, const Version& coordinatesVersion) {
    geometry_msgs::TwistStamped twistStampedMsg;

    // If no previous state is available, initialize and return zero velocities
    if (!has_previous_state_) {
        initializePreviousState(body, coordinatesVersion);
        twistStampedMsg.header.stamp = ros::Time::now();
        twistStampedMsg.twist.linear.x = 0.0;
        twistStampedMsg.twist.linear.y = 0.0;
        twistStampedMsg.twist.linear.z = 0.0;
        twistStampedMsg.twist.angular.x = 0.0;
        twistStampedMsg.twist.angular.y = 0.0;
        twistStampedMsg.twist.angular.z = 0.0;
        return twistStampedMsg;
    }

    Eigen::Vector3d current_position;
    Eigen::Quaterniond current_orientation;
    if (coordinatesVersion < Version("2.0") && coordinatesVersion >= Version("1.7")) {
        current_position = Eigen::Vector3d(-body.pose.position.x, body.pose.position.z, body.pose.position.y);
        current_orientation = Eigen::Quaterniond(body.pose.orientation.w, -body.pose.orientation.x, body.pose.orientation.z, body.pose.orientation.y);

    } else {
        current_position = Eigen::Vector3d(body.pose.position.x, body.pose.position.y, body.pose.position.z);
        current_orientation = Eigen::Quaterniond(body.pose.orientation.w, body.pose.orientation.x, body.pose.orientation.y, body.pose.orientation.z);
    }

    // Compute linear velocity in world coordinates
    Eigen::Vector3d linear_velocity_world = (current_position - previous_position_) / delta_time;

    // Compute angular velocity in world coordinates
    // Getting quaternion difference between current and previous orientation
    Eigen::Quaterniond delta_orientation = current_orientation * previous_orientation_.inverse();

    // current_orientation.x() = -current_orientation.x();
    // current_orientation.y() = -current_orientation.y();
    // current_orientation.z() = -current_orientation.z();
    // Converting to axis angle
    Eigen::AngleAxisd angle_axis(delta_orientation);
    // Finite difference to get angular velocity
    Eigen::Vector3d angular_velocity_world = angle_axis.axis() * angle_axis.angle() / delta_time;

    // Convert velocities to local coordinates
    Eigen::Matrix3d rotation_matrix = current_orientation.toRotationMatrix();
    Eigen::Vector3d linear_velocity_local = rotation_matrix.transpose() * linear_velocity_world;
    Eigen::Vector3d angular_velocity_local = rotation_matrix.transpose() * angular_velocity_world;

    // Low-pass filter the velocities directly
    filtered_linear_velocity_ = alpha_ * linear_velocity_local + (1.0 - alpha_) * filtered_linear_velocity_;
    filtered_angular_velocity_ = alpha_ * angular_velocity_local + (1.0 - alpha_) * filtered_angular_velocity_;

    // Fill in the ROS TwistStamped message
    twistStampedMsg.header.stamp = ros::Time::now();
    twistStampedMsg.twist.linear.x = filtered_linear_velocity_.x();
    twistStampedMsg.twist.linear.y = filtered_linear_velocity_.y();
    twistStampedMsg.twist.linear.z = filtered_linear_velocity_.z();
    twistStampedMsg.twist.angular.x = filtered_angular_velocity_.x();
    twistStampedMsg.twist.angular.y = filtered_angular_velocity_.y();
    twistStampedMsg.twist.angular.z = filtered_angular_velocity_.z();

    // Update the previous state
    updatePreviousState(current_position, current_orientation);
    return twistStampedMsg;
}

void VelocityEstimator::initializePreviousState(RigidBody const& body, const Version& coordinatesVersion) {
    if (coordinatesVersion < Version("2.0") && coordinatesVersion >= Version("1.7")) {
        previous_position_ = Eigen::Vector3d(-body.pose.position.x, body.pose.position.z, body.pose.position.y);
        previous_orientation_ = Eigen::Quaterniond(body.pose.orientation.w, -body.pose.orientation.x, body.pose.orientation.z, body.pose.orientation.y);
    } else {
        previous_position_ = Eigen::Vector3d(body.pose.position.x, body.pose.position.y, body.pose.position.z);
        previous_orientation_ = Eigen::Quaterniond(body.pose.orientation.w, body.pose.orientation.x, body.pose.orientation.z, body.pose.orientation.y);
    }
    has_previous_state_ = true;
}

void VelocityEstimator::updatePreviousState(const Eigen::Vector3d& current_position, const Eigen::Quaterniond& current_orientation) {
    previous_position_ = current_position;
    previous_orientation_ = current_orientation;
}

} // namespace mocap_optitrack

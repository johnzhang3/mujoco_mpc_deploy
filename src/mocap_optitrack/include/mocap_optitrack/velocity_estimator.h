// VelocityEstimator.h

#ifndef VELOCITY_ESTIMATOR_H
#define VELOCITY_ESTIMATOR_H

#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>
#include <mocap_optitrack/version.h>
#include <mocap_optitrack/data_model.h>

namespace mocap_optitrack {

class VelocityEstimator {
public:
    VelocityEstimator(double alpha = 0.6);

    // Method to get the filtered velocities in local coordinates
    geometry_msgs::TwistStamped getRosVelocities(RigidBody const& body, double delta_time, const Version& coordinatesVersion);

private:
    double alpha_;
    bool has_previous_state_;
    Eigen::Vector3d previous_position_;
    Eigen::Quaterniond previous_orientation_;
    Eigen::Vector3d filtered_linear_velocity_;
    Eigen::Vector3d filtered_angular_velocity_;

    // Initialize the previous state
    void initializePreviousState(RigidBody const& body, const Version& coordinatesVersion);

    // Update the previous state with the current data
    void updatePreviousState(const Eigen::Vector3d& current_position, const Eigen::Quaterniond& current_orientation);
};

} // namespace mocap_optitrack

#endif // VELOCITY_ESTIMATOR_H

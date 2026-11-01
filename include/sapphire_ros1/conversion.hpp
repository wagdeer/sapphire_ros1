#pragma once

#include <sapphire/sensor/cloud_ingest.hpp>
#include <sapphire/types.hpp>

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <cstddef>
#include <cstdint>
#include <string>

namespace sapphire_ros1 {

/// Convert a ROS 1 timestamp to seconds without discarding nanoseconds.
double stampToSeconds(const ros::Time& stamp);

/// Convert absolute seconds to a ROS 1 timestamp.
/// Throws std::invalid_argument for non-finite or unsupported ROS 1 times.
ros::Time stampFromSeconds(double stamp);

/// Build the core's zero-copy view over a ROS PointCloud2 payload.
sapphire::sensor::CloudView makeCloudView(
    const sensor_msgs::PointCloud2& message);

/// Convert ROS IMU fields to the core IMU type.
sapphire::ImuData toCoreImu(const sensor_msgs::Imu& message);

/// Materialize core points from a successful cloud-ingest result.
sapphire::PointCloudPtr toCoreCloud(
    const sapphire::sensor::IngestResult& ingested,
    double time_offset_s = 0.0);

/// Convert a core isometry to a normalized ROS pose.
geometry_msgs::Pose toRosPose(const sapphire::Isometry3d& transform);

/// Convert an odometry result without relying on ROS PCL or tf2_eigen.
nav_msgs::Odometry toRosOdometry(
    const sapphire::OdometryResult& result,
    const std::string& world_frame,
    const std::string& child_frame,
    const sapphire::Isometry3d& T_child_lidar =
        sapphire::Isometry3d::Identity());

/// Serialize the Sapphire point cloud directly to a self-describing
/// PointCloud2 message. The layout is x/y/z/intensity/timestamp.
sensor_msgs::PointCloud2 toRosPointCloud(
    const sapphire::PointCloud& cloud,
    const ros::Time& stamp,
    const std::string& frame_id);

const char* datatypeLabel(uint8_t datatype);

}  // namespace sapphire_ros1

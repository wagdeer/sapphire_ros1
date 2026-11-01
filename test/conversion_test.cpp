#include <sapphire_ros1/conversion.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace {

double readFloat64(const std::vector<uint8_t>& data, size_t offset) {
    double value = 0.0;
    std::memcpy(&value, data.data() + offset, sizeof(value));
    return value;
}

TEST(StampConversion, PreservesExactFractionalSeconds) {
    const ros::Time stamp = sapphire_ros1::stampFromSeconds(
        1700000000.125);
    EXPECT_EQ(stamp.sec, 1700000000u);
    EXPECT_EQ(stamp.nsec, 125000000u);
    EXPECT_DOUBLE_EQ(
        sapphire_ros1::stampToSeconds(stamp), 1700000000.125);
}

TEST(StampConversion, RejectsInvalidRos1Times) {
    EXPECT_THROW(
        sapphire_ros1::stampFromSeconds(
            std::numeric_limits<double>::quiet_NaN()),
        std::invalid_argument);
    EXPECT_THROW(sapphire_ros1::stampFromSeconds(-1.0), std::invalid_argument);
    EXPECT_THROW(
        sapphire_ros1::stampFromSeconds(4294967296.0), std::out_of_range);
}

TEST(ImuConversion, CopiesTimeAccelerationAndGyro) {
    sensor_msgs::Imu message;
    message.header.stamp = sapphire_ros1::stampFromSeconds(
        1700000001.25);
    message.linear_acceleration.x = 1.0;
    message.linear_acceleration.y = 2.0;
    message.linear_acceleration.z = 9.8;
    message.angular_velocity.x = 0.1;
    message.angular_velocity.y = 0.2;
    message.angular_velocity.z = 0.3;

    const sapphire::ImuData imu = sapphire_ros1::toCoreImu(message);
    EXPECT_DOUBLE_EQ(imu.stamp, 1700000001.25);
    EXPECT_DOUBLE_EQ(imu.accel.x(), 1.0);
    EXPECT_DOUBLE_EQ(imu.accel.y(), 2.0);
    EXPECT_DOUBLE_EQ(imu.accel.z(), 9.8);
    EXPECT_DOUBLE_EQ(imu.gyro.x(), 0.1);
    EXPECT_DOUBLE_EQ(imu.gyro.y(), 0.2);
    EXPECT_DOUBLE_EQ(imu.gyro.z(), 0.3);
}

TEST(PointCloudConversion, RoundTripsFieldsAndTimestamps) {
    sapphire::PointCloud cloud;
    sapphire::Point point;
    point.x = 1.0f;
    point.y = 2.0f;
    point.z = 3.0f;
    point.intensity = 4.0f;
    point.timestamp = 1700000002.125;
    cloud.push_back(point);
    cloud.width = 1;
    cloud.height = 1;
    cloud.is_dense = false;

    const ros::Time stamp = sapphire_ros1::stampFromSeconds(
        1700000002.125);
    const sensor_msgs::PointCloud2 message = sapphire_ros1::toRosPointCloud(
        cloud, stamp, "lidar");

    ASSERT_EQ(message.fields.size(), 5u);
    EXPECT_EQ(message.header.frame_id, "lidar");
    EXPECT_EQ(message.header.stamp, stamp);
    EXPECT_EQ(message.width, 1u);
    EXPECT_EQ(message.height, 1u);
    EXPECT_EQ(message.point_step, 24u);
    EXPECT_EQ(message.row_step, 24u);
    ASSERT_EQ(message.data.size(), 24u);
    EXPECT_NEAR(
        readFloat64(message.data, 16),
        point.timestamp,
        1e-12);

    const sapphire::sensor::CloudView view =
        sapphire_ros1::makeCloudView(message);
    sapphire::sensor::IngestConfig config;
    config.time_field = "timestamp";
    config.time_unit = "s";
    config.time_ref = "absolute";
    const sapphire::sensor::IngestResult ingested =
        sapphire::sensor::ingest(view, config);
    ASSERT_EQ(ingested.points.size(), 1u);
    EXPECT_EQ(ingested.points.front().stamp, point.timestamp);
    EXPECT_FLOAT_EQ(ingested.points.front().intensity, point.intensity);

    const sapphire::PointCloudPtr core_cloud =
        sapphire_ros1::toCoreCloud(ingested);
    ASSERT_EQ(core_cloud->points.size(), 1u);
    EXPECT_EQ(core_cloud->width, 1u);
    EXPECT_EQ(core_cloud->height, 1u);
    EXPECT_FLOAT_EQ(core_cloud->points.front().x, point.x);
    EXPECT_DOUBLE_EQ(core_cloud->points.front().timestamp, point.timestamp);

    const sapphire::PointCloudPtr shifted_cloud =
        sapphire_ros1::toCoreCloud(ingested, 0.0015);
    EXPECT_DOUBLE_EQ(
        shifted_cloud->points.front().timestamp,
        point.timestamp + 0.0015);
}

TEST(OdometryConversion, UsesWorldAndChildFrames) {
    sapphire::OdometryResult result;
    result.stamp = 1700000003.5;
    result.T_world_lidar = sapphire::Isometry3d::Identity();
    result.T_world_lidar.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
    result.v_world = Eigen::Vector3d(0.5, 0.0, 0.0);

    const nav_msgs::Odometry message =
        sapphire_ros1::toRosOdometry(result, "map", "lidar");
    EXPECT_EQ(message.header.frame_id, "map");
    EXPECT_EQ(message.child_frame_id, "lidar");
    EXPECT_DOUBLE_EQ(message.pose.pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(message.pose.pose.position.y, 2.0);
    EXPECT_DOUBLE_EQ(message.pose.pose.position.z, 3.0);
    EXPECT_DOUBLE_EQ(message.twist.twist.linear.x, 0.5);
    EXPECT_DOUBLE_EQ(message.pose.pose.orientation.w, 1.0);
}

TEST(OdometryConversion, CanExpressPoseInImuFrame) {
    sapphire::OdometryResult result;
    result.stamp = 1700000004.0;
    result.T_world_lidar = sapphire::Isometry3d::Identity();

    sapphire::Isometry3d T_imu_lidar = sapphire::Isometry3d::Identity();
    T_imu_lidar.translation() = Eigen::Vector3d(0.1, 0.2, 0.3);
    const nav_msgs::Odometry message = sapphire_ros1::toRosOdometry(
        result, "map", "imu", T_imu_lidar.inverse());

    EXPECT_EQ(message.child_frame_id, "imu");
    EXPECT_NEAR(message.pose.pose.position.x, -0.1, 1e-12);
    EXPECT_NEAR(message.pose.pose.position.y, -0.2, 1e-12);
    EXPECT_NEAR(message.pose.pose.position.z, -0.3, 1e-12);
}

}  // namespace

#pragma once

#include <sapphire/sensor/cloud_ingest.hpp>
#include <sapphire/types.hpp>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/timer.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <atomic>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace sapphire {
class OdometryPipeline;
struct OdometryResult;
}  // namespace sapphire

namespace sapphire_ros1 {

/// Thin ROS 1 adapter for the ROS-independent Sapphire odometry pipeline.
class SapphireRos {
public:
    SapphireRos();
    ~SapphireRos();

    SapphireRos(const SapphireRos&) = delete;
    SapphireRos& operator=(const SapphireRos&) = delete;

    /// Write the map trajectory in TUM format. Safe to call more than once.
    /// Returns false only when a configured write fails. An empty path, a
    /// missing pipeline, and an empty trajectory are successful. A repeated
    /// call returns true; the first call reports the I/O result.
    bool saveTrajectory();

private:
    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& message);
    void imuCallback(const sensor_msgs::ImuConstPtr& message);
    void pgoVisualizationCallback(const ros::WallTimerEvent& event);

    void publishOdometry(const sapphire::OdometryResult& result);
    void publishDeskewed(double stamp);
    void publishPgoVisualization();
    void publishTrajectory();

    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_handle_;

    ros::Publisher odom_publisher_;
    ros::Publisher trajectory_publisher_;
    ros::Publisher deskewed_publisher_;
    ros::Publisher pgo_map_publisher_;
    ros::Publisher pgo_graph_publisher_;
    ros::WallTimer pgo_visualization_timer_;

    ros::Subscriber lidar_subscriber_;
    ros::Subscriber imu_subscriber_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<sapphire::OdometryPipeline> pipeline_;

    sapphire::sensor::IngestConfig lidar_ingest_;
    bool require_point_time_ = false;
    bool lidar_time_locked_ = false;
    double locked_lidar_span_s_ = 0.0;
    bool warned_lidar_span_ratio_ = false;
    bool warned_lidar_header_mismatch_ = false;

    std::string config_file_;
    std::string world_frame_ = "map";
    std::string lidar_frame_ = "lidar";
    std::string imu_frame_ = "imu";
    std::string pose_frame_ = "lidar";
    double lidar_time_offset_s_ = 0.0;
    double imu_time_offset_s_ = 0.0;
    sapphire::Isometry3d T_pose_lidar_ = sapphire::Isometry3d::Identity();
    std::filesystem::path trajectory_output_path_;

    std::atomic<bool> trajectory_saved_{false};
    std::optional<double> last_lidar_result_stamp_;
    std::mutex lidar_callback_mutex_;
    std::mutex odometry_publish_mutex_;
    int64_t last_tf_stamp_ns_ = 0;
    std::mutex trajectory_publish_mutex_;
    bool trajectory_published_ = false;
    size_t trajectory_samples_published_ = 0;
    size_t trajectory_revision_published_ = 0;
};

}  // namespace sapphire_ros1

#include <sapphire_ros1/sapphire_ros.hpp>

#include <sapphire/config.hpp>
#include <sapphire/io/tum.hpp>
#include <sapphire/odometry/pipeline.hpp>
#include <sapphire_ros1/conversion.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <ros/console.h>

#include <cmath>
#include <exception>
#include <stdexcept>
#include <utility>
#include <vector>

namespace sapphire_ros1 {
namespace {

geometry_msgs::Point toRosPoint(const sapphire::Isometry3d& pose) {
    geometry_msgs::Point point;
    point.x = pose.translation().x();
    point.y = pose.translation().y();
    point.z = pose.translation().z();
    return point;
}

}  // namespace

SapphireRos::SapphireRos()
    : node_handle_(),
      private_node_handle_("~")
{
    ROS_INFO("[sapphire_ros1] node starting");

    private_node_handle_.param<std::string>(
        "config_file", config_file_, std::string());
    private_node_handle_.param<std::string>(
        "world_frame", world_frame_, std::string("map"));
    private_node_handle_.param<std::string>(
        "lidar_frame", lidar_frame_, std::string("lidar"));
    private_node_handle_.param<std::string>(
        "imu_frame", imu_frame_, std::string("imu"));
    private_node_handle_.param<std::string>(
        "pose_frame", pose_frame_, std::string("lidar"));
    private_node_handle_.param<double>(
        "lidar_time_offset", lidar_time_offset_s_, 0.0);
    private_node_handle_.param<double>(
        "imu_time_offset", imu_time_offset_s_, 0.0);
    if (config_file_.empty()) {
        throw std::invalid_argument("~config_file must not be empty");
    }
    if (
        world_frame_.empty() || lidar_frame_.empty() || imu_frame_.empty()) {
        throw std::invalid_argument(
            "~world_frame, ~lidar_frame, and ~imu_frame must not be empty");
    }
    if (
        !std::isfinite(lidar_time_offset_s_)
        || !std::isfinite(imu_time_offset_s_)) {
        throw std::invalid_argument("sensor time offsets must be finite");
    }
    if (pose_frame_ != "lidar" && pose_frame_ != "imu") {
        throw std::invalid_argument(
            "~pose_frame must be either 'lidar' or 'imu'");
    }

    const sapphire::Config config = sapphire::loadConfig(config_file_);
    if (pose_frame_ == "imu") {
        T_pose_lidar_ = config.extrinsics.T_imu_lidar.inverse();
    }
    trajectory_output_path_ = sapphire::trajectoryOutputPath(config);
    std::string trajectory_output_override;
    private_node_handle_.param<std::string>(
        "trajectory_output", trajectory_output_override, std::string());
    if (!trajectory_output_override.empty()) {
        trajectory_output_path_ = trajectory_output_override;
    }
    lidar_ingest_.time_field = config.sensor.lidar.time_field;
    lidar_ingest_.time_unit = config.sensor.lidar.time_unit;
    lidar_ingest_.time_ref = config.sensor.lidar.time_ref;
    lidar_ingest_.max_scan_duration =
        config.sensor.lidar.max_scan_duration;
    lidar_ingest_.header_tolerance = config.sensor.lidar.header_tolerance;
    require_point_time_ = config.sensor.lidar.require_point_time;

    ROS_INFO(
        "[sapphire_ros1] loaded algorithm config: %s",
        config_file_.c_str());
    if (trajectory_output_path_.empty()) {
        ROS_INFO(
            "[sapphire_ros1] TUM trajectory dump is off "
            "(output.trajectory_dir is empty)");
    } else {
        ROS_INFO(
            "[sapphire_ros1] TUM trajectory will be saved on exit: %s",
            trajectory_output_path_.c_str());
    }

    pipeline_ = std::make_unique<sapphire::OdometryPipeline>(config);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();

    odom_publisher_ = node_handle_.advertise<nav_msgs::Odometry>(
        "sapphire/odometry", 10);
    deskewed_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(
        "sapphire/deskewed", 1);

    if (config.pgo.enabled) {
        trajectory_publisher_ = node_handle_.advertise<nav_msgs::Path>(
            "sapphire/trajectory", 1, true);
        pgo_map_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(
            "sapphire/pgo/map", 1, true);
        pgo_graph_publisher_ =
            node_handle_.advertise<visualization_msgs::MarkerArray>(
                "sapphire/pgo/graph", 1, true);
        pgo_visualization_timer_ = node_handle_.createTimer(
            ros::Duration(1.0),
            &SapphireRos::pgoVisualizationCallback,
            this);
    }

    // At least two callback threads are required: LiDAR processing can wait
    // for scan-end IMU coverage, so a single-threaded spinner can deadlock.
    lidar_subscriber_ =
        node_handle_.subscribe("lidar/points", 1, &SapphireRos::lidarCallback,
                               this);
    // Keep enough IMU samples queued to cover the core's 500 ms scan-end wait.
    imu_subscriber_ =
        node_handle_.subscribe(
            "imu/data", 100, &SapphireRos::imuCallback, this);

    ROS_INFO("[sapphire_ros1] ready");
}

SapphireRos::~SapphireRos() {
    (void)saveTrajectory();
}

bool SapphireRos::saveTrajectory() {
    if (trajectory_saved_.exchange(true)) {
        return true;
    }
    if (trajectory_output_path_.empty() || !pipeline_) {
        return true;
    }

    sapphire::MapTrajectory trajectory = pipeline_->mapTrajectory();
    for (sapphire::MapTrajectorySample& sample : trajectory.samples) {
        sample.T_map_lidar = sample.T_map_lidar * T_pose_lidar_;
    }
    const sapphire::TrajectorySaveReport report = sapphire::saveMapTrajectory(
        trajectory_output_path_, trajectory);
    switch (report.status) {
    case sapphire::TrajectorySaveStatus::NotConfigured:
        return true;
    case sapphire::TrajectorySaveStatus::Empty:
        ROS_WARN(
            "[sapphire_ros1] no TUM trajectory written to %s",
            trajectory_output_path_.c_str());
        return true;
    case sapphire::TrajectorySaveStatus::Written:
        ROS_INFO(
            "[sapphire_ros1] wrote %zu TUM poses to %s",
            report.result.written,
            trajectory_output_path_.c_str());
        if (report.result.skipped > 0) {
            ROS_WARN(
                "[sapphire_ros1] skipped %zu trajectory poses with "
                "non-finite values or non-increasing timestamps",
                report.result.skipped);
        }
        return true;
    case sapphire::TrajectorySaveStatus::IoFailure:
        ROS_ERROR(
            "[sapphire_ros1] failed to write TUM trajectory: %s",
            report.error.c_str());
        return false;
    }
    return false;
}

void SapphireRos::lidarCallback(
    const sensor_msgs::PointCloud2ConstPtr& message)
{
    // LiDAR must serialize with other LiDAR work, while IMU callbacks remain
    // free to run on the second spinner thread.
    std::lock_guard<std::mutex> lidar_lock(lidar_callback_mutex_);
    if (message->width == 0 || message->height == 0) {
        return;
    }

    try {
        const sapphire::sensor::CloudView view = makeCloudView(*message);
        const sapphire::sensor::IngestResult ingested =
            sapphire::sensor::ingest(view, lidar_ingest_);
        const double stamp = view.header_stamp;

        if (!std::isfinite(stamp)) {
            ROS_WARN_THROTTLE(
                5.0, "[sensor] dropping lidar frame with non-finite stamp");
            return;
        }
        if (ingested.points.empty()) {
            ROS_WARN_THROTTLE(
                5.0,
                "[sensor] dropping lidar frame: %s",
                ingested.time.note.empty() ? "no points"
                                           : ingested.time.note.c_str());
            return;
        }
        if (!ingested.time.ok && require_point_time_) {
            ROS_WARN_THROTTLE(
                5.0,
                "[sensor] dropping lidar frame: %s",
                ingested.time.note.c_str());
            return;
        }

        if (ingested.time.ok && !lidar_time_locked_) {
            lidar_ingest_.time_field = ingested.time.field;
            lidar_ingest_.time_unit =
                sapphire::sensor::unitName(ingested.time.unit);
            lidar_ingest_.time_ref =
                ingested.time.ref == sapphire::sensor::TimeRef::Absolute
                ? "absolute"
                : "relative";
            locked_lidar_span_s_ = ingested.time.span_s;
            lidar_time_locked_ = true;

            const sapphire::sensor::FieldDescriptor* time_field =
                sapphire::sensor::findField(view, ingested.time.field);
            ROS_INFO(
                "[sensor] lidar field=%s(%s) unit=%s ref=%s span=%.6fs "
                "header_offset=%+.6fs kept=%zu dropped_time=%zu deskew=%s",
                ingested.time.field.c_str(),
                datatypeLabel(
                    time_field == nullptr ? 0 : time_field->datatype),
                lidar_ingest_.time_unit.c_str(),
                lidar_ingest_.time_ref.c_str(),
                ingested.time.span_s,
                ingested.time.header_offset_s,
                ingested.points.size(),
                ingested.dropped_time,
                ingested.deskewable ? "on" : "off");
            ROS_INFO(
                "[sensor] profile: time_field=\"%s\" time_unit=\"%s\" "
                "time_ref=\"%s\"",
                lidar_ingest_.time_field.c_str(),
                lidar_ingest_.time_unit.c_str(),
                lidar_ingest_.time_ref.c_str());
        } else if (
            ingested.time.ok && lidar_time_locked_
            && locked_lidar_span_s_ > 0.0 && !warned_lidar_span_ratio_) {
            const double ratio =
                ingested.time.span_s / locked_lidar_span_s_;
            if (ratio > 10.0 || ratio < 0.1) {
                warned_lidar_span_ratio_ = true;
                ROS_WARN(
                    "[sensor] lidar span changed by more than 10x under the "
                    "locked unit: locked=%.6fs now=%.6fs",
                    locked_lidar_span_s_,
                    ingested.time.span_s);
            }
        }
        if (
            ingested.time.ok && ingested.time.header_mismatch
            && !warned_lidar_header_mismatch_) {
            warned_lidar_header_mismatch_ = true;
            ROS_WARN(
                "[sensor] lidar header_mismatch: header_offset=%+.6fs; "
                "using point time as absolute seconds",
                ingested.time.header_offset_s);
        }

        pipeline_->pushLidar(
            stamp + lidar_time_offset_s_,
            toCoreCloud(ingested, lidar_time_offset_s_));

        const sapphire::OdometryResult result = pipeline_->latestResult();
        if (
            last_lidar_result_stamp_.has_value()
            && result.stamp == *last_lidar_result_stamp_) {
            return;
        }
        last_lidar_result_stamp_ = result.stamp;
        if (result.converged) {
            const std::optional<sapphire::OdometryResult> propagated =
                pipeline_->latestPropagatedResult();
            publishOdometry(propagated.value_or(result));
            publishDeskewed(result.stamp);
            publishTrajectory();
        }
    } catch (const std::exception& error) {
        ROS_ERROR_THROTTLE(
            5.0, "[sapphire_ros1] LiDAR callback failed: %s", error.what());
    }
}

void SapphireRos::imuCallback(const sensor_msgs::ImuConstPtr& message) {
    try {
        sapphire::ImuData imu = toCoreImu(*message);
        imu.stamp += imu_time_offset_s_;
        pipeline_->pushImu(imu);
        if (const auto propagated = pipeline_->latestPropagatedResult()) {
            publishOdometry(*propagated);
        }
    } catch (const std::exception& error) {
        ROS_ERROR_THROTTLE(
            5.0, "[sapphire_ros1] IMU callback failed: %s", error.what());
    }
}

void SapphireRos::publishOdometry(
    const sapphire::OdometryResult& result)
{
    const std::string& child_frame =
        pose_frame_ == "imu" ? imu_frame_ : lidar_frame_;
    const nav_msgs::Odometry message = toRosOdometry(
        result, world_frame_, child_frame, T_pose_lidar_);
    const int64_t stamp_ns =
        static_cast<int64_t>(message.header.stamp.sec) * 1000000000LL
        + static_cast<int64_t>(message.header.stamp.nsec);

    std::lock_guard<std::mutex> lock(odometry_publish_mutex_);
    odom_publisher_.publish(message);
    if (stamp_ns <= last_tf_stamp_ns_) {
        return;
    }
    last_tf_stamp_ns_ = stamp_ns;

    geometry_msgs::TransformStamped transform;
    transform.header = message.header;
    transform.child_frame_id = message.child_frame_id;
    transform.transform.translation.x = message.pose.pose.position.x;
    transform.transform.translation.y = message.pose.pose.position.y;
    transform.transform.translation.z = message.pose.pose.position.z;
    transform.transform.rotation = message.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
}

void SapphireRos::publishTrajectory() {
    if (!trajectory_publisher_) {
        return;
    }

    // Lidar callbacks and the PGO timer can publish concurrently. Keep the
    // snapshot, freshness check, and publish under one lock so an older
    // snapshot cannot replace a path that already contains a later loop.
    std::lock_guard<std::mutex> lock(trajectory_publish_mutex_);
    const sapphire::MapTrajectory trajectory = pipeline_->mapTrajectory();
    if (trajectory.samples.empty()) {
        return;
    }

    const bool newer_revision =
        trajectory.revision > trajectory_revision_published_;
    const bool more_samples =
        trajectory.revision == trajectory_revision_published_
        && trajectory.samples.size() > trajectory_samples_published_;
    if (trajectory_published_ && !newer_revision && !more_samples) {
        return;
    }

    nav_msgs::Path path;
    path.header.frame_id = world_frame_;
    path.header.stamp =
        stampFromSeconds(trajectory.samples.back().stamp);
    path.poses.reserve(trajectory.samples.size());
    for (const sapphire::MapTrajectorySample& sample : trajectory.samples) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = world_frame_;
        pose.header.stamp = stampFromSeconds(sample.stamp);
        pose.pose = toRosPose(sample.T_map_lidar * T_pose_lidar_);
        path.poses.push_back(std::move(pose));
    }

    trajectory_publisher_.publish(path);
    trajectory_published_ = true;
    trajectory_samples_published_ = trajectory.samples.size();
    trajectory_revision_published_ = trajectory.revision;
}

void SapphireRos::publishPgoVisualization() {
    publishTrajectory();
    const ros::Time stamp = ros::Time::now();

    if (
        pgo_graph_publisher_
        && pgo_graph_publisher_.getNumSubscribers() > 0) {
        pipeline_->requestPoseGraphSnapshot();
        const sapphire::PoseGraphSnapshot snapshot =
            pipeline_->poseGraphSnapshot();
        if (!snapshot.optimized_poses.empty()) {
            visualization_msgs::Marker odometry_edges;
            odometry_edges.header.stamp = stamp;
            odometry_edges.header.frame_id = world_frame_;
            odometry_edges.ns = "odometry_edges";
            odometry_edges.id = 0;
            odometry_edges.type = visualization_msgs::Marker::LINE_LIST;
            odometry_edges.action = visualization_msgs::Marker::ADD;
            odometry_edges.pose.orientation.w = 1.0;
            odometry_edges.scale.x = 0.05;
            odometry_edges.color.r = 0.1f;
            odometry_edges.color.g = 0.8f;
            odometry_edges.color.b = 0.2f;
            odometry_edges.color.a = 1.0f;
            odometry_edges.points.reserve(
                (snapshot.optimized_poses.size() - 1) * 2);
            for (size_t i = 1; i < snapshot.optimized_poses.size(); ++i) {
                odometry_edges.points.push_back(
                    toRosPoint(snapshot.optimized_poses[i - 1]));
                odometry_edges.points.push_back(
                    toRosPoint(snapshot.optimized_poses[i]));
            }

            visualization_msgs::Marker loop_edges;
            loop_edges.header = odometry_edges.header;
            loop_edges.ns = "loop_edges";
            loop_edges.id = 0;
            loop_edges.type = visualization_msgs::Marker::LINE_LIST;
            loop_edges.action = visualization_msgs::Marker::ADD;
            loop_edges.pose.orientation.w = 1.0;
            loop_edges.scale.x = 0.12;
            loop_edges.color.r = 1.0f;
            loop_edges.color.g = 0.1f;
            loop_edges.color.b = 0.1f;
            loop_edges.color.a = 1.0f;
            loop_edges.points.reserve(snapshot.loop_edges.size() * 2);
            for (const sapphire::PoseGraphEdge& edge : snapshot.loop_edges) {
                if (
                    edge.source >= snapshot.optimized_poses.size()
                    || edge.target >= snapshot.optimized_poses.size()) {
                    continue;
                }
                loop_edges.points.push_back(
                    toRosPoint(snapshot.optimized_poses[edge.source]));
                loop_edges.points.push_back(
                    toRosPoint(snapshot.optimized_poses[edge.target]));
            }

            visualization_msgs::Marker nodes;
            nodes.header = odometry_edges.header;
            nodes.ns = "nodes";
            nodes.id = 0;
            nodes.type = visualization_msgs::Marker::SPHERE_LIST;
            nodes.action = visualization_msgs::Marker::ADD;
            nodes.pose.orientation.w = 1.0;
            nodes.scale.x = 0.15;
            nodes.scale.y = 0.15;
            nodes.scale.z = 0.15;
            nodes.color.r = 0.1f;
            nodes.color.g = 0.4f;
            nodes.color.b = 1.0f;
            nodes.color.a = 1.0f;
            nodes.points.reserve(snapshot.optimized_poses.size());
            for (const sapphire::Isometry3d& pose :
                 snapshot.optimized_poses) {
                nodes.points.push_back(toRosPoint(pose));
            }

            visualization_msgs::MarkerArray graph;
            graph.markers.push_back(std::move(odometry_edges));
            graph.markers.push_back(std::move(loop_edges));
            graph.markers.push_back(std::move(nodes));
            pgo_graph_publisher_.publish(graph);
        }
    }

    if (
        pgo_map_publisher_
        && pgo_map_publisher_.getNumSubscribers() > 0) {
        pipeline_->requestPoseGraphMap();
        const sapphire::PointCloudConstPtr map =
            pipeline_->latestPoseGraphMap();
        if (map && !map->empty()) {
            pgo_map_publisher_.publish(
                toRosPointCloud(*map, stamp, world_frame_));
        }
    }
}

void SapphireRos::publishDeskewed(double stamp) {
    if (
        !deskewed_publisher_
        || deskewed_publisher_.getNumSubscribers() == 0) {
        return;
    }

    const sapphire::PointCloudConstPtr cloud = pipeline_->latestDeskewed();
    if (!cloud || cloud->points.empty()) {
        return;
    }
    deskewed_publisher_.publish(
        toRosPointCloud(*cloud, stampFromSeconds(stamp), world_frame_));
}

void SapphireRos::pgoVisualizationCallback(
    const ros::TimerEvent& /*event*/)
{
    try {
        publishPgoVisualization();
    } catch (const std::exception& error) {
        ROS_ERROR_THROTTLE(
            5.0,
            "[sapphire_ros1] PGO visualization failed: %s",
            error.what());
    }
}

}  // namespace sapphire_ros1

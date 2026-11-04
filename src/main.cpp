#include <sapphire_ros1/sapphire_ros.hpp>

#include <ros/console.h>
#include <ros/init.h>
#include <ros/ros.h>
#include <spdlog/spdlog.h>

#include <csignal>
#include <cstdlib>
#include <exception>
#include <string>

namespace {

volatile std::sig_atomic_t shutdown_requested = 0;

extern "C" void requestShutdown(int /*signal*/) {
    shutdown_requested = 1;
}

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "sapphire_ros");

    spdlog::set_level(spdlog::level::warn);
    if (const char* level = std::getenv("SAPPHIRE_LOG_LEVEL")) {
        const std::string name(level);
        if (name == "trace") {
            spdlog::set_level(spdlog::level::trace);
        } else if (name == "debug") {
            spdlog::set_level(spdlog::level::debug);
        } else if (name == "info") {
            spdlog::set_level(spdlog::level::info);
        } else if (name == "warn") {
            spdlog::set_level(spdlog::level::warn);
        } else if (name == "error" || name == "err") {
            spdlog::set_level(spdlog::level::err);
        } else if (name == "critical") {
            spdlog::set_level(spdlog::level::critical);
        }
    }

    try {
        sapphire_ros1::SapphireRos node;
        ros::AsyncSpinner spinner(2);
        std::signal(SIGINT, requestShutdown);
        std::signal(SIGTERM, requestShutdown);
        spinner.start();
        while (ros::ok() && shutdown_requested == 0) {
            ros::WallDuration(0.1).sleep();
        }
        spinner.stop();
        const bool trajectory_ok = node.saveTrajectory();
        if (ros::ok()) {
            ros::shutdown();
        }
        // The pose-graph worker may still be servicing a dense loop-closure
        // backlog. The trajectory save has already been attempted, so skip
        // potentially long C++ destructors that would make roslaunch SIGKILL
        // an otherwise finished run. The exit status reports an I/O failure.
        std::_Exit(trajectory_ok ? EXIT_SUCCESS : EXIT_FAILURE);
    } catch (const std::exception& error) {
        ROS_FATAL("[sapphire_ros1] fatal startup error: %s", error.what());
        return 1;
    }
    return 0;
}

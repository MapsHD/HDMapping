#pragma once
//
// Optional ROS 2 bag export for the Trajectory Viewer.
//
// This header is ROS-free on purpose: it only describes *what* to export so the
// viewer (and any other non-ROS translation unit) can include it unconditionally.
// The implementation in RosExport.cpp is the only place that pulls in rclcpp /
// rosbag2_cpp, and it is compiled only when CALIB_ENABLE_ROS_EXPORT is defined.
//
#include <Eigen/Geometry>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <CalibCore/Camera.h> // Intrinsics, Extrinsics
#include <CalibCore/Trajectory.h> // Trajectory, TrajPose

using namespace calib;

// Everything the exporter needs, gathered by the viewer. Plain data only.
struct RosExportInput
{
    // Frame names used in the bag.
    std::string mapFrame = "map";
    std::string lidarFrame = "lidar";
    std::string cameraFrame = "camera";

    // Trajectory of T_map_lidar poses (timestamps in nanoseconds, shared clock).
    Trajectory traj;

    // Camera images, keyed by timestamp (ns) -> .jpg path. The map is inherently
    // ordered by timestamp, so it doubles as the sorted list of image stamps.
    std::map<int64_t, std::string> imageFiles;
    bool calibLoaded = false;
    Intrinsics K;
    Extrinsics E; // tx/ty/tz (camera position); rotation is R_wc below, not E.om/fi/ka
    Eigen::Matrix3f R_wc = Eigen::Matrix3f::Identity(); // camera orientation in world/LiDAR frame

    // LiDAR chunks: each .laz plus its optional MRP correction (T applied to the
    // points to bring them into the map frame). Points carry per-point ns stamps.
    struct Chunk
    {
        std::string lazPath;
        Eigen::Affine3f M = Eigen::Affine3f::Identity();
        bool hasM = false;
    };
    std::vector<Chunk> lidarChunks;
};

struct RosExportOptions
{
    std::string outUri = "ros2_export"; // output bag directory (rosbag2 uri)
    std::string storageId = "mcap"; // "mcap" or "sqlite3"

    bool exportTf = true; // /tf (dynamic) + /tf_static
    bool exportCamera = true; // /camera/image_raw[/compressed] + /camera/camera_info
    bool compressCamera = true; // true: CompressedImage (jpeg) ; false: raw Image (bgr8)
    // Rectify (undistort) images to the pinhole model before writing. Needed for
    // RViz-style overlays, which project with the pinhole P and ignore the
    // distortion coefficients. When on, CameraInfo is published with zero D.
    bool undistortCamera = true;

    // LiDAR can be exported in two flavours, independently:
    //  - undistorted: points as registered by LIO, in the map frame (already
    //    motion-compensated). Topic /lidar/points_undistorted, frame_id = map.
    //  - raw: points re-projected into the sensor frame at each point's stamp via
    //    the inverse trajectory pose (re-introduces scan motion). Topic
    //    /lidar/points_raw, frame_id = lidar, positioned live by /tf.
    bool exportLidarUndistorted = true;
    bool exportLidarRaw = false;

    double aggregationSec = 0.1; // LiDAR points grouped into windows of this length
    int lidarDecim = 1; // keep every Nth point (>=1)
};

// Writes the bag. Returns true on success; `status` always gets a human-readable
// summary (or the error). Safe to call only when built with CALIB_ENABLE_ROS_EXPORT;
// otherwise a stub returns false explaining the build is non-ROS.
bool exportRos2Bag(const RosExportInput& in, const RosExportOptions& opt, std::string& status);
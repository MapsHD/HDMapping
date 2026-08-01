#include "RosExport.h"

#ifndef CALIB_ENABLE_ROS_EXPORT
// ── Non-ROS build: provide a stub so the viewer always links. ─────────────────
bool exportRos2Bag(const RosExportInput&, const RosExportOptions&, std::string& status) {
    status = "ROS export not available: built without CALIB_ENABLE_ROS_EXPORT";
    return false;
}
#else

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>

#include <cstdio>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "PointCloud.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iterator>

namespace {

constexpr char kTopicTfStatic[]     = "/tf_static";
constexpr char kTopicTf[]           = "/tf";
constexpr char kTopicImgCompressed[]= "/camera/image_raw/compressed";
constexpr char kTopicImgRaw[]       = "/camera/image_raw";
constexpr char kTopicCamInfo[]      = "/camera/camera_info";
constexpr char kTopicLidarUndist[]  = "/lidar/points_undistorted";
constexpr char kTopicLidarRaw[]     = "/lidar/points_raw";

builtin_interfaces::msg::Time toRosTime(int64_t ns) {
    builtin_interfaces::msg::Time t;
    t.sec     = static_cast<int32_t>(ns / 1000000000LL);
    t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    return t;
}

geometry_msgs::msg::Transform toTransform(const Eigen::Affine3f& T) {
    geometry_msgs::msg::Transform tf;
    tf.translation.x = T.translation().x();
    tf.translation.y = T.translation().y();
    tf.translation.z = T.translation().z();
    Eigen::Quaternionf q(T.linear());
    q.normalize();
    tf.rotation.x = q.x();
    tf.rotation.y = q.y();
    tf.rotation.z = q.z();
    tf.rotation.w = q.w();
    return tf;
}

// Build an xyz+intensity PointCloud2 over a slice of float quads [x,y,z,i]*n.
sensor_msgs::msg::PointCloud2 makeCloud(const std::string& frame, int64_t stampNs,
                                        const std::vector<float>& xyzi) {
    using PF = sensor_msgs::msg::PointField;
    sensor_msgs::msg::PointCloud2 pc;
    pc.header.stamp    = toRosTime(stampNs);
    pc.header.frame_id = frame;

    const uint32_t n = static_cast<uint32_t>(xyzi.size() / 4);
    const char* names[4] = {"x", "y", "z", "intensity"};
    for (int k = 0; k < 4; ++k) {
        PF f;
        f.name     = names[k];
        f.offset   = static_cast<uint32_t>(k * sizeof(float));
        f.datatype = PF::FLOAT32;
        f.count    = 1;
        pc.fields.push_back(f);
    }
    pc.height       = 1;
    pc.width        = n;
    pc.is_bigendian = false;
    pc.is_dense     = true;
    pc.point_step   = 4 * sizeof(float);
    pc.row_step     = pc.point_step * n;
    pc.data.resize(static_cast<size_t>(pc.row_step));
    std::memcpy(pc.data.data(), xyzi.data(), pc.data.size());
    return pc;
}

struct RawPt { int64_t ts; float x, y, z, intensity; };

} // namespace

bool exportRos2Bag(const RosExportInput& in,
                   const RosExportOptions& opt,
                   std::string& status) {
    const int step = std::max(1, opt.lidarDecim);
    const bool haveTraj = !in.traj.poses.empty();

    bool wantRaw = opt.exportLidarRaw;
    if (wantRaw && !haveTraj) wantRaw = false;  // need poses to undo motion

    rosbag2_storage::StorageOptions so;
    so.uri        = opt.outUri;
    so.storage_id = opt.storageId;
    rosbag2_cpp::ConverterOptions co;
    co.input_serialization_format  = "cdr";
    co.output_serialization_format = "cdr";

    rosbag2_cpp::Writer writer;
    try {
        writer.open(so, co);
    } catch (const std::exception& e) {
        status = std::string("Failed to open bag '") + opt.outUri + "': " + e.what();
        return false;
    }

    // Earliest timestamp in the dataset → stamp for the static transform.
    int64_t startTs = 0;
    if (haveTraj)                  startTs = in.traj.poses.front().ts_ns;
    else if (!in.imageFiles.empty()) startTs = in.imageFiles.begin()->first;

    size_t nTf = 0, nImg = 0, nCloud = 0;
    std::fprintf(stderr, "[RosExport] writing bag '%s' (%s)\n",
                 opt.outUri.c_str(), opt.storageId.c_str());

    try {
        // ── /tf_static : lidar -> camera (from extrinsics) ────────────────────
        if (opt.exportTf && in.calibLoaded) {
            // Pre-create the topic with TRANSIENT_LOCAL durability (matching the
            // standard static_transform_broadcaster) so tf listeners joining late
            // still receive it; otherwise it is offered as VOLATILE and rejected.
            rosbag2_storage::TopicMetadata tm;
            tm.name                 = kTopicTfStatic;
            tm.type                 = "tf2_msgs/msg/TFMessage";
            tm.serialization_format = "cdr";
            tm.offered_qos_profiles = { rclcpp::QoS(1).transient_local() };
            writer.create_topic(tm);

            Eigen::Affine3f T_lc = Eigen::Affine3f::Identity();
            T_lc.linear()      = eulerZYXtoMat3(in.E.rx, in.E.ry, in.E.rz);
            T_lc.translation() = Eigen::Vector3f(in.E.tx, in.E.ty, in.E.tz);

            geometry_msgs::msg::TransformStamped ts;
            ts.header.stamp    = toRosTime(startTs);
            ts.header.frame_id = in.lidarFrame;
            ts.child_frame_id  = in.cameraFrame;
            ts.transform       = toTransform(T_lc);

            tf2_msgs::msg::TFMessage m;
            m.transforms.push_back(ts);
            writer.write(m, kTopicTfStatic, rclcpp::Time(startTs));
        }

        // ── /tf : map -> lidar, one message per trajectory pose ───────────────
        if (opt.exportTf && haveTraj) {
            for (const auto& p : in.traj.poses) {
                geometry_msgs::msg::TransformStamped ts;
                ts.header.stamp    = toRosTime(p.ts_ns);
                ts.header.frame_id = in.mapFrame;
                ts.child_frame_id  = in.lidarFrame;
                ts.transform       = toTransform(p.T);

                tf2_msgs::msg::TFMessage m;
                m.transforms.push_back(ts);
                writer.write(m, kTopicTf, rclcpp::Time(p.ts_ns));
                ++nTf;
            }
        }

        std::fprintf(stderr, "[RosExport] tf: %zu transforms\n", nTf);

        // ── camera images (+ camera_info) ─────────────────────────────────────
        if (opt.exportCamera && !in.imageFiles.empty()) {
            // Rectification maps (built lazily once the image size is known).
            // Mirrors App.cpp: undistort to the same K so that a pinhole
            // projection — which is all RViz uses — lines up with the image.
            const cv::Mat Km = (cv::Mat_<double>(3, 3) <<
                                in.K.fx, 0,       in.K.cx,
                                0,       in.K.fy, in.K.cy,
                                0,       0,       1);
            const cv::Mat Dm = (cv::Mat_<double>(1, 8) <<
                                in.K.k1, in.K.k2, in.K.p1, in.K.p2,
                                in.K.k3, in.K.k4, in.K.k5, in.K.k6);
            cv::Mat    map1, map2;
            bool       mapsReady = false;
            int        camW = 0, camH = 0;
            const bool rectify = opt.undistortCamera && in.calibLoaded;
            // Original jpeg bytes can be copied verbatim only when we neither
            // rectify nor need to re-encode (compressed + no undistort).
            const bool copyJpegBytes = opt.compressCamera && !rectify;

            for (const auto& [ts, path] : in.imageFiles) {
                std::vector<uint8_t> outBytes;  // jpeg, when compressed
                cv::Mat              outImg;     // bgr8, when raw

                if (copyJpegBytes) {
                    std::ifstream f(path, std::ios::binary);
                    if (!f) continue;
                    outBytes.assign(std::istreambuf_iterator<char>(f),
                                    std::istreambuf_iterator<char>());
                    if (outBytes.empty()) continue;
                } else {
                    cv::Mat bgr = cv::imread(path, cv::IMREAD_COLOR);
                    if (bgr.empty()) continue;
                    if (rectify) {
                        if (!mapsReady) {
                            cv::initUndistortRectifyMap(Km, Dm, cv::noArray(), Km,
                                bgr.size(), CV_16SC2, map1, map2);
                            mapsReady = true;
                        }
                        cv::Mat und;
                        cv::remap(bgr, und, map1, map2, cv::INTER_LINEAR);
                        bgr = und;
                    }
                    camW = bgr.cols; camH = bgr.rows;
                    if (opt.compressCamera) {
                        cv::imencode(".jpg", bgr, outBytes);
                    } else {
                        if (!bgr.isContinuous()) bgr = bgr.clone();
                        outImg = bgr;
                    }
                }

                if (opt.compressCamera) {
                    sensor_msgs::msg::CompressedImage img;
                    img.header.stamp    = toRosTime(ts);
                    img.header.frame_id = in.cameraFrame;
                    img.format          = "jpeg";
                    img.data            = std::move(outBytes);
                    writer.write(img, kTopicImgCompressed, rclcpp::Time(ts));
                } else {
                    sensor_msgs::msg::Image img;
                    img.header.stamp    = toRosTime(ts);
                    img.header.frame_id = in.cameraFrame;
                    img.height          = static_cast<uint32_t>(outImg.rows);
                    img.width           = static_cast<uint32_t>(outImg.cols);
                    img.encoding        = "bgr8";
                    img.is_bigendian    = 0;
                    img.step            = static_cast<uint32_t>(outImg.cols * 3);
                    img.data.assign(outImg.datastart, outImg.dataend);
                    writer.write(img, kTopicImgRaw, rclcpp::Time(ts));
                }
                ++nImg;

                // CameraInfo alongside, once we know the resolution.
                if (in.calibLoaded) {
                    if (camW == 0) {  // copy-bytes path: peek dimensions once
                        cv::Mat probe = cv::imread(path, cv::IMREAD_COLOR);
                        if (!probe.empty()) { camW = probe.cols; camH = probe.rows; }
                    }
                    if (camW > 0) {
                        sensor_msgs::msg::CameraInfo ci;
                        ci.header.stamp    = toRosTime(ts);
                        ci.header.frame_id = in.cameraFrame;
                        ci.height          = static_cast<uint32_t>(camH);
                        ci.width           = static_cast<uint32_t>(camW);
                        ci.distortion_model = "rational_polynomial";
                        if (rectify)  // image already rectified → no distortion
                            ci.d = {0, 0, 0, 0, 0, 0, 0, 0};
                        else
                            ci.d = {in.K.k1, in.K.k2, in.K.p1, in.K.p2,
                                    in.K.k3, in.K.k4, in.K.k5, in.K.k6};
                        ci.k = {in.K.fx, 0.f,      in.K.cx,
                                0.f,      in.K.fy, in.K.cy,
                                0.f,      0.f,      1.f};
                        ci.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
                        ci.p = {in.K.fx, 0.f,      in.K.cx, 0.f,
                                0.f,      in.K.fy, in.K.cy, 0.f,
                                0.f,      0.f,      1.f,     0.f};
                        writer.write(ci, kTopicCamInfo, rclcpp::Time(ts));
                    }
                }
            }
        }

        std::fprintf(stderr, "[RosExport] camera: %zu images\n", nImg);

        // ── LiDAR : load chunks → map-frame points → time-windowed clouds ─────
        if ((opt.exportLidarUndistorted || wantRaw) && !in.lidarChunks.empty()) {
            std::vector<RawPt> pts;
            for (const auto& ch : in.lidarChunks) {
                PointCloud pc;
                if (!pc.load(ch.lazPath)) continue;
                for (size_t i = 0; i < pc.points.size(); i += step) {
                    const auto& p = pc.points[i];
                    Eigen::Vector3f pw(p.x, p.y, p.z);
                    if (ch.hasM) pw = ch.M * pw;
                    pts.push_back({p.ts_ns, pw.x(), pw.y(), pw.z(), p.intensity});
                }
            }

            if (!pts.empty()) {
                std::sort(pts.begin(), pts.end(),
                          [](const RawPt& a, const RawPt& b) { return a.ts < b.ts; });
                const int64_t aggNs = std::max<int64_t>(1, (int64_t)(opt.aggregationSec * 1e9));
                const int64_t t0    = pts.front().ts;
                std::fprintf(stderr, "[RosExport] lidar: %zu points, span %.2f s, window %.3f s\n",
                             pts.size(), (pts.back().ts - t0) / 1e9, opt.aggregationSec);

                // cache for the raw (sensor-frame) re-projection
                const TrajPose* lastPose = nullptr;
                Eigen::Affine3f lastInv  = Eigen::Affine3f::Identity();

                size_t i = 0;
                while (i < pts.size()) {
                    const int64_t w        = (pts[i].ts - t0) / aggNs;
                    const int64_t winStamp = t0 + w * aggNs;
                    size_t j = i;
                    while (j < pts.size() && (pts[j].ts - t0) / aggNs == w) ++j;

                    if (opt.exportLidarUndistorted) {
                        std::vector<float> buf;
                        buf.reserve((j - i) * 4);
                        for (size_t k = i; k < j; ++k) {
                            buf.push_back(pts[k].x); buf.push_back(pts[k].y);
                            buf.push_back(pts[k].z); buf.push_back(pts[k].intensity);
                        }
                        writer.write(makeCloud(in.mapFrame, winStamp, buf),
                                     kTopicLidarUndist, rclcpp::Time(winStamp));
                        ++nCloud;
                    }
                    if (wantRaw) {
                        std::vector<float> buf;
                        buf.reserve((j - i) * 4);
                        for (size_t k = i; k < j; ++k) {
                            const TrajPose* p = in.traj.nearest(pts[k].ts);
                            if (p != lastPose) { lastPose = p; lastInv = p->T.inverse(); }
                            Eigen::Vector3f pl = lastInv * Eigen::Vector3f(pts[k].x, pts[k].y, pts[k].z);
                            buf.push_back(pl.x()); buf.push_back(pl.y());
                            buf.push_back(pl.z()); buf.push_back(pts[k].intensity);
                        }
                        writer.write(makeCloud(in.lidarFrame, winStamp, buf),
                                     kTopicLidarRaw, rclcpp::Time(winStamp));
                        ++nCloud;
                    }
                    i = j;
                }
            }
        }
    } catch (const std::exception& e) {
        status = std::string("Export failed while writing: ") + e.what();
        return false;
    }

    writer.close();
    std::fprintf(stderr, "[RosExport] done: %zu tf, %zu img, %zu clouds\n", nTf, nImg, nCloud);
    status = "Wrote bag '" + opt.outUri + "' (" + opt.storageId + "): "
           + std::to_string(nTf)    + " tf, "
           + std::to_string(nImg)   + " img, "
           + std::to_string(nCloud) + " clouds"
           + (opt.exportLidarRaw && !haveTraj ? "  [raw skipped: no trajectory]" : "");
    return true;
}

#endif  // CALIB_ENABLE_ROS_EXPORT
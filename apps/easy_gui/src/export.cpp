#include "export.h"

#include "las14_writer.h"

#include <lidar_odometry_utils.h>
#include <toml_io.h>

#include <Core/session.h>
#include <Core/transformations.h>
#include <HDMapping/HDMAPPING_ConfigureInfo.hpp>

#include <laszip/laszip_api.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>

namespace fs = std::filesystem;

namespace easy_gui
{
    namespace
    {
        constexpr double kCoordinateScale = 0.0001; // as step 2's export (LazWriter), so XYZ quantise identically
        //! A raw point belongs to the last trajectory node before it, if that node is at most this much older. The same
        //! tolerance step 1 applies when it assigns points to nodes (compute_step_1).
        constexpr double kNodeTolerance_s = 0.01;
        constexpr double kRangeBin_m = 0.1;

        //! Extra bytes of every exported point, in this order.
        const std::vector<ExtraBytesAttribute> kAttributes = {
            { ExtraBytesAttribute::Type::F32, "range_m", "3D distance from the lidar [m]" },
            { ExtraBytesAttribute::Type::U8, "point_origin", "0 step 1, 1 near, 2 gap point" },
        };

#pragma pack(push, 1)
        struct ExtraBytes
        {
            float range_m;
            uint8_t point_origin;
        };
#pragma pack(pop)
        static_assert(sizeof(ExtraBytes) == 5);

        //! Streaming laszip reader: one point at a time, positions with the header's scale and offset applied.
        class LazReader
        {
        public:
            ~LazReader()
            {
                if (reader_)
                {
                    laszip_close_reader(reader_);
                    laszip_destroy(reader_);
                }
            }

            bool open(const fs::path& file, std::string& error)
            {
                laszip_BOOL compressed = 0;
                if (laszip_create(&reader_) || laszip_open_reader(reader_, file.string().c_str(), &compressed) ||
                    laszip_get_header_pointer(reader_, &header_) || laszip_get_point_pointer(reader_, &point_))
                {
                    error = "could not open '" + file.string() + "'";
                    return false;
                }
                count_ = header_->number_of_point_records ? header_->number_of_point_records : header_->extended_number_of_point_records;
                return true;
            }

            uint64_t count() const
            {
                return count_;
            }

            bool next(std::string& error)
            {
                if (laszip_read_point(reader_))
                {
                    error = "could not read a point";
                    return false;
                }
                return true;
            }

            Eigen::Vector3d position() const
            {
                return { header_->x_offset + header_->x_scale_factor * static_cast<double>(point_->X),
                         header_->y_offset + header_->y_scale_factor * static_cast<double>(point_->Y),
                         header_->z_offset + header_->z_scale_factor * static_cast<double>(point_->Z) };
            }

            const laszip_point& point() const
            {
                return *point_;
            }

        private:
            laszip_POINTER reader_ = nullptr;
            laszip_header* header_ = nullptr;
            laszip_point* point_ = nullptr;
            uint64_t count_ = 0;
        };

        //! One trajectory node with everything the export needs.
        struct Node
        {
            double time_s; // lidar clock (Unix epoch seconds on Mandeye)
            size_t scan;
            Eigen::Affine3d global; // scan pose (session_poses.mrp) x node pose relative to the scan
            //! Last node of a step 1 piece (worker chunk): step 1 takes the points in [first node, last node) of each
            //! piece, so raw points from this node until the next piece's first node are not in its output.
            bool piece_end;
        };

        void addRange(ExportStats& stats, double range)
        {
            const uint64_t total = stats.odometry_points + stats.near_points + stats.check_band_points;
            if (total == 0 || range < stats.range_min_m)
                stats.range_min_m = range;
            if (total == 0 || range > stats.range_max_m)
                stats.range_max_m = range;
            const size_t bin = std::min(stats.range_histogram.size() - 1, static_cast<size_t>(std::max(0.0, range) / kRangeBin_m));
            stats.range_histogram[bin]++;
        }

        bool writeTrajectory(
            const fs::path& file, const Session& session, const Eigen::Vector3d& offset, ExportStats& stats, std::string& error)
        {
            std::ofstream out(file);
            if (!out)
            {
                error = "Could not create the trajectory file '" + file.string() + "'.";
                return false;
            }
            out << "timestamp_unix_s,x,y,z,qw,qx,qy,qz,om_deg,fi_deg,ka_deg,scan_index,host_clock_unix_s,imu_om_deg,imu_fi_deg,imu_ka_"
                   "deg\n";
            const auto& clouds = session.point_clouds_container.point_clouds;
            char line[512];
            for (size_t i = 0; i < clouds.size(); i++)
            {
                for (const auto& node : clouds[i].local_trajectory)
                {
                    // As save_trajectories() in step 2: scan pose x node pose, plus the session offset.
                    Eigen::Affine3d pose = clouds[i].m_pose * node.m_pose;
                    pose.translation() += offset;
                    const Eigen::Quaterniond q(pose.rotation());
                    const TaitBryanPose tb = pose_tait_bryan_from_affine_matrix(pose);
                    // node.timestamps.second is the IMU file's timestampUnix column: the scanner computer's clock in
                    // milliseconds (step 1 passes it through unchanged).
                    std::snprintf(
                        line,
                        sizeof(line),
                        "%.6f,%.4f,%.4f,%.4f,%.9f,%.9f,%.9f,%.9f,%.6f,%.6f,%.6f,%zu,%.3f,%.6f,%.6f,%.6f\n",
                        node.timestamps.first / 1e9,
                        pose.translation().x(),
                        pose.translation().y(),
                        pose.translation().z(),
                        q.w(),
                        q.x(),
                        q.y(),
                        q.z(),
                        rad2deg(tb.om),
                        rad2deg(tb.fi),
                        rad2deg(tb.ka),
                        i,
                        node.timestamps.second / 1e3,
                        rad2deg(node.imu_om_fi_ka.x()),
                        rad2deg(node.imu_om_fi_ka.y()),
                        rad2deg(node.imu_om_fi_ka.z()));
                    out << line;
                    stats.trajectory_nodes++;
                }
            }
            out.close();
            if (!out)
            {
                error = "Could not write the trajectory file '" + file.string() + "' (is the disk full?).";
                return false;
            }
            return true;
        }

        std::string formatCount(uint64_t n)
        {
            std::string s = std::to_string(n);
            for (int i = static_cast<int>(s.size()) - 3; i > 0; i -= 3)
                s.insert(static_cast<size_t>(i), ",");
            return s;
        }

        bool writeReadme(const ExportRequest& request, const ExportStats& stats, std::string& error)
        {
            std::ofstream out(request.readme_file);
            if (!out)
            {
                error = "Could not create '" + request.readme_file.string() + "'.";
                return false;
            }
            const std::string laz = request.laz_file.filename().string();
            const std::string csv = request.trajectory_file.filename().string();
            char inner[32], near_min[32], output_filter[32], outer[32];
            std::snprintf(inner, sizeof(inner), "%.2f m", stats.filter_threshold_xy_inner);
            std::snprintf(outer, sizeof(outer), "%.1f m", stats.filter_threshold_xy_outer);
            std::snprintf(near_min, sizeof(near_min), "%.2f m", request.near_min_range_m);
            std::snprintf(output_filter, sizeof(output_filter), "%.2f m", stats.threshould_output_filter);

            out << "Easy Forest HDMapping results\n"
                << "==========================\n\n"
                << "Made by Easy Forest HDMapping " << HDMAPPING_CONFIGURE_PROJECT_VERSION << " from the recording\n"
                << "  " << request.recording.laz_files.front().parent_path().string() << "\n"
                << "with lidar odometry (precision forestry settings). No loop closure and no georeferencing: coordinates\n"
                << "are in metres in a local frame that starts at the scanner's first position (z up, from the IMU).\n\n"
                << "Files\n-----\n"
                << "  " << laz << "  point cloud (LAZ, LAS 1.4, point format 6), "
                << formatCount(stats.odometry_points + stats.near_points + stats.gap_points + stats.check_band_points) << " points\n"
                << "  " << csv << "  trajectory of the scanner (CSV), " << formatCount(stats.trajectory_nodes) << " poses\n"
                << "  easy_gui.log  processing log\n"
                << "  work/lio_session/  lidar odometry session; open session.mjs in multi_view_tls_registration_step_2\n"
                << "                     to refine or georeference the result\n\n"
                << "Time\n----\n"
                << "All times are Unix epoch seconds (UTC, seconds since 1970-01-01) as counted by the scanner's lidar\n"
                << "clock. They are NOT GPS week time and NOT adjusted standard GPS time, even though the LAS field is called\n"
                << "gps_time (the LAS header's global encoding bit 0 is 0 accordingly).\n"
                << (request.place_by_raw_time
                        ? "Every point carries its raw measurement time (experimental \"place by raw time\" export option).\n\n"
                        : "Lidar odometry's points (point_origin = 0) carry lidar odometry's time, not the raw measurement time:\n"
                          "within each 0.1 s piece of the recording, odometry spreads its points' times evenly between the first and\n"
                          "last pose of that piece. That is close to the measurement time (typically 1-4 ms off, at most about\n"
                          "20 ms), but it cannot be used to match points to the raw files. Near points and gap points\n"
                          "(point_origin = 1, 2) carry their raw measurement time.\n\n")
                << "Point cloud fields (" << laz << ")\n"
                << std::string(21 + laz.size(), '-') << "\n"
                << "  X, Y, Z          position [m], 0.1 mm resolution\n"
                << "  intensity        lidar reflectivity as recorded (Livox: 0-255; usually 0-160)\n"
                << "  gps_time         time [Unix epoch seconds], see \"Time\"\n"
                << "  point_source_id  index of the lidar odometry scan (scan_lio_<index>.laz in work/lio_session, and\n"
                << "                   scan_index in " << csv << "); about 5 s of recording each\n"
                << "  user_data        lidar id (" << request.lidar_id << " for every point of this single-lidar recording)\n"
                << "  return_number, number_of_returns  always 1; classification  always 0 (not classified)\n"
                << "  range_m          extra bytes, float32: 3D distance from the lidar at the moment of measurement [m];\n"
                << "                   can exceed " << outer << " for steep points, as that limit is horizontal\n"
                << "  point_origin     extra bytes, uint8: where the point comes from (see below)\n"
                << (request.place_by_raw_time ? "                   0 = inside lidar odometry's distance limits, placed by raw time\n"
                                              : "                   0 = lidar odometry's own point\n")
                << "                   1 = near point, 2 = gap point (both added by Easy Forest HDMapping)\n"
                << (stats.check_band_points ? "                   3 = TEST MODE alignment-check point (duplicate of an odometry point)\n"
                                            : "")
                << "\n"
                << "Where the points come from\n--------------------------\n"
                << "Lidar odometry only uses and exports points whose horizontal distance from the lidar (in the lidar's own\n"
                << "frame) is between " << inner << " and " << outer << " and whose 3D distance is above " << output_filter
                << ", and it works in\n"
                << "0.1 s pieces: it skips the raw points measured after the last pose of a piece and before the first pose\n"
                << "of the next one (about 5 % of the recording time). Easy Forest HDMapping adds both kinds of missing points back from\n"
                << "the raw recording, so the cloud has no hole around the walking path and nothing is missing between the\n"
                << "pieces.\n\n"
                << "point_origin = 0 (" << formatCount(stats.odometry_points) << " points): "
                << (request.place_by_raw_time ? "points inside lidar odometry's limits, taken from the raw recording.\n"
                                              : "lidar odometry's own points, as it placed them.\n")
                << "point_origin = 1, near points (" << formatCount(stats.near_points) << "): raw points that\n"
                << "  - are at least " << near_min << " from the lidar in 3D (closer returns are mostly the operator's body and the\n"
                << "    scanner handle; the limit is the advanced setting \"Closest points kept\"), and\n"
                << "  - lidar odometry left out because of distance: horizontal distance at most " << inner << ", or 3D\n"
                << "    distance at most " << output_filter << ", and horizontal distance below " << outer << ".\n"
                << "point_origin = 2, gap points (" << formatCount(stats.gap_points) << "): raw points inside lidar odometry's distance\n"
                << "  limits that it skipped between two 0.1 s pieces.\n"
                << "Near and gap points are placed with the trajectory pose lidar odometry uses for points measured at the\n"
                << "same time (the last pose before the point), so they line up with lidar odometry's points"
                << (request.place_by_raw_time
                        ? ".\n"
                        : " to within a few\nmillimetres (rarely more than 2 cm, because of lidar odometry's evenly spread times).\n")
                << (request.place_by_raw_time
                        ? "With the experimental \"place by raw time\" option, point_origin = 0 points are placed the same way.\n"
                        : "")
                << "Points at exactly (0, 0, 0) or with time 0 are \"no return\" records and are never exported, and points\n"
                << "measured where the trajectory has no pose within 10 ms (start and end of the recording) are left out.\n\n"
                << "Trajectory fields (" << csv << ", comma-separated, one header row)\n"
                << std::string(53 + csv.size(), '-') << "\n"
                << "  timestamp_unix_s   time of the pose [Unix epoch seconds], same clock as the point cloud's gps_time\n"
                << "  x, y, z            position of the scanner (IMU) [m], same frame as the point cloud\n"
                << "  qw, qx, qy, qz     orientation as a unit quaternion (rotates scanner coordinates into the frame)\n"
                << "  om_deg, fi_deg, ka_deg  the same orientation as HDMapping's Tait-Bryan angles [deg]\n"
                << "  scan_index         lidar odometry scan the pose belongs to (point_source_id in the point cloud)\n"
                << "  host_clock_unix_s  the scanner computer's own clock at that IMU sample [Unix epoch seconds,\n"
                << "                     millisecond resolution]; it is not synchronised with timestamp_unix_s\n"
                << "  imu_om_deg, imu_fi_deg, imu_ka_deg  orientation from the IMU alone, before lidar odometry [deg]\n";
            out.close();
            if (!out)
            {
                error = "Could not write '" + request.readme_file.string() + "'.";
                return false;
            }
            return true;
        }
    } // namespace

    fs::path findSessionParameterFile(const fs::path& session_folder)
    {
        std::error_code ec;
        fs::path found;
        for (const auto& entry : fs::directory_iterator(session_folder, ec))
        {
            const std::string name = entry.path().filename().string();
            if (name.starts_with("HDMapping_params_") && entry.path().extension() == ".toml" && (found.empty() || entry.path() > found))
                found = entry.path();
        }
        return found;
    }

    bool exportSession(
        const ExportRequest& request,
        const std::atomic<bool>& cancel,
        const ExportProgress& progress,
        ExportStats& stats,
        std::string& error)
    {
        stats = ExportStats{};
        const auto cancelled = [&]()
        {
            if (cancel)
                error = "Export was cancelled.";
            return cancel.load();
        };

        // --- the parameters step 1 ran with: its distance filters define which raw points are "near points"
        const fs::path parameter_file = findSessionParameterFile(request.session_file.parent_path());
        auto params = std::make_unique<LidarOdometryParams>();
        TomlIO toml_io;
        if (parameter_file.empty() || !toml_io.LoadParametersFromTomlFile(parameter_file.string(), *params))
        {
            error = "The lidar odometry result is incomplete: its settings file (HDMapping_params_*.toml) is missing in '" +
                request.session_file.parent_path().string() + "'.";
            return false;
        }
        stats.filter_threshold_xy_inner = params->filter_threshold_xy_inner;
        stats.filter_threshold_xy_outer = params->filter_threshold_xy_outer;
        stats.threshould_output_filter = params->threshould_output_filter;
        const int poses_per_piece = params->threshold_nr_poses;
        stats.threshold_nr_poses = poses_per_piece;
        if (poses_per_piece <= 0)
        {
            error = "The lidar odometry settings file has no valid threshold_nr_poses.";
            return false;
        }

        // --- poses and trajectories exactly as step 2 loads them; the points are streamed below instead
        progress(0.0, "Reading the lidar odometry result");
        Session session;
        session.load_cache_mode = true;
        if (!session.load(request.session_file.string(), false, 0.0, 0.0, 0.0, false) ||
            session.point_clouds_container.point_clouds.empty())
        {
            error = "Could not read the lidar odometry result '" + request.session_file.string() + "'.";
            return false;
        }
        const auto& clouds = session.point_clouds_container.point_clouds;
        const Eigen::Vector3d offset = session.point_clouds_container.offset;
        stats.scans = clouds.size();
        if (clouds.size() > 65535)
        {
            error = "The recording is too long for one point cloud file (more than 65535 lidar odometry scans).";
            return false;
        }

        // --- calibration, as load_data builds it: lidar frame -> the frame step 1 works in (identity without a file)
        Eigen::Affine3d calibration = Eigen::Affine3d::Identity();
        {
            const auto id_to_sn = MLvxCalib::GetIdToSnMapping(request.recording.sn_file.string());
            const auto combined = MLvxCalib::CombineIntoCalibration(
                id_to_sn,
                request.recording.calibration_file.empty()
                    ? std::unordered_map<std::string, Eigen::Affine3d>{}
                    : MLvxCalib::GetCalibrationFromFile(request.recording.calibration_file.string()));
            if (const auto it = combined.find(request.lidar_id); it != combined.end())
                calibration = it->second;
        }
        const Eigen::Affine3d lidar_from_sensor = calibration.inverse();

        // --- all trajectory nodes in time order, for placing the near points
        std::vector<Node> nodes;
        for (size_t i = 0; i < clouds.size(); i++)
        {
            if (clouds[i].local_trajectory.empty())
            {
                error = "The lidar odometry result is incomplete: scan " + std::to_string(i) + " has no trajectory.";
                return false;
            }
            // Every step 1 piece has exactly threshold_nr_poses nodes, and save_result concatenates whole pieces into a
            // scan, so the piece boundaries are at multiples of it within each scan.
            const auto& trajectory = clouds[i].local_trajectory;
            for (size_t j = 0; j < trajectory.size(); j++)
                nodes.push_back(
                    { trajectory[j].timestamps.first / 1e9,
                      i,
                      clouds[i].m_pose * trajectory[j].m_pose,
                      static_cast<int>(j % poses_per_piece) == poses_per_piece - 1 });
        }
        std::stable_sort(
            nodes.begin(),
            nodes.end(),
            [](const Node& a, const Node& b)
            {
                return a.time_s < b.time_s;
            });
        std::vector<double> node_times(nodes.size());
        for (size_t i = 0; i < nodes.size(); i++)
            node_times[i] = nodes[i].time_s;

        // --- output files
        std::error_code ec;
        fs::create_directories(request.laz_file.parent_path(), ec);
        Las14Writer writer;
        if (!writer.open(
                request.laz_file.string(),
                kAttributes,
                offset,
                kCoordinateScale,
                std::string("HDMapping ") + HDMAPPING_CONFIGURE_PROJECT_VERSION,
                error))
        {
            error = "Could not create the point cloud file '" + request.laz_file.string() + "' (" + error + ").";
            return false;
        }
        const uint8_t user_data = static_cast<uint8_t>(request.lidar_id);
        const size_t scans_to_copy = request.place_by_raw_time ? 0 : clouds.size();
        const double work_units = static_cast<double>(scans_to_copy + request.recording.pairs);
        auto writePoint =
            [&](const Eigen::Vector3d& position, double time_s, uint16_t intensity, size_t scan, double range, PointOrigin origin)
        {
            const ExtraBytes extra{ static_cast<float>(range), origin };
            Las14Point p;
            p.position = position;
            p.gps_time = time_s;
            p.intensity = intensity;
            p.point_source_id = static_cast<uint16_t>(scan);
            p.user_data = user_data;
            p.extra_bytes = reinterpret_cast<const uint8_t*>(&extra);
            addRange(stats, range);
            (origin == kStep1           ? stats.odometry_points
                 : origin == kNearPoint ? stats.near_points
                 : origin == kGapPoint  ? stats.gap_points
                                        : stats.check_band_points)++;
            return writer.write(p, error);
        };

        // --- 1. step 1's points, one scan at a time (not with place_by_raw_time: pass 2 places them from the raw files)
        for (size_t i = 0; i < scans_to_copy; i++)
        {
            if (cancelled())
                return false;
            progress(
                static_cast<double>(i) / work_units,
                "Writing the point cloud (scan " + std::to_string(i + 1) + " of " + std::to_string(clouds.size()) + ")");

            const auto& pc = clouds[i];
            std::vector<double> times; // node times in step 1's unit (seconds x 1e9), as the points' gps_time
            std::vector<Eigen::Affine3d> sensor_from_local;
            for (const auto& n : pc.local_trajectory)
            {
                times.push_back(n.timestamps.first);
                sensor_from_local.push_back(n.m_pose.inverse());
            }

            LazReader reader;
            if (!reader.open(pc.file_name, error))
            {
                error = "The lidar odometry result is incomplete: " + error + ".";
                return false;
            }
            for (uint64_t k = 0; k < reader.count(); k++)
            {
                if (!reader.next(error))
                {
                    error = "Could not read '" + pc.file_name + "': " + error + ".";
                    return false;
                }
                const laszip_point& rp = reader.point();
                if (rp.gps_time == 0.0) // step 2's export skips these too
                    continue;
                // Step 2 subtracts the session offset from the loaded points and adds it back after the pose.
                const Eigen::Vector3d local = reader.position() - offset;

                // Range: the point in the sensor frame, through the node step 1 used for it (the last node before the
                // point's time), then into the lidar frame.
                const auto it = std::lower_bound(times.begin(), times.end(), rp.gps_time);
                const size_t node = it == times.begin() ? 0 : static_cast<size_t>(it - times.begin()) - 1;
                const double range = (lidar_from_sensor * (sensor_from_local[node] * local)).norm();

                if (!writePoint(pc.m_pose * local + offset, rp.gps_time / 1e9, rp.intensity, i, range, kStep1))
                {
                    error = "Could not write the point cloud file (is the disk full?): " + error + ".";
                    return false;
                }
            }
        }

        // --- 2. near and gap points (with place_by_raw_time: all points) from the raw recording, one file at a time
        const double inner = params->filter_threshold_xy_inner;
        const double outer = params->filter_threshold_xy_outer;
        const double output_filter = params->threshould_output_filter;
        for (size_t f = 0; f < request.recording.pairs; f++)
        {
            if (cancelled())
                return false;
            progress(
                static_cast<double>(scans_to_copy + f) / work_units,
                std::string(request.place_by_raw_time ? "Writing the point cloud" : "Adding the near and gap points") + " (file " +
                    std::to_string(f + 1) + " of " + std::to_string(request.recording.pairs) + ")");

            const fs::path& raw_file = request.recording.laz_files[f];
            LazReader reader;
            if (!reader.open(raw_file, error))
            {
                error = "Could not read the recording file '" + raw_file.filename().string() + "'.";
                return false;
            }
            for (uint64_t k = 0; k < reader.count(); k++)
            {
                if (!reader.next(error))
                {
                    error = "Could not read the recording file '" + raw_file.filename().string() + "': " + error + ".";
                    return false;
                }
                stats.raw_points_read++;
                const laszip_point& rp = reader.point();
                if ((rp.X == 0 && rp.Y == 0 && rp.Z == 0) || rp.gps_time == 0.0)
                {
                    stats.raw_no_return++;
                    continue;
                }
                const Eigen::Vector3d lidar = reader.position();
                const double range_xy = std::hypot(lidar.x(), lidar.y());
                const double range = lidar.norm();
                const Eigen::Vector3d sensor = calibration * lidar;

                // Step 1's distance filters: load_point_cloud keeps inner < range_xy < outer, points_to_vector keeps
                // |sensor| > threshould_output_filter. Near points are exactly the complement, at close range.
                const bool step1_keeps = range_xy > inner && range_xy < outer && sensor.norm() > output_filter;
                if (!step1_keeps && (range < request.near_min_range_m || range_xy >= outer))
                    continue;

                const double t = rp.gps_time;
                const auto it = std::lower_bound(node_times.begin(), node_times.end(), t);
                if (it == node_times.begin() || t - *(it - 1) > kNodeTolerance_s)
                {
                    if (!step1_keeps || request.place_by_raw_time)
                        stats.raw_outside_trajectory++;
                    continue;
                }
                // Placed with the last node before the point (step 1's rule). Whether step 1 has the point depends on
                // the last node at or before it: a piece's last node starts a gap step 1 skips.
                const Node& node = nodes[static_cast<size_t>(it - node_times.begin()) - 1];
                const size_t at_or_before =
                    static_cast<size_t>(std::upper_bound(node_times.begin(), node_times.end(), t) - node_times.begin()) - 1;
                const bool in_gap = nodes[at_or_before].piece_end;

                PointOrigin origin;
                if (!step1_keeps)
                    origin = kNearPoint;
                else if (in_gap)
                    origin = kGapPoint;
                else if (request.place_by_raw_time)
                    origin = kStep1;
                else if (range_xy < request.check_band_outer_m)
                    origin = kCheckBand;
                else
                    continue; // step 1 has this point: exported from scan_lio_N.laz in pass 1

                if (!writePoint(node.global * sensor + offset, t, rp.intensity, node.scan, range, origin))
                {
                    error = "Could not write the point cloud file (is the disk full?): " + error + ".";
                    return false;
                }
            }
        }

        if (!writer.close(error))
        {
            error = "Could not finish the point cloud file (is the disk full?): " + error + ".";
            return false;
        }
        stats.laz_bytes = fs::file_size(request.laz_file, ec);

        progress(1.0, "Writing the trajectory");
        if (!writeTrajectory(request.trajectory_file, session, offset, stats, error))
            return false;
        stats.trajectory_bytes = fs::file_size(request.trajectory_file, ec);
        return writeReadme(request, stats, error);
    }
} // namespace easy_gui

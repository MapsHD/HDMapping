#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <imgui_internal.h>

#include <ImGuizmo.h>

// #define GLEW_STATIC
// #include <GL/glew.h>
#include <GL/freeglut.h>
#include <portable-file-dialogs.h>

#include "lidar_odometry.h"
#include "lidar_odometry_utils.h"
#include <export_laz.h>
#include <registration_plane_feature.h>

#include <utils.hpp>

#include "toml_io.h"
#include <HDMapping/Version.hpp>
#include <chrono>
#include <ctime>
#include <export_laz.h>
#include <mutex>
#include <pfd_wrapper.hpp>
#include <session.h>

#ifdef _WIN32
#include "resource.h"
#include <windows.h>

#endif

///////////////////////////////////////////////////////////////////////////////////

// This is LiDAR odometry (step 1)
// This program calculates trajectory based on IMU and LiDAR data provided by MANDEYE mobile mapping system
// https://github.com/JanuszBedkowski/mandeye_controller The output is a session proving trajekctory and point clouds that can be  further
// processed by "multi_view_tls_registration" program.

// #define SAMPLE_PERIOD (1.0 / 200.0)

std::string winTitle = std::string("Step 1 (Lidar odometry) ") + HDMAPPING_VERSION_STRING;

std::vector<std::string> infoLines = {
    "This program is first step in MANDEYE process.",
    "",
    "It results trajectory and point clouds as single session for 'multi_view_tls_registration_step_2' program.",
    "",
    "Next step will be to load session.json file with 'multi_view_tls_registration_step_2' program."
};

// App specific shortcuts (Type and Shortcut are just for easy reference)
static const std::vector<ShortcutEntry> appShortcuts = { { "Normal keys", "A", "" },
                                                         { "", "Ctrl+A", "" },
                                                         { "", "B", "" },
                                                         { "", "Ctrl+B", "" },
                                                         { "", "C", "" },
                                                         { "", "Ctrl+C", "" },
                                                         { "", "D", "" },
                                                         { "", "Ctrl+D", "" },
                                                         { "", "E", "" },
                                                         { "", "Ctrl+E", "" },
                                                         { "", "F", "" },
                                                         { "", "Ctrl+F", "" },
                                                         { "", "G", "" },
                                                         { "", "Ctrl+G", "" },
                                                         { "", "H", "" },
                                                         { "", "Ctrl+H", "" },
                                                         { "", "I", "" },
                                                         { "", "Ctrl+I", "" },
                                                         { "", "J", "" },
                                                         { "", "Ctrl+K", "" },
                                                         { "", "K", "" },
                                                         { "", "Ctrl+K", "" },
                                                         { "", "L", "" },
                                                         { "", "Ctrl+L", "" },
                                                         { "", "M", "" },
                                                         { "", "Ctrl+M", "" },
                                                         { "", "N", "" },
                                                         { "", "Ctrl+N", "" },
                                                         { "", "O", "" },
                                                         { "", "Ctrl+O", "Open data" },
                                                         { "", "P", "" },
                                                         { "", "Ctrl+P", "" },
                                                         { "", "Q", "" },
                                                         { "", "Ctrl+Q", "" },
                                                         { "", "R", "" },
                                                         { "", "Ctrl+R", "" },
                                                         { "", "Shift+R", "" },
                                                         { "", "S", "" },
                                                         { "", "Ctrl+S", "" },
                                                         { "", "Ctrl+Shift+S", "" },
                                                         { "", "T", "" },
                                                         { "", "Ctrl+T", "" },
                                                         { "", "U", "" },
                                                         { "", "Ctrl+U", "" },
                                                         { "", "V", "" },
                                                         { "", "Ctrl+V", "" },
                                                         { "", "W", "" },
                                                         { "", "Ctrl+W", "" },
                                                         { "", "X", "" },
                                                         { "", "Ctrl+X", "" },
                                                         { "", "Y", "" },
                                                         { "", "Ctrl+Y", "" },
                                                         { "", "Z", "" },
                                                         { "", "Ctrl+Z", "" },
                                                         { "", "Shift+Z", "" },
                                                         { "", "1-9", "" },
                                                         { "Special keys", "Up arrow", "" },
                                                         { "", "Shift + up arrow", "" },
                                                         { "", "Ctrl + up arrow", "" },
                                                         { "", "Down arrow", "" },
                                                         { "", "Shift + down arrow", "" },
                                                         { "", "Ctrl + down arrow", "" },
                                                         { "", "Left arrow", "" },
                                                         { "", "Shift + left arrow", "" },
                                                         { "", "Ctrl + left arrow", "" },
                                                         { "", "Right arrow", "" },
                                                         { "", "Shift + right arrow", "" },
                                                         { "", "Ctrl + right arrow", "" },
                                                         { "", "Pg down", "" },
                                                         { "", "Pg up", "" },
                                                         { "", "- key", "" },
                                                         { "", "+ key", "" },
                                                         { "Mouse related", "Left click + drag", "" },
                                                         { "", "Right click + drag", "n" },
                                                         { "", "Scroll", "" },
                                                         { "", "Shift + scroll", "" },
                                                         { "", "Shift + drag", "" },
                                                         { "", "Ctrl + left click", "" },
                                                         { "", "Ctrl + right click", "" },
                                                         { "", "Ctrl + middle click", "" } };

namespace fs = std::filesystem;

bool is_settings_gui = false;
bool full_debug_messages = false;
NDT ndt;
bool show_reference_buckets = true;
bool show_reference_points = false;
int dec_reference_points = 100;
bool show_initial_points = true;
bool show_trajectory = true;
bool show_trajectory_as_axes = false;
bool show_covs = false;
int dec_covs = 10;
bool simple_gui = true;
bool step_1_done = false;
bool step_2_done = false;
bool step_3_done = false;
bool calculations_failed = false;

int lastPar = 1;

std::vector<WorkerData> worker_data;

std::string working_directory = "";
bool initial_transformation_gizmo = false;

float m_gizmo[] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

float x_displacement = 0.01;

int index_begin = -1;
int index_end = -1;
bool gizmo_stretch_interval = false;
Eigen::Affine3d stretch_gizmo_m = Eigen::Affine3d::Identity();

LidarOdometryParams params;
std::vector<std::string> csv_files;
std::vector<std::string> sn_files;
std::string imuSnToUse;
Session session;

std::vector<std::vector<Point3Di>> pointsPerFile;
Imu imu_data;
Trajectory trajectory;
std::atomic<bool> loRunning{ false };
std::atomic<float> loProgress{ 0.0 };
std::atomic<bool> loPause{ false };
std::chrono::time_point<std::chrono::system_clock> loStartTime;
std::atomic<double> loElapsedSeconds{ 0.0 };
std::atomic<double> loEstimatedTimeRemaining{ 0.0 };

fs::path outwd;

#if _WIN32
#define DEFAULT_PATH "C:\\"
#else
#define DEFAULT_PATH "~"
#endif

///////////////////////////////////////////////////////////////////////////////////

void set_lidar_odometry_default_params(LidarOdometryParams& params)
{
    params.decimation = 0.01;
    params.in_out_params_indoor.resolution_X = 0.1;
    params.in_out_params_indoor.resolution_Y = 0.1;
    params.in_out_params_indoor.resolution_Z = 0.1;

    params.in_out_params_outdoor.resolution_X = 0.3;
    params.in_out_params_outdoor.resolution_Y = 0.3;
    params.in_out_params_outdoor.resolution_Z = 0.3;

    params.filter_threshold_xy_inner = 0.3;
    params.filter_threshold_xy_outer = 70.0;
    params.threshould_output_filter = 0.3;

    params.use_robust_and_accurate_lidar_odometry = false;
    params.distance_bucket = 0.2;
    params.polar_angle_deg = 10.0;
    params.azimutal_angle_deg = 10.0;
    params.robust_and_accurate_lidar_odometry_iterations = 20;

    params.max_distance_lidar = 70.0;
    params.nr_iter = 1000;
    params.sliding_window_trajectory_length_threshold = 200;
    params.real_time_threshold_seconds = 10;
}

// Helper function to format time in human-readable format
std::string formatTime(double seconds)
{
    if (seconds < 0)
        return "Calculating...";

    int hours = static_cast<int>(seconds / 3600);
    int minutes = static_cast<int>((seconds - hours * 3600) / 60);
    int secs = static_cast<int>(seconds - hours * 3600 - minutes * 60);

    std::ostringstream oss;

    if (hours > 0)
    {
        oss << std::setw(2) << std::setfill('0') << hours << "h ";
    }
    if (minutes > 0 || hours > 0)
    {
        oss << std::setw(2) << std::setfill('0') << minutes << "m ";
    }

    oss << std::setw(2) << std::setfill('0') << secs << "s";

    return oss.str();
}

// Helper function to format estimated completion time
std::string formatCompletionTime(double remainingSeconds)
{
    if (remainingSeconds < 0)
        return "Calculating...";

    auto now = std::chrono::system_clock::now();
    auto estimatedCompletion = now + std::chrono::seconds(static_cast<long long>(remainingSeconds));
    auto completion_time_t = std::chrono::system_clock::to_time_t(estimatedCompletion);

    // Format as HH:MM:SS - Cross-platform time formatting
    struct tm completion_tm;
#ifdef _WIN32
    localtime_s(&completion_tm, &completion_time_t);
#else
    localtime_r(&completion_time_t, &completion_tm);
#endif

    char timeStr[32];
    strftime(timeStr, sizeof(timeStr), "%H:%M", &completion_tm);

    return std::string(timeStr);
}

std::vector<std::vector<Point3Di>> get_batches_of_points(std::string laz_file, int point_count_threshold, std::vector<Point3Di> prev_points)
{
    std::vector<std::vector<Point3Di>> res_points;
    std::vector<Point3Di> points = load_point_cloud(laz_file, false, 0, 10000, {});

    std::vector<Point3Di> tmp_points = prev_points;
    int counter = tmp_points.size();
    for (size_t i = 0; i < points.size(); i++)
    {
        counter++;
        tmp_points.push_back(points[i]);
        if (counter > point_count_threshold)
        {
            res_points.push_back(tmp_points);
            tmp_points.clear();
            counter = 0;
        }
    }

    if (tmp_points.size() > 0)
    {
        res_points.push_back(tmp_points);
    }
    return res_points;
}

int get_index(set<int> s, int k)
{
    int index = 0;
    for (auto u : s)
    {
        if (u == k)
        {
            return index;
        }
        index++;
    }
    return -1;
}

void find_best_stretch(
    std::vector<Point3Di> points, std::vector<double> timestamps, std::vector<Eigen::Affine3d> poses, std::string fn1, std::string fn2)
{
    for (size_t i = 0; i < points.size(); i++)
    {
        auto lower = std::lower_bound(timestamps.begin(), timestamps.end(), points[i].timestamp);
        points[i].index_pose = std::distance(timestamps.begin(), lower);
        // std::cout << "points[i].timestamp " << points[i].timestamp << " timestamps " << timestamps[points[i].index_pose] << std::endl;
    }

    std::set<int> indexes;

    for (size_t i = 0; i < points.size(); i++)
    {
        indexes.insert(points[i].index_pose);
    }
    // build trajectory
    std::vector<Eigen::Affine3d> trajectory;
    std::vector<double> ts;

    for (auto& s : indexes)
    {
        trajectory.push_back(poses[s]);
        ts.push_back(timestamps[s]);
    }

    std::cout << "trajectory.size() " << trajectory.size() << std::endl;
    // Sleep(2000);

    std::vector<Point3Di> points_reindexed = points;
    for (size_t i = 0; i < points_reindexed.size(); i++)
    {
        points_reindexed[i].index_pose = get_index(indexes, points[i].index_pose);
    }
    ///
    std::vector<Eigen::Affine3d> best_trajectory = trajectory;
    unsigned long long min_buckets = ULLONG_MAX;

    for (double x = 0.0; x < 0.2; x += 0.0005)
    {
        std::vector<Eigen::Affine3d> trajectory_stretched;

        Eigen::Affine3d m_x_offset = Eigen::Affine3d::Identity();
        m_x_offset(0, 3) = x;

        Eigen::Affine3d m = trajectory[0];
        trajectory_stretched.push_back(m);
        for (size_t i = 1; i < trajectory.size(); i++)
        {
            Eigen::Affine3d m_update = trajectory[i - 1].inverse() * trajectory[i] * (m_x_offset);
            m = m * m_update;
            trajectory_stretched.push_back(m);
        }

        NDT::GridParameters rgd_params;
        rgd_params.resolution_X = 0.3;
        rgd_params.resolution_Y = 0.3;
        rgd_params.resolution_Z = 0.3;
        NDTBucketMapType my_buckets;

        std::vector<Point3Di> points_global = points_reindexed;

        std::vector<Point3Di> points_global2;

        for (auto& p : points_global)
        {
            if (p.point.z() > 0)
            {
                p.point = trajectory_stretched[p.index_pose] * p.point;
                points_global2.push_back(p);
                // if (p.point.norm() > 6 && p.point.norm() < 15)
                //{
                // points_global.push_back(p);
                // }
            }
        }
        update_rgd(rgd_params, my_buckets, points_global2, trajectory_stretched[0].translation());

        std::cout << "number of buckets [" << x << "]: " << my_buckets.size() << std::endl;
        if (my_buckets.size() < min_buckets)
        {
            min_buckets = my_buckets.size();
            best_trajectory = trajectory_stretched;
        }
    }

    std::map<double, Eigen::Matrix4d> trajectory_for_interpolation;
    for (size_t i = 0; i < best_trajectory.size(); i++)
    {
        trajectory_for_interpolation[ts[i]] = best_trajectory[i].matrix();
    }

    std::vector<Eigen::Vector3d> pointcloud_global;
    std::vector<unsigned short> intensity;
    std::vector<double> timestamps_;
    for (const auto& p : points_reindexed)
    {
        Eigen::Matrix4d pose = getInterpolatedPose(trajectory_for_interpolation, p.timestamp);
        Eigen::Affine3d b;
        b.matrix() = pose;
        Eigen::Vector3d vec = b * p.point;
        pointcloud_global.push_back(vec);
        intensity.push_back(p.intensity);
        timestamps_.push_back(p.timestamp);
    }

    std::cout << "saving file: " << fn1 << std::endl;
    exportLaz(fn1, pointcloud_global, intensity, timestamps_);

    pointcloud_global.clear();
    for (const auto& p : points_reindexed)
    {
        Eigen::Vector3d vec = trajectory[p.index_pose] * p.point;
        pointcloud_global.push_back(vec);
    }
    exportLaz(fn2, pointcloud_global, intensity, timestamps_);
}

#if 0
void alternative_approach()
{
    int point_count_threshold = 10000;

    std::cout << "aternative_approach" << std::endl;

    std::vector<std::string> input_file_names;
    input_file_names = mandeye::fd::OpenFileDialog("Load las files", {}, true);
    std::sort(std::begin(input_file_names), std::end(input_file_names));

    std::vector<std::string> csv_files;
    std::vector<std::string> laz_files;
    std::for_each(std::begin(input_file_names), std::end(input_file_names), [&](const std::string& fileName)
        {
            if (fileName.ends_with(".laz") || fileName.ends_with(".las"))
            {
                laz_files.push_back(fileName);
            }
            if (fileName.ends_with(".csv"))
            {
                csv_files.push_back(fileName);
            } });

            std::cout << "imu files: " << std::endl;
            for (const auto& fn : csv_files)
            {
                std::cout << fn << std::endl;
            }

            std::cout << "loading imu" << std::endl;
            std::vector<std::tuple<std::pair<double, double>, FusionVector, FusionVector>> imu_data;

            std::for_each(std::begin(csv_files), std::end(csv_files), [&imu_data](const std::string& fn)
                {
                    auto imu = load_imu(fn.c_str(), 0);
                    std::cout << fn << std::endl;
                    imu_data.insert(std::end(imu_data), std::begin(imu), std::end(imu)); });

            FusionAhrs ahrs;
            FusionAhrsInitialise(&ahrs);

            if (params.fusionConventionNwu)
            {
                ahrs.settings.convention = FusionConventionNwu;
            }
            if (params.fusionConventionEnu)
            {
                ahrs.settings.convention = FusionConventionEnu;
            }
            if (params.fusionConventionNed)
            {
                ahrs.settings.convention = FusionConventionNed;
            }

            std::map<double, Eigen::Matrix4d> trajectory;

            int counter = 1;
            static bool first = true;
            static double last_ts;

            for (const auto& [timestamp_pair, gyr, acc] : imu_data)
            {
                const FusionVector gyroscope = { static_cast<float>(gyr.axis.x * RAD_TO_DEG), static_cast<float>(gyr.axis.y * RAD_TO_DEG), static_cast<float>(gyr.axis.z * RAD_TO_DEG) };
                const FusionVector accelerometer = { acc.axis.x, acc.axis.y, acc.axis.z };

                //FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
                if (first)
                {
                    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, 1.0 / 200.0);
                    first = false;
                }
                else
                {
                    double curr_ts = timestamp_pair.first;
                    double ts_diff = curr_ts - last_ts;
                    if (ts_diff < 0.01)
                    {
                        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, ts_diff);
                    }
                    else
                    {
                        std::cout << "IMU TS jump!!!" << std::endl;
                        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, 1.0 / 200.0);
                    }
                }
                last_ts = timestamp_pair.first;

                FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);

                Eigen::Quaterniond d{ quat.element.w, quat.element.x, quat.element.y, quat.element.z };
                Eigen::Affine3d t{ Eigen::Matrix4d::Identity() };
                t.rotate(d);

                trajectory[timestamp_pair.first] = t.matrix();
                const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
                counter++;
                if (counter % 100 == 0)
                {
                    printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f [%d of %d]\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw, counter++, imu_data.size());
                }
            }

            ///////////////////////////////////////////////////////////////////////////////
            std::cout << "point cloud file names" << std::endl;
            for (const auto& fn : laz_files)
            {
                std::cout << fn << std::endl;
            }

            std::vector<Point3Di> prev_points;
            std::vector<std::vector<Point3Di>> all_points;
            std::vector<std::vector<Point3Di>> tmp_points = get_batches_of_points(laz_files[0], point_count_threshold, prev_points);

            for (size_t i = 0; i < tmp_points.size() - 1; i++)
            {
                all_points.push_back(tmp_points[i]);
            }

            for (size_t i = 1; i < laz_files.size(); i++)
            {
                prev_points = tmp_points[tmp_points.size() - 1];
                tmp_points = get_batches_of_points(laz_files[i], point_count_threshold, prev_points);
                for (size_t j = 0; j < tmp_points.size() - 1; j++)
                {
                    all_points.push_back(tmp_points[j]);
                }
            }

            //////////
            std::vector<double> timestamps;
            std::vector<Eigen::Affine3d> poses;
            for (const auto& t : trajectory)
            {
                timestamps.push_back(t.first);
                Eigen::Affine3d m;
                m.matrix() = t.second;
                poses.push_back(m);
            }

            for (size_t i = 0; i < all_points.size(); i++)
            {
                std::cout << all_points[i].size() << std::endl;
                std::string fn1 = "C:/data/tmp/" + std::to_string(i) + "_best.laz";
                std::string fn2 = "C:/data/tmp/" + std::to_string(i) + "_original.laz";

                find_best_stretch(all_points[i], timestamps, poses, fn1, fn2);
            }
}
#endif

Eigen::Vector3d GLWidgetGetOGLPos(int x, int y, const ObservationPicking& observation_picking)
{
    const auto laser_beam = GetLaserBeam(x, y);

    RegistrationPlaneFeature::Plane pl;

    pl.a = 0;
    pl.b = 0;
    pl.c = 1;
    pl.d = -observation_picking.picking_plane_height;

    Eigen::Vector3d pos = rayIntersection(laser_beam, pl);

    std::cout << "intersection: " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;

    return pos;
}

void step1(const std::atomic<bool>& loPause)
{
    std::string input_folder_name;
    std::vector<std::string> input_file_names;
    input_folder_name = mandeye::fd::SelectFolder("Select Mandeye data folder");

    std::cout << "Selected folder: '" << input_folder_name << std::endl;

    if (fs::exists(input_folder_name))
    {
        std::string newTitle = winTitle + " - ..\\" + std::filesystem::path(input_folder_name).filename().string();
        glutSetWindowTitle(newTitle.c_str());

        for (const auto& entry : fs::directory_iterator(input_folder_name))
            if (entry.is_regular_file())
                input_file_names.push_back(entry.path().string());

        if (load_data(input_file_names, params, pointsPerFile, imu_data, full_debug_messages))
        {
            working_directory = fs::path(input_file_names[0]).parent_path().string();
            calculate_trajectory(
                trajectory,
                imu_data,
                params.fusionConventionNwu,
                params.fusionConventionEnu,
                params.fusionConventionNed,
                params.ahrs_gain,
                full_debug_messages,
                params.use_removie_imu_bias_from_first_stationary_scan);
            compute_step_1(pointsPerFile, params, trajectory, worker_data, loPause);
            step_1_done = true;
        }
        else
        {
            std::string message_info = "Problem with loading data from folder '" + input_folder_name +
                "' (Pease check if folder exists). Program will close once You click OK!!!";
            std::cout << message_info << std::endl;
            [[maybe_unused]] pfd::message message("Information", message_info.c_str(), pfd::choice::ok, pfd::icon::info);
            message.result();

            exit(1);
        }
    }
}

void step2(const std::atomic<bool>& loPause)
{
    double ts_failure = 0.0;
    if (compute_step_2(worker_data, params, ts_failure, loProgress, loPause, full_debug_messages))
        step_2_done = true;
    else
    {
        for (size_t fileNo = 0; fileNo < csv_files.size(); fileNo++)
        {
            const std::string& imufn = csv_files.at(fileNo);
            const std::string snFn = (fileNo >= sn_files.size()) ? ("") : (sn_files.at(fileNo));
            const auto idToSn = MLvxCalib::GetIdToSnMapping(snFn);
            // GetId of Imu to use
            int imuNumberToUse = MLvxCalib::GetImuIdToUse(idToSn, imuSnToUse);
            auto imu = load_imu(imufn.c_str(), imuNumberToUse);

            if (imu.size() > 0)
            {
                if (std::get<0>(imu[imu.size() - 1]).first > ts_failure)
                    break;
            }
            std::cout << "file: '" << imufn << "' [OK]" << std::endl;
        }
        calculations_failed = true;
    }
}

void save_results(bool info, double elapsed_seconds)
{
    outwd = get_next_result_path(working_directory);
    save_result(worker_data, params, outwd, elapsed_seconds);
    if (info)
    {
        std::string message_info = "Results saved to folder: '" + outwd.string() + "'";
        std::cout << message_info << std::endl;
        [[maybe_unused]] pfd::message message("Information", message_info.c_str(), pfd::choice::ok, pfd::icon::info);
        message.result();
    }
}

void settings_gui()
{
    if (ImGui::Begin("Settings", &is_settings_gui))
    {
        ImGui::Checkbox("simple_gui", &simple_gui);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Show even more parameters");

        ImGui::NewLine();

        if (!loRunning)
        {
            // TaitBryanPose motion_model_correction;
            ImGui::Text("motion_model_correction [deg]:");
            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::InputDouble("om, rotation via X", &params.motion_model_correction.om);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(omText);
            ImGui::InputDouble("fi, rotation via Y", &params.motion_model_correction.fi);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(fiText);
            ImGui::InputDouble("ka, rotation via Z", &params.motion_model_correction.ka);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(kaText);
            ImGui::PopItemWidth();

            if (ImGui::Button("Set example"))
            {
                params.motion_model_correction.om = 0.0;
                params.motion_model_correction.fi = 0.05;
                params.motion_model_correction.ka = 0.0;
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("motion_model_corrections for LiDAR X-axis (forward direction)");

            ImGui::NewLine();

            ImGui::Text("lidar_odometry_motion_model sigmas [m] / [deg]:");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("position/rotational uncertainties");

            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::InputDouble("x_1", &params.lidar_odometry_motion_model_x_1_sigma_m, 0.0, 0.0, "%.4f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(xText);
            ImGui::SameLine();
            ImGui::InputDouble("om_1", &params.lidar_odometry_motion_model_om_1_sigma_deg, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(omText);

            ImGui::InputDouble("y_1", &params.lidar_odometry_motion_model_y_1_sigma_m, 0.0, 0.0, "%.4f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(yText);
            ImGui::SameLine();
            ImGui::InputDouble("fi_1", &params.lidar_odometry_motion_model_fi_1_sigma_deg, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(fiText);

            ImGui::InputDouble("z_1", &params.lidar_odometry_motion_model_z_1_sigma_m, 0.0, 0.0, "%.4f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(zText);
            ImGui::SameLine();
            ImGui::InputDouble("ka_1", &params.lidar_odometry_motion_model_ka_1_sigma_deg, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(kaText);

            ImGui::PopItemWidth();

            ImGui::Text("lidar_odometry_motion_model_fix_origin sigmas [m] / [deg]:");

            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::InputDouble("x_1##o", &params.lidar_odometry_motion_model_fix_origin_x_1_sigma_m);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(xText);
            ImGui::SameLine();
            ImGui::InputDouble("om_1##o", &params.lidar_odometry_motion_model_fix_origin_om_1_sigma_deg);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(omText);

            ImGui::InputDouble("y_1##o", &params.lidar_odometry_motion_model_fix_origin_y_1_sigma_m);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(yText);
            ImGui::SameLine();
            ImGui::InputDouble("fi_1##o", &params.lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(fiText);

            ImGui::InputDouble("z_1##o", &params.lidar_odometry_motion_model_fix_origin_z_1_sigma_m);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(zText);
            ImGui::SameLine();
            ImGui::InputDouble("ka_1##o", &params.lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(kaText);

            ImGui::PopItemWidth();
        }

        ImGui::NewLine();

        if (!simple_gui)
        {
            ImGui::SetNextItemWidth(ImGuiNumberWidth);
            ImGui::InputInt("Threshold nr poses", &params.threshold_nr_poses);
            if (params.threshold_nr_poses < 1)
            {
                params.threshold_nr_poses = 1;
            }

            ImGui::NewLine();
        }

        // ImGui::Checkbox("show_all_points", &show_all_points);

        if (calculations_failed)
        {
            ImGui::Text("CALCULATIONS FAILED... please read information in console");
            ImGui::End();
            return;
        }

        ImGui::PushItemWidth(ImGuiNumberWidth);
        ImGui::InputDouble("Filter threshold XY inner [m]", &params.filter_threshold_xy_inner, 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
        {
            ImGui::BeginTooltip();
            ImGui::Text("all local points inside lidar xy_circle radius will be removed during load");
            ImGui::Text("Minimum value is given by Lidar's specs (Close Proximity Blind Zone)");
            ImGui::Text("Value can be higher to filter out close range permanent obstacles");
            ImGui::Text("e.g.: 0.1[m] for Livox Mid-360");
            ImGui::EndTooltip();
        }
        ImGui::InputDouble("Filter threshold XY outer [m]", &params.filter_threshold_xy_outer, 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
        {
            ImGui::BeginTooltip();
            ImGui::Text("all local points outside lidar xy_circle radius will be removed during load");
            ImGui::Text("Maximum value is given by Lidar's specs (Detection Range)");
            ImGui::Text("Value can be lower to adapt for different reflectivity or real world contrains");
            ImGui::Text("e.g.: 70[m] @ 80%% reflectivity for Livox Mid-360");
            ImGui::EndTooltip();
        }
        ImGui::InputDouble("Threshold output filter [m]", &params.threshould_output_filter, 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("all local points inside lidar xy_circle radius will be removed during save");
        ImGui::PopItemWidth();

        if (!simple_gui)
        {
            ImGui::NewLine();
            ImGui::Text("NDT bucket size (inner/outer)");
            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::InputDouble("##x", &params.in_out_params_indoor.resolution_X, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(xText);
            if (params.in_out_params_indoor.resolution_X < 0.01)
            {
                params.in_out_params_indoor.resolution_X = 0.01;
            }
            ImGui::SameLine();
            ImGui::InputDouble("X##ndt", &params.in_out_params_outdoor.resolution_X, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(xText);
            if (params.in_out_params_outdoor.resolution_X < 0.01)
            {
                params.in_out_params_outdoor.resolution_X = 0.01;
            }

            ImGui::InputDouble("##y", &params.in_out_params_indoor.resolution_Y, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(yText);
            if (params.in_out_params_indoor.resolution_Y < 0.01)
            {
                params.in_out_params_indoor.resolution_Y = 0.01;
            }
            ImGui::SameLine();
            ImGui::InputDouble("Y##ndt", &params.in_out_params_outdoor.resolution_Y, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(yText);
            if (params.in_out_params_outdoor.resolution_Y < 0.01)
            {
                params.in_out_params_outdoor.resolution_Y = 0.01;
            }

            ImGui::InputDouble("##z", &params.in_out_params_indoor.resolution_Z, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(zText);
            if (params.in_out_params_indoor.resolution_Z < 0.01)
            {
                params.in_out_params_indoor.resolution_Z = 0.01;
            }
            ImGui::SameLine();
            ImGui::InputDouble("Z##ndt", &params.in_out_params_outdoor.resolution_Z, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(zText);
            if (params.in_out_params_outdoor.resolution_Z < 0.01)
            {
                params.in_out_params_outdoor.resolution_Z = 0.01;
            }

            ImGui::NewLine();

            ImGui::InputDouble("Downsampling", &params.decimation, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Larger value of downsampling better performance, but worse accuracy");

            ImGui::InputDouble("Max distance of processed points [m]", &params.max_distance_lidar, 0.0, 0.0, "%.2f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Local LiDAR coordinates");

            ImGui::InputInt("Number of iterations", &params.nr_iter);
            ImGui::InputDouble(
                "Sliding window trajectory length threshold [m]", &params.sliding_window_trajectory_length_threshold, 0.0, 0.0, "%.2f");
            ImGui::InputInt("Threshold initial points", &params.threshold_initial_points);

            ImGui::NewLine();

            static int fusionConvention; // 0=NWU, 1=ENU, 2=NED
            // initialize if none selected
            if (fusionConvention < 0 || fusionConvention > 2)
                fusionConvention = 0;

            ImGui::Text("Fusion convention: ");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Coordinate system conventions for sensor fusion defining how the axes are oriented relative to world frame");

            ImGui::SameLine();
            ImGui::RadioButton("NWU", &fusionConvention, 0);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("North West Up");
            ImGui::SameLine();
            ImGui::RadioButton("ENU", &fusionConvention, 1);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("East North Up");
            ImGui::SameLine();
            ImGui::RadioButton("NED", &fusionConvention, 2);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("North East Down");

            // then update your bools if you still need them
            params.fusionConventionNwu = (fusionConvention == 0);
            params.fusionConventionEnu = (fusionConvention == 1);
            params.fusionConventionNed = (fusionConvention == 2);

            ImGui::NewLine();

            ImGui::Checkbox("Use motion from previous step", &params.use_motion_from_previous_step);
            ImGui::InputDouble("AHRS gain", &params.ahrs_gain, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                ImGui::Text("Attitude and Heading Reference System gain:");
                ImGui::Text(
                    "How strongly the accelerometer/magnetometer corrections influence the orientation estimate versus gyroscope "
                    "integration");
                ImGui::Text("Larger value means faster response to changes in orientation, but more noise");
                ImGui::EndTooltip();
            }

            ImGui::PopItemWidth();
        }
        ImGui::NewLine();
        if (!step_1_done)
        {
            if (ImGui::Button("Load data"))
            {
                loStartTime = std::chrono::system_clock::now();

                step1(loPause);
                std::cout << "Load data done please click 'Compute all' to continue calculations" << std::endl;

                std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - loStartTime;
                double elapsedSeconds = elapsed.count();

                std::cout << "Elapsed time: " << formatTime(elapsedSeconds).c_str() << std::endl;
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Select folder containing IMU *.csv and LiDAR *.laz files produced by MANDEYE (e.g.: 'continousScanning_*')");
        }
        if (step_1_done && !step_2_done)
        {
            if (ImGui::Button("Compute all"))
                step2(loPause);

            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Press this button for automatic LiDAR odometry calculation -> it will produce trajectory");
        }

        if (step_1_done && step_2_done)
        {
            if (ImGui::Button("Point cloud consistency and trajectory smoothness"))
                run_consistency(worker_data, params);
            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                ImGui::Text("This process makes trajectory smooth, point cloud will be more consistent");
                ImGui::SetTooltip("Press optionally before pressing 'Save result'");
                ImGui::EndTooltip();
            }

            ImGui::SameLine();
            ImGui::Checkbox("Use multiple Gaussians for each bucket", &params.use_mutliple_gaussian);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Multiple Gaussians suppose to work better in floor plan indoor scenarios (multiple neighbouring rooms)");

            if (ImGui::Button("Save result"))
                save_results(true, 0.0);

            ImGui::SameLine();
            ImGui::Text(
                "Press this button for saving resulting trajectory and point clouds as single session for "
                "'multi_view_tls_registration_step_2' program");
        }
        if (step_1_done && step_2_done)
        {
            if (ImGui::Button("Save all point clouds to single las/laz file"))
            {
                const auto output_file_name = mandeye::fd::SaveFileDialog("Save las/laz file", mandeye::fd::LAS_LAZ_filter, ".laz");

                if (output_file_name.size() > 0)
                {
                    session.fill_session_from_worker_data(worker_data, false, true, true, params.threshould_output_filter);
                    // save_all_to_las(session, output_file_name, false);
                    save_all_to_las(session, output_file_name, true, true);
                }
            }
        }
        if (!simple_gui)
        {
            ImGui::NewLine();
            ImGui::Checkbox("Use robust and accurate lidar odometry", &params.use_robust_and_accurate_lidar_odometry);

            if (params.use_robust_and_accurate_lidar_odometry)
            {
                ImGui::PushItemWidth(ImGuiNumberWidth);
                ImGui::InputDouble("Distance bucket [m]", &params.distance_bucket, 0.0, 0.0, "%.3f");
                ImGui::InputDouble("Polar angle [deg]", &params.polar_angle_deg, 0.0, 0.0, "%.3f");
                ImGui::InputDouble("Azimutal angle [deg]", &params.azimutal_angle_deg, 0.0, 0.0, "%.3f");
                ImGui::InputInt("Number of iterations", &params.robust_and_accurate_lidar_odometry_iterations);
                // ImGui::InputDouble("Max distance lidar", &params.max_distance_lidar);
                ImGui::PopItemWidth();
            }

            ImGui::NewLine();

            ImGui::Text("Rigid ICP using spherical coordinates");
            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::InputDouble("Distance bucket [m]##ricp", &params.distance_bucket_rigid_sf, 0.0, 0.0, "%.3f");
            ImGui::InputDouble("Polar angle [deg]##ricp", &params.polar_angle_deg_rigid_sf, 0.0, 0.0, "%.3f");
            ImGui::InputDouble("azimutal angle [deg]##ricp", &params.azimutal_angle_deg_rigid_sf, 0.0, 0.0, "%.3f");
            ImGui::InputInt("Number of iterations##ricp", &params.robust_and_accurate_lidar_odometry_rigid_sf_iterations);
            ImGui::InputDouble("Max distance [m]##ricp", &params.max_distance_lidar_rigid_sf, 0.0, 0.0, "%.2f");
            ImGui::PopItemWidth();

            ImGui::NewLine();

            ImGui::Text("Rigid spherical feature sigmas [m] / [deg]:");
            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::InputDouble("x##rgdsfs", &params.rgd_sf_sigma_x_m, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(xText);
            ImGui::SameLine();
            ImGui::InputDouble("om##rgdsfs", &params.rgd_sf_sigma_om_deg, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(omText);

            ImGui::InputDouble("y##rgdsfs", &params.rgd_sf_sigma_y_m, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(yText);
            ImGui::SameLine();
            ImGui::InputDouble("fi##rgdsfs", &params.rgd_sf_sigma_fi_deg, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(fiText);

            ImGui::InputDouble("z##rgdsfs", &params.rgd_sf_sigma_z_m, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(zText);
            ImGui::SameLine();
            ImGui::InputDouble("ka##rgdsfs", &params.rgd_sf_sigma_ka_deg, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(kaText);

            ImGui::PopItemWidth();

            ImGui::NewLine();

            if (ImGui::Button("Save trajectory to ASCII (x y z)"))
            {
                std::string output_file_name = "";
                output_file_name = mandeye::fd::SaveFileDialog("Save trajectory", {}, "");
                std::cout << "File to save: '" << output_file_name << "'" << std::endl;

                if (output_file_name.size() > 0)
                {
                    save_trajectory_to_ascii(worker_data, output_file_name);
                }
            }

            ImGui::NewLine();
            ImGui::Text("Reference point clouds for initial alignment:");

            static std::vector<std::string> input_file_names;

            if (ImGui::Button("Load laz/las files"))
            {
                input_file_names = mandeye::fd::OpenFileDialog("Load laz/las files", mandeye::fd::LAS_LAZ_filter, true);
                if (input_file_names.size() > 0)
                {
                    show_reference_points = true;
                    load_reference_point_clouds(input_file_names, params);
                }
            }

            if (!input_file_names.empty())
            {
                ImGui::Checkbox("Show points", &show_reference_points);
                ImGui::SameLine();
                ImGui::Checkbox("Show buckets", &show_reference_buckets);
                ImGui::SetNextItemWidth(ImGuiNumberWidth);
                ImGui::InputInt("Downsamplingn###ref", &dec_reference_points);

                if (ImGui::Button("Filter reference buckets"))
                {
                    filter_reference_buckets(params);
                }
            }

            if (params.initial_points.size() > 0)
            {
                ImGui::NewLine();
                ImGui::Text("Manipulate initial transformation:");
                ImGui::Checkbox("Show gizmo", &initial_transformation_gizmo);
                // gizmo_stretch_interval
                if (initial_transformation_gizmo)
                {
                    m_gizmo[0] = (float)params.m_g(0, 0);
                    m_gizmo[1] = (float)params.m_g(1, 0);
                    m_gizmo[2] = (float)params.m_g(2, 0);
                    m_gizmo[3] = (float)params.m_g(3, 0);
                    m_gizmo[4] = (float)params.m_g(0, 1);
                    m_gizmo[5] = (float)params.m_g(1, 1);
                    m_gizmo[6] = (float)params.m_g(2, 1);
                    m_gizmo[7] = (float)params.m_g(3, 1);
                    m_gizmo[8] = (float)params.m_g(0, 2);
                    m_gizmo[9] = (float)params.m_g(1, 2);
                    m_gizmo[10] = (float)params.m_g(2, 2);
                    m_gizmo[11] = (float)params.m_g(3, 2);
                    m_gizmo[12] = (float)params.m_g(0, 3);
                    m_gizmo[13] = (float)params.m_g(1, 3);
                    m_gizmo[14] = (float)params.m_g(2, 3);
                    m_gizmo[15] = (float)params.m_g(3, 3);
                }
                else
                {
                    if (ImGui::Button("Align to reference"))
                    {
                        for (int i = 0; i < 30; i++)
                        {
                            align_to_reference(params.in_out_params_indoor, params.initial_points, params.m_g, params.reference_buckets);
                        }
                    }
                }
            }

            if (worker_data.size() > 0)
            {
                ImGui::NewLine();

                ImGui::Text("Scans selection:");

                ImGui::Text("index from: ");
                ImGui::SameLine();
                ImGui::PushItemWidth(ImGuiNumberWidth);
                ImGui::SliderInt("##fs", &index_begin, 0, index_end);
                ImGui::SameLine();
                ImGui::InputInt("##fi", &index_begin, 1, 5);
                if (index_begin < 0)
                    index_begin = 0;
                if (index_begin >= index_end)
                    index_begin = std::min(index_end, static_cast<int>(worker_data.size() - 1));

                ImGui::SameLine();
                ImGui::Text(" to: ");
                ImGui::SameLine();

                int prev = index_end;
                ImGui::SliderInt("##ts", &index_end, index_begin, static_cast<int>(worker_data.size() - 1));
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("max %zu", worker_data.size() - 1);
                ImGui::SameLine();
                ImGui::InputInt("##ti", &index_end, 1, 5);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("max %zu", worker_data.size() - 1);
                if (index_end < index_begin)
                    index_end = index_begin;
                if (index_end >= worker_data.size() - 1)
                    index_end = worker_data.size() - 1;
                if (prev != index_end)
                {
                    stretch_gizmo_m = worker_data[index_end].intermediate_trajectory[0];
                }

                ImGui::PopItemWidth();

                ImGui::Text("Selection: ");
                if (ImGui::Button("Select all"))
                {
                    for (size_t k = 0; k < worker_data.size(); k++)
                    {
                        worker_data[k].show = true;
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button("Unselect"))
                {
                    for (size_t k = 0; k < worker_data.size(); k++)
                    {
                        worker_data[k].show = false;
                    }
                }

                ImGui::Text("Select scans from range:");
                ImGui::SameLine();
                if (ImGui::Button("<from, to>"))
                {
                    for (size_t k = 0; k < worker_data.size(); k++)
                        worker_data[k].show = (k >= index_begin && k <= index_end);
                }

                if (ImGui::Button("<from - 1, to - 1>"))
                {
                    if (index_begin > 1 && index_end > 1)
                    {
                        index_begin--;
                        index_end--;

                        for (size_t k = 0; k < worker_data.size(); k++)
                            worker_data[k].show = (k >= index_begin && k <= index_end);
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button("<from + 1, to + 1>"))
                {
                    if (index_begin + 1 < worker_data.size() && index_end + 1 < worker_data.size())
                    {
                        index_begin++;
                        index_end++;

                        for (size_t k = 0; k < worker_data.size(); k++)
                            worker_data[k].show = (k >= index_begin && k <= index_end);
                    }
                }

                if (ImGui::Button("from - 10, to - 10>"))
                {
                    if (index_begin > 10 && index_end > 10)
                    {
                        index_begin -= 10;
                        index_end -= 10;

                        for (size_t k = 0; k < worker_data.size(); k++)
                            worker_data[k].show = (k >= index_begin && k <= index_end);
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button("<from + 10, to + 10>"))
                {
                    if (index_begin + 10 < worker_data.size() && index_end + 10 < worker_data.size())
                    {
                        index_begin += 10;
                        index_end += 10;

                        for (size_t k = 0; k < worker_data.size(); k++)
                            worker_data[k].show = (k >= index_begin && k <= index_end);
                    }
                }

                if (ImGui::Button("Export selected scans"))
                {
                    auto output_file_name = mandeye::fd::SaveFileDialog("Save las/laz file", mandeye::fd::LAS_LAZ_filter, ".laz");
                    Eigen::Affine3d pose;
                    if (output_file_name.size() > 0)
                    {
                        session.fill_session_from_worker_data(worker_data, true, false, false, params.threshould_output_filter);
                        save_all_to_las(session, output_file_name, false, true);
                    }
                    // TODO: give value to pose even if output_file_name is wrong

                    std::cout << "----------------------------------------" << std::endl;
                    std::cout << "please add following lines to RESSO file:" << std::endl;
                    std::cout << fs::path(output_file_name).filename() << "(!!!please remove brackets!!!)" << std::endl;
                    std::cout << pose.matrix() << std::endl;

                    std::cout << "example RESSO file" << std::endl;
                    std::cout << ".................................................." << std::endl;
                    std::cout << "3" << std::endl
                              << "scan_0.laz" << std::endl
                              << "0.999775 0.000552479 -0.0212158 -0.0251188" << std::endl
                              << "0.000834612 0.997864 0.0653156 -0.0381429" << std::endl
                              << "0.0212066 - 0.0653186 0.997639 -0.000757752"
                              << "0 0 0 1" << std::endl
                              << "scan_1.laz" << std::endl
                              << "0.999783 0.00178963 -0.0207603 -0.0309683" << std::endl
                              << "-0.000467341 0.99798 0.0635239 -0.0517512" << std::endl
                              << "0.0208321 -0.0635004 0.997764 0.00331449" << std::endl
                              << "0 0 0 1" << std::endl
                              << "scan_2.laz" << std::endl
                              << "0.999783 0.00163449 -0.0207736 -0.0309985" << std::endl
                              << "-0.000312224 0.997982 0.0634957 -0.0506113" << std::endl
                              << "0.0208355 -0.0634754 0.997766 0.0028499" << std::endl
                              << "0 0 0 1" << std::endl;
                    std::cout << "................................................." << std::endl;
                }

                ImGui::SameLine();

                if (ImGui::Button("Save RESSO file"))
                {
                    auto output_file_name = mandeye::fd::SaveFileDialog("Save RESSO file", {}, "");
                    std::cout << "RESSO file to save: '" << output_file_name << "'" << std::endl;

                    if (output_file_name.size() > 0)
                    {
                        session.point_clouds_container.save_poses(fs::path(output_file_name).string(), false);
                    }
                }

                if (index_end > index_begin)
                {
                    ImGui::Separator();
                    ImGui::Checkbox("Gizmo_stretch_interval", &gizmo_stretch_interval);
                }

                // gizmo_stretch_interval
                if (gizmo_stretch_interval)
                {
                    if (index_end < worker_data.size())
                    {
                        if (worker_data[index_end].intermediate_trajectory.size() > 0)
                        {
                            m_gizmo[0] = (float)stretch_gizmo_m(0, 0);
                            m_gizmo[1] = (float)stretch_gizmo_m(1, 0);
                            m_gizmo[2] = (float)stretch_gizmo_m(2, 0);
                            m_gizmo[3] = (float)stretch_gizmo_m(3, 0);
                            m_gizmo[4] = (float)stretch_gizmo_m(0, 1);
                            m_gizmo[5] = (float)stretch_gizmo_m(1, 1);
                            m_gizmo[6] = (float)stretch_gizmo_m(2, 1);
                            m_gizmo[7] = (float)stretch_gizmo_m(3, 1);
                            m_gizmo[8] = (float)stretch_gizmo_m(0, 2);
                            m_gizmo[9] = (float)stretch_gizmo_m(1, 2);
                            m_gizmo[10] = (float)stretch_gizmo_m(2, 2);
                            m_gizmo[11] = (float)stretch_gizmo_m(3, 2);
                            m_gizmo[12] = (float)stretch_gizmo_m(0, 3);
                            m_gizmo[13] = (float)stretch_gizmo_m(1, 3);
                            m_gizmo[14] = (float)stretch_gizmo_m(2, 3);
                            m_gizmo[15] = (float)stretch_gizmo_m(3, 3);
                        }
                    }

                    if (ImGui::Button("Accept Gizmo (only translation)"))
                    {
                        if (index_end < worker_data.size())
                        {
                            if (worker_data[index_end].intermediate_trajectory.size() > 0)
                            {
                                Eigen::Affine3d current_gizmo = Eigen::Affine3d::Identity();
                                current_gizmo(0, 0) = m_gizmo[0];
                                current_gizmo(1, 0) = m_gizmo[1];
                                current_gizmo(2, 0) = m_gizmo[2];
                                current_gizmo(3, 0) = m_gizmo[3];
                                current_gizmo(0, 1) = m_gizmo[4];
                                current_gizmo(1, 1) = m_gizmo[5];
                                current_gizmo(2, 1) = m_gizmo[6];
                                current_gizmo(3, 1) = m_gizmo[7];
                                current_gizmo(0, 2) = m_gizmo[8];
                                current_gizmo(1, 2) = m_gizmo[9];
                                current_gizmo(2, 2) = m_gizmo[10];
                                current_gizmo(3, 2) = m_gizmo[11];
                                current_gizmo(0, 3) = m_gizmo[12];
                                current_gizmo(1, 3) = m_gizmo[13];
                                current_gizmo(2, 3) = m_gizmo[14];
                                current_gizmo(3, 3) = m_gizmo[15];

                                auto first_pose = worker_data[index_begin].intermediate_trajectory[0];

                                Eigen::Vector3d translation = current_gizmo.translation() - first_pose.translation();

                                float number_all_nodes_inside_interval = 0;
                                for (int i = index_begin; i < index_end; i++)
                                {
                                    number_all_nodes_inside_interval += (float)worker_data[i].intermediate_trajectory.size();
                                }

                                std::vector<std::vector<Eigen::Affine3d>> all_poses;
                                for (size_t i = 0; i < worker_data.size(); i++)
                                {
                                    std::vector<Eigen::Affine3d> poses;
                                    for (size_t j = 0; j < worker_data[i].intermediate_trajectory.size(); j++)
                                    {
                                        poses.push_back(worker_data[i].intermediate_trajectory[j]);
                                    }
                                    all_poses.push_back(poses);
                                }

                                float counter = 0;

                                Eigen::Affine3d last_m = Eigen::Affine3d::Identity();

                                for (int i = index_begin; i < index_end; i++)
                                {
                                    for (size_t j = 0; j < worker_data[i].intermediate_trajectory.size(); j++)
                                    {
                                        TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(worker_data[i].intermediate_trajectory[j]);

                                        pose.px =
                                            first_pose.translation().x() + translation.x() * (counter / number_all_nodes_inside_interval);
                                        pose.py =
                                            first_pose.translation().y() + translation.y() * (counter / number_all_nodes_inside_interval);
                                        pose.pz =
                                            first_pose.translation().z() + translation.z() * (counter / number_all_nodes_inside_interval);

                                        counter += 1.0f;

                                        worker_data[i].intermediate_trajectory[j] = affine_matrix_from_pose_tait_bryan(pose);

                                        last_m = worker_data[i].intermediate_trajectory[j];
                                    }
                                }

                                for (size_t i = index_end; i < worker_data.size(); i++)
                                {
                                    for (size_t j = 0; j < worker_data[i].intermediate_trajectory.size(); j++)
                                    {
                                        Eigen::Affine3d m_update = Eigen::Affine3d::Identity();

                                        if (j == 0)
                                        {
                                            m_update = all_poses[i - 1][all_poses[i - 1].size() - 1].inverse() * all_poses[i][0];
                                        }
                                        else
                                        {
                                            m_update = all_poses[i][j - 1].inverse() * all_poses[i][j];
                                        }
                                        last_m = last_m * m_update;
                                        worker_data[i].intermediate_trajectory[j] = last_m;
                                    }
                                }
                            }
                        }
                    }
                    ImGui::Separator();
                }

                ImGui::NewLine();
                ImGui::Text("Show/hide individual scans:");

                int n_items = (int)worker_data.size();
                int digits = (n_items > 0) ? (int)std::log10(n_items - 1) + 1 : 1;

                for (int i = 0; i < n_items; i++)
                {
                    // std::string text = "[" + std::to_string(i) + "]";
                    // ImGui::Checkbox(text.c_str(), &worker_data[i].show);
                    std::stringstream ss;
                    ss << "[" << std::setw(digits) << std::setfill('0') << i << "]";

                    ImGui::Checkbox(ss.str().c_str(), &worker_data[i].show);

                    // Arrange checkboxes in rows of 5
                    if ((i + 1) % 5 != 0)
                        ImGui::SameLine();
                }
            }
        }
    }

    ImGui::End();
}

void progress_window()
{
    ImGui::Begin("Progress", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::Text(("Working directory:\n'" + working_directory + "'").c_str());

    // Calculate elapsed time and ETA
    auto currentTime = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed = currentTime - loStartTime;
    double elapsedSeconds = elapsed.count();
    loElapsedSeconds.store(elapsedSeconds);

    float progress = loProgress.load();
    double estimatedTimeRemaining = 0.0;

    if (progress > 0.01f && progress < 100.0f)
    { // Only estimate when we have meaningful progress
        double totalEstimatedTime = elapsedSeconds / progress;
        estimatedTimeRemaining = totalEstimatedTime - elapsedSeconds;
        loEstimatedTimeRemaining.store(estimatedTimeRemaining);
    }

    // Format progress text with time information
    char progressText[256];
    char timeInfo[512];

    if (progress > 0.01f)
    {
        std::string completionTime = formatCompletionTime(estimatedTimeRemaining);
        snprintf(progressText, sizeof(progressText), "Processing: %.1f%%", progress * 100.0f);
        snprintf(
            timeInfo,
            sizeof(timeInfo),
            "Elapsed: %s | Remaining: %s | Estimated finish: ~%s",
            formatTime(elapsedSeconds).c_str(),
            formatTime(estimatedTimeRemaining).c_str(),
            completionTime.c_str());
    }
    else
    {
        snprintf(progressText, sizeof(progressText), "Processing: %.1f%%", progress * 100.0f);
        snprintf(timeInfo, sizeof(timeInfo), "Elapsed: %s | Calculating completion time...", formatTime(elapsedSeconds).c_str());
    }

    ImGui::ProgressBar(progress, ImVec2(-1.0f, 0.0f), progressText);
    ImGui::Text("%s", timeInfo);

    ImGui::NewLine();

    if (!loPause)
    {
        if (ImGui::Button("Pause"))
            loPause.store(true);

        if (ImGui::IsItemHovered())
        {
            ImGui::BeginTooltip();
            ImGui::Text("Pausing calculation will not save intermediate state!");
            ImGui::Text("You can't resume if you close the program!");
            ImGui::EndTooltip();
        }
    }
    else
    {
        if (ImGui::Button("Resume"))
            loPause.store(false);
    }
    ImGui::SameLine();
    ImGui::Text("Also check console for progress..");

    ImGui::End();
}

void openData()
{
    is_settings_gui = false;
    info_gui = false;

    loRunning.store(true);
    loProgress.store(0.0f);
    loElapsedSeconds.store(0.0);
    loEstimatedTimeRemaining.store(0.0);
    loStartTime = std::chrono::system_clock::now();

    step1(loPause);

    if (step_1_done)
    {
        std::thread loThread(
            []()
            {
                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();

                step2(loPause);

                end = std::chrono::system_clock::now();
                std::chrono::duration<double> elapsed_seconds = end - start;

                save_results(false, elapsed_seconds.count());

                loRunning = false;

                std::ostringstream oss;
                oss << "Data saved to folder:\n'" << outwd.string() << "'\n"
                    << "Calculated trajectory length: " << std::fixed << std::setprecision(1)
                    << params.total_length_of_calculated_trajectory << "[m]\n"
                    << "Elapsed time: " << formatTime(elapsed_seconds.count()).c_str();

                [[maybe_unused]] pfd::message message("Information", oss.str(), pfd::choice::ok, pfd::icon::info);
                message.result();
            });

        loThread.detach();
    }
    else // no data loaded
        loRunning = false;
}

void step1(
    const std::string& folder,
    LidarOdometryParams& params,
    std::vector<std::vector<Point3Di>>& pointsPerFile,
    Imu& imu_data,
    std::string& working_directory,
    Trajectory& trajectory,
    std::vector<WorkerData>& worker_data,
    const std::atomic<bool>& loPause)
{
    std::vector<std::string> input_file_names;

    for (const auto& entry : fs::directory_iterator(folder))
    {
        if (!entry.is_directory())
        {
            std::cout << entry.path() << std::endl;
            input_file_names.push_back(entry.path().string());
        }
    }

    if (load_data(input_file_names, params, pointsPerFile, imu_data, full_debug_messages))
    {
        working_directory = fs::path(input_file_names[0]).parent_path().string();
        calculate_trajectory(
            trajectory,
            imu_data,
            params.fusionConventionNwu,
            params.fusionConventionEnu,
            params.fusionConventionNed,
            params.ahrs_gain,
            full_debug_messages,
            params.use_removie_imu_bias_from_first_stationary_scan);
        compute_step_1(pointsPerFile, params, trajectory, worker_data, loPause);
        std::cout << "step_1_done" << std::endl;
    }
}

void step2(std::vector<WorkerData>& worker_data, LidarOdometryParams& params, const std::atomic<bool>& loPause)
{
    double ts_failure = 0.0;
    std::atomic<float> loProgress;
    compute_step_2(worker_data, params, ts_failure, loProgress, loPause, full_debug_messages);
}

void save_results(
    bool info,
    double elapsed_seconds,
    std::string& working_directory,
    std::vector<WorkerData>& worker_data,
    LidarOdometryParams& params,
    fs::path outwd)
{
    save_result(worker_data, params, outwd, elapsed_seconds);
}

void display()
{
    ImGuiIO& io = ImGui::GetIO();
    glViewport(0, 0, (GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);

    glClearColor(bg_color.x * bg_color.w, bg_color.y * bg_color.w, bg_color.z * bg_color.w, bg_color.w);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);

    updateCameraTransition();

    reshape((GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);

    viewLocal = Eigen::Affine3f::Identity();

    viewLocal.translate(rotation_center);

    viewLocal.translate(Eigen::Vector3f(translate_x, translate_y, translate_z));
    if (!lock_z)
        viewLocal.rotate(Eigen::AngleAxisf(rotate_x * DEG_TO_RAD, Eigen::Vector3f::UnitX()));
    else
        viewLocal.rotate(Eigen::AngleAxisf(-90.0 * DEG_TO_RAD, Eigen::Vector3f::UnitX()));
    viewLocal.rotate(Eigen::AngleAxisf(rotate_y * DEG_TO_RAD, Eigen::Vector3f::UnitZ()));

    viewLocal.translate(-rotation_center);

    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(viewLocal.matrix().data());

    showAxes();

    if (show_initial_points)
    {
        glColor3d(0.0, 1.0, 0.0);
        glPointSize(point_size);
        glBegin(GL_POINTS);
        for (const auto& p : params.initial_points)
        {
            auto pp = params.m_g * p.point;
            glVertex3d(pp.x(), pp.y(), pp.z());
        }
        glEnd();
    }

    if (show_covs)
    {
        for (const auto& b : params.buckets_indoor)
            draw_ellipse(b.second.cov, b.second.mean, Eigen::Vector3f(0.0f, 0.0f, 1.0f), 3);
    }

#if 0 // ToDo
    for (size_t i = 0; i < worker_data.size(); i++)
    {
        if (worker_data[i].show)
        {
            glPointSize(1);
            glColor3d(0.0, 0.0, 1.0);
            glBegin(GL_POINTS);
            for (const auto &p : worker_data[i].intermediate_points)
            {
                // std::cout << "kk";
                // std::cout << p.index_pose;
                Eigen::Vector3d pt = worker_data[i].intermediate_trajectory[p.index_pose] * p.point;
                glVertex3d(pt.x(), pt.y(), pt.z());
            }
            glEnd();
            glPointSize(1);

            glLineWidth(1);
            glBegin(GL_LINES);
            const auto &it = worker_data[i].intermediate_trajectory[0];
            glColor3f(1, 0, 0);
            glVertex3f(it(0, 3), it(1, 3), it(2, 3));
            glVertex3f(it(0, 3) + it(0, 0), it(1, 3) + it(1, 0), it(2, 3) + it(2, 0));

            glColor3f(0, 1, 0);
            glVertex3f(it(0, 3), it(1, 3), it(2, 3));
            glVertex3f(it(0, 3) + it(0, 1), it(1, 3) + it(1, 1), it(2, 3) + it(2, 1));

            glEnd();
        }
    }
#endif

    if (show_reference_points)
    {
        glColor3f(1, 0, 0);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < params.reference_points.size(); i += dec_reference_points)
            glVertex3f(params.reference_points[i].point.x(), params.reference_points[i].point.y(), params.reference_points[i].point.z());
        glEnd();
    }

    if (show_trajectory_as_axes)
    {
        // glBegin(GL_LINE_STRIP);
        glBegin(GL_LINES);
        for (const auto& wd : worker_data)
        {
            for (const auto& it : wd.intermediate_trajectory)
            {
                glColor3f(1, 0, 0);
                glVertex3f(it(0, 3), it(1, 3), it(2, 3));
                glVertex3f(it(0, 3) + it(0, 0) * 0.1, it(1, 3) + it(1, 0) * 0.1, it(2, 3) + it(2, 0) * 0.1);

                glColor3f(0, 1, 0);
                glVertex3f(it(0, 3), it(1, 3), it(2, 3));
                glVertex3f(it(0, 3) + it(0, 1) * 0.1, it(1, 3) + it(1, 1) * 0.1, it(2, 3) + it(2, 1) * 0.1);

                glColor3f(0, 0, 1);
                glVertex3f(it(0, 3), it(1, 3), it(2, 3));
                glVertex3f(it(0, 3) + it(0, 2) * 0.1, it(1, 3) + it(1, 2) * 0.1, it(2, 3) + it(2, 2) * 0.1);
            }
        }
        glEnd();
    }

    if (show_trajectory)
    {
        glPointSize(3);
        glColor3f(0, 1, 1);
        glBegin(GL_POINTS);
        for (const auto& wd : worker_data)
        {
            for (const auto& it : wd.intermediate_trajectory)
                glVertex3f(it(0, 3), it(1, 3), it(2, 3));
        }
        glEnd();
        glPointSize(1);

        glColor3f(1, 0, 0);
        glBegin(GL_LINES);
        if (worker_data.size() > 0)
        {
            const auto& wd = worker_data[worker_data.size() - 1];
            if (wd.intermediate_trajectory.size() > 0)
            {
                const auto& it = wd.intermediate_trajectory[wd.intermediate_trajectory.size() - 1];

                glVertex3f(it(0, 3) - 1, it(1, 3), it(2, 3));
                glVertex3f(it(0, 3) + 1, it(1, 3), it(2, 3));

                glVertex3f(it(0, 3), it(1, 3) - 1, it(2, 3));
                glVertex3f(it(0, 3), it(1, 3) + 1, it(2, 3));

                glVertex3f(it(0, 3), it(1, 3), it(2, 3) - 1);
                glVertex3f(it(0, 3), it(1, 3), it(2, 3) + 1);
            }
        }
        glEnd();
    }

    if (show_reference_buckets)
    {
        glColor3f(1, 0, 0);
        glBegin(GL_POINTS);
        for (const auto& b : params.reference_buckets)
            glVertex3f(b.second.mean.x(), b.second.mean.y(), b.second.mean.z());
        glEnd();
    }

    //
    if (worker_data.size() > 0)
    {
        if (index_begin < worker_data.size())
        {
            glColor3f(1, 1, 0);
            glBegin(GL_LINES);
            if (worker_data[index_begin].intermediate_trajectory.size() > 0)
            {
                auto p = worker_data[index_begin].intermediate_trajectory[0].translation();
                glVertex3f(p.x() - 1, p.y(), p.z());
                glVertex3f(p.x() + 1, p.y(), p.z());

                glVertex3f(p.x(), p.y() - 1, p.z());
                glVertex3f(p.x(), p.y() + 1, p.z());

                glVertex3f(p.x(), p.y(), p.z() - 1);
                glVertex3f(p.x(), p.y(), p.z() + 1);
            }
            glEnd();
        }

        if (index_end < worker_data.size())
        {
            glColor3f(0, 1, 1);
            glBegin(GL_LINES);
            if (worker_data[index_end].intermediate_trajectory.size() > 0)
            {
                auto p = worker_data[index_end].intermediate_trajectory[0].translation();
                glVertex3f(p.x() - 1, p.y(), p.z());
                glVertex3f(p.x() + 1, p.y(), p.z());

                glVertex3f(p.x(), p.y() - 1, p.z());
                glVertex3f(p.x(), p.y() + 1, p.z());

                glVertex3f(p.x(), p.y(), p.z() - 1);
                glVertex3f(p.x(), p.y(), p.z() + 1);
            }
            glEnd();
        }
    }

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGLUT_NewFrame();
    ImGui::NewFrame();

    ShowMainDockSpace();

    view_kbd_shortcuts();

    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_O, false))
    {
        openData();

        // workaround
        io.AddKeyEvent(ImGuiKey_O, false);
        io.AddKeyEvent(ImGuiMod_Ctrl, false);
    }

    if (!loRunning)
    {
        if (ImGui::BeginMainMenuBar())
        {
            if (ImGui::BeginMenu("Presets"))
            {
                if (ImGui::MenuItem("1 Velocity < 8km/h, tiny spaces", nullptr, (lastPar == 1)))
                {
                    lastPar = 1;

                    set_lidar_odometry_default_params(params);

                    std::cout << "clicked: Set parameters for velocity up to 8km/h, tiny spaces" << std::endl;
                }
                if (ImGui::MenuItem("2 Velocity < 8km/h, quick but less accurate/precise", nullptr, (lastPar == 2)))
                {
                    lastPar = 2;

                    params.decimation = 0.03;
                    params.in_out_params_indoor.resolution_X = 0.1;
                    params.in_out_params_indoor.resolution_Y = 0.1;
                    params.in_out_params_indoor.resolution_Z = 0.1;

                    params.in_out_params_outdoor.resolution_X = 0.3;
                    params.in_out_params_outdoor.resolution_Y = 0.3;
                    params.in_out_params_outdoor.resolution_Z = 0.3;

                    params.filter_threshold_xy_inner = 0.3;
                    params.filter_threshold_xy_outer = 70.0;
                    params.threshould_output_filter = 0.3;

                    params.use_robust_and_accurate_lidar_odometry = false;
                    params.distance_bucket = 0.2;
                    params.polar_angle_deg = 10.0;
                    params.azimutal_angle_deg = 10.0;
                    params.robust_and_accurate_lidar_odometry_iterations = 20;

                    params.max_distance_lidar = 30.0;
                    params.nr_iter = 30;
                    params.sliding_window_trajectory_length_threshold = 200;
                    params.real_time_threshold_seconds = 0.3;

                    std::cout << "clicked: Set parameters for velocity < 8km/h, quick but less accurate/precise" << std::endl;
                }
                if (ImGui::MenuItem("3 Velocity < 30 km/h, fast motion, open spaces", nullptr, (lastPar == 3)))
                {
                    lastPar = 3;

                    params.decimation = 0.03;
                    params.in_out_params_indoor.resolution_X = 0.3;
                    params.in_out_params_indoor.resolution_Y = 0.3;
                    params.in_out_params_indoor.resolution_Z = 0.3;

                    params.in_out_params_outdoor.resolution_X = 0.5;
                    params.in_out_params_outdoor.resolution_Y = 0.5;
                    params.in_out_params_outdoor.resolution_Z = 0.5;

                    params.filter_threshold_xy_inner = 3.0;
                    params.filter_threshold_xy_outer = 70.0;
                    params.threshould_output_filter = 3.0;

                    params.use_robust_and_accurate_lidar_odometry = false;
                    params.distance_bucket = 0.2;
                    params.polar_angle_deg = 10.0;
                    params.azimutal_angle_deg = 10.0;
                    params.robust_and_accurate_lidar_odometry_iterations = 20;

                    params.max_distance_lidar = 70.0;
                    params.nr_iter = 500;
                    params.sliding_window_trajectory_length_threshold = 200;
                    params.real_time_threshold_seconds = 10;

                    std::cout << "clicked: Set parameters for velocity < 30 km/h, fast motion, open spaces" << std::endl;
                }
                if (ImGui::MenuItem("4 Velocity < 8km/h, precise forestry", nullptr, (lastPar == 4)))
                {
                    lastPar = 4;

                    params.decimation = 0.01;
                    params.in_out_params_indoor.resolution_X = 0.1;
                    params.in_out_params_indoor.resolution_Y = 0.1;
                    params.in_out_params_indoor.resolution_Z = 0.1;

                    params.in_out_params_outdoor.resolution_X = 0.3;
                    params.in_out_params_outdoor.resolution_Y = 0.3;
                    params.in_out_params_outdoor.resolution_Z = 0.3;

                    params.filter_threshold_xy_inner = 1.5;
                    params.filter_threshold_xy_outer = 70.0;
                    params.threshould_output_filter = 1.5;

                    params.use_robust_and_accurate_lidar_odometry = false;
                    params.distance_bucket = 0.2;
                    params.polar_angle_deg = 10.0;
                    params.azimutal_angle_deg = 10.0;
                    params.robust_and_accurate_lidar_odometry_iterations = 20;

                    params.max_distance_lidar = 70.0;
                    params.nr_iter = 500;
                    params.sliding_window_trajectory_length_threshold = 10000;
                    params.real_time_threshold_seconds = 10;

                    std::cout << "clicked: Set parameters for velocity < 8km/h, precise forestry" << std::endl;
                }

                ImGui::EndMenu();
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Set parameters for MANDEYE data processing");

            if (ImGui::BeginMenu("Parameters"))
            {
                ImGui::MenuItem(
                    "Remove IMU bias from first stationary scan", nullptr, &params.use_removie_imu_bias_from_first_stationary_scan);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("IMU bias will be removed given there's a stationary period at the start of the recording");

                ImGui::MenuItem("Multithread", nullptr, &params.useMultithread);
                ImGui::SetNextItemWidth(ImGuiNumberWidth / 2);
                ImGui::InputDouble("Time threshold [s]", &params.real_time_threshold_seconds, 0.0, 0.0, "%.1f");
                if (ImGui::IsItemHovered())
                {
                    ImGui::BeginTooltip();
                    ImGui::Text("Optimization timeout");
                    ImGui::Text("smaller value gives faster calculations, less precise results");
                    ImGui::Text("e.g.: 0.1 [s] will make processing time aprox equal to scan time");
                    ImGui::Text("0.3 [s] will make processing time aprox equal to 3x scan time");
                    ImGui::EndTooltip();
                }

                if (ImGui::MenuItem("Set real time performance"))
                    params.real_time_threshold_seconds = 0.1;
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("For IMU 200Hz, 20 nodes in optimization window");

                ImGui::Separator();

                ImGui::Text("Debug:");

                ImGui::MenuItem("Full processing messages", nullptr, &full_debug_messages);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Show more messages during processing in console window");
                ImGui::MenuItem("Save calibration validation file", nullptr, &params.save_calibration_validation);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Save calibrated points from first laz file in 'calibrationValidation.asc' file");
                ImGui::SetNextItemWidth(ImGuiNumberWidth);
                ImGui::InputInt("Number of calibration validation points", &params.calibration_validation_points);

                ImGui::Separator();
                if (ImGui::BeginMenu("Ablation study"))
                {
                    ImGui::MenuItem("Use planarity", nullptr, &params.ablation_study_use_planarity);
                    ImGui::MenuItem("Use norm", nullptr, &params.ablation_study_use_norm);
                    ImGui::MenuItem("Use hierarchical RGD", nullptr, &params.ablation_study_use_hierarchical_rgd);
                    ImGui::MenuItem("Use view point and normal vectors", nullptr, &params.ablation_study_use_view_point_and_normal_vectors);

                    ImGui::EndMenu();
                }

                ImGui::Separator();

                ImGui::MenuItem("Saving results with index pose", nullptr, &params.save_index_pose);

                ImGui::Separator();

                if (ImGui::MenuItem("Load parameters"))
                {
                    auto input_file_names = mandeye::fd::OpenFileDialog("Load parameters file", mandeye::fd::Toml_filter, ".toml");

                    if (input_file_names.size() > 0)
                    {
                        try
                        {
                            // Use the original TomlIO class for loading parameters in GUI
                            TomlIO toml_io;
                            toml_io.LoadParametersFromTomlFile(input_file_names[0], params);
                            std::cout << "Parameters loaded from: " << input_file_names[0] << std::endl;
                        } catch (const std::exception& e)
                        {
                            std::cerr << "Error loading TOML file: " << e.what() << std::endl;
                        }
                    }
                }

                if (ImGui::MenuItem("Save parameters"))
                {
                    auto output_file_name = mandeye::fd::SaveFileDialog("Save parameters file", mandeye::fd::Toml_filter, ".toml");
                    std::cout << "Parameters file to save: '" << output_file_name << "'" << std::endl;

                    if (!output_file_name.empty())
                    {
                        // Use the original TomlIO class for saving parameters in GUI
                        TomlIO toml_io;
                        bool success = toml_io.SaveParametersToTomlFile(output_file_name, params);
                        if (success)
                            std::cout << "Parameters file generated: " << output_file_name << std::endl;
                        else
                            std::cerr << "Failed to save parameters." << std::endl;
                    }
                }

                ImGui::EndMenu();
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Processing parameter list");

            ImGui::SameLine();
            ImGui::Dummy(ImVec2(20, 0));
            ImGui::SameLine();

            if (ImGui::Button("Load & process scanning"))
                openData();
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Select folder from where to load data and start processing (Ctrl+O)");

            ImGui::SameLine();
            ImGui::Dummy(ImVec2(20, 0));
            ImGui::SameLine();

            if (ImGui::BeginMenu("View"))
            {
                ImGui::BeginDisabled(!(session.point_clouds_container.point_clouds.size() > 0));
                {
                    auto tmp = point_size;
                    ImGui::SetNextItemWidth(ImGuiNumberWidth);
                    ImGui::InputInt("Points size", &point_size);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("keyboard 1-9 keys");
                    if (point_size < 1)
                        point_size = 1;
                    else if (point_size > 10)
                        point_size = 10;

                    if (tmp != point_size)
                        for (auto& point_cloud : session.point_clouds_container.point_clouds)
                            point_cloud.point_size = point_size;

                    ImGui::Separator();
                }
                ImGui::EndDisabled();

                ImGui::MenuItem("Show initial points", nullptr, &show_initial_points);
                ImGui::MenuItem("Show trajectory", nullptr, &show_trajectory);
                ImGui::MenuItem("Show trajectory as axes", nullptr, &show_trajectory_as_axes);
                ImGui::MenuItem("Show compass/ruler", "key C", &compass_ruler);

                ImGui::MenuItem("Lock Z", "Shift + Z", &lock_z, !is_ortho);

                // ImGui::MenuItem("show_covs", nullptr, &show_covs);

                ImGui::Separator();

                ImGui::Text("Colors:");

                ImGui::ColorEdit3("Background", (float*)&params.clear_color, ImGuiColorEditFlags_NoInputs);

                bg_color = params.clear_color;

                ImGui::Separator();

                ImGui::MenuItem("Settings", nullptr, &is_settings_gui);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Show power user settings window with more parameters");

                ImGui::EndMenu();
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Scene view relevant parameters");

            camMenu();

            ImGui::SameLine(
                ImGui::GetWindowWidth() - ImGui::CalcTextSize("Info").x - ImGui::GetStyle().ItemSpacing.x * 2 -
                ImGui::GetStyle().FramePadding.x * 2);

            ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4, 2));
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetStyleColorVec4(ImGuiCol_HeaderHovered));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImGui::GetStyleColorVec4(ImGuiCol_Header));
            if (ImGui::SmallButton("Info"))
                info_gui = !info_gui;

            ImGui::PopStyleVar(2);
            ImGui::PopStyleColor(3);

            ImGui::EndMainMenuBar();
        }
    }

    if (initial_transformation_gizmo)
    {
        ImGuiIO& io = ImGui::GetIO();
        // ImGuizmo -----------------------------------------------
        ImGuizmo::BeginFrame();
        ImGuizmo::Enable(true);
        ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

        GLfloat projection[16];
        glGetFloatv(GL_PROJECTION_MATRIX, projection);

        GLfloat modelview[16];
        glGetFloatv(GL_MODELVIEW_MATRIX, modelview);

        ImGuizmo::Manipulate(
            &modelview[0],
            &projection[0],
            ImGuizmo::TRANSLATE | ImGuizmo::ROTATE_Z | ImGuizmo::ROTATE_X | ImGuizmo::ROTATE_Y,
            ImGuizmo::WORLD,
            m_gizmo,
            NULL);

        params.m_g(0, 0) = m_gizmo[0];
        params.m_g(1, 0) = m_gizmo[1];
        params.m_g(2, 0) = m_gizmo[2];
        params.m_g(3, 0) = m_gizmo[3];
        params.m_g(0, 1) = m_gizmo[4];
        params.m_g(1, 1) = m_gizmo[5];
        params.m_g(2, 1) = m_gizmo[6];
        params.m_g(3, 1) = m_gizmo[7];
        params.m_g(0, 2) = m_gizmo[8];
        params.m_g(1, 2) = m_gizmo[9];
        params.m_g(2, 2) = m_gizmo[10];
        params.m_g(3, 2) = m_gizmo[11];
        params.m_g(0, 3) = m_gizmo[12];
        params.m_g(1, 3) = m_gizmo[13];
        params.m_g(2, 3) = m_gizmo[14];
        params.m_g(3, 3) = m_gizmo[15];
    }

    if (gizmo_stretch_interval)
    {
        ImGuiIO& io = ImGui::GetIO();
        // ImGuizmo -----------------------------------------------
        ImGuizmo::BeginFrame();
        ImGuizmo::Enable(true);
        ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

        GLfloat projection[16];
        glGetFloatv(GL_PROJECTION_MATRIX, projection);

        GLfloat modelview[16];
        glGetFloatv(GL_MODELVIEW_MATRIX, modelview);

        ImGuizmo::Manipulate(
            &modelview[0],
            &projection[0],
            ImGuizmo::TRANSLATE | ImGuizmo::ROTATE_Z | ImGuizmo::ROTATE_X | ImGuizmo::ROTATE_Y,
            ImGuizmo::WORLD,
            m_gizmo,
            NULL);

        stretch_gizmo_m(0, 0) = m_gizmo[0];
        stretch_gizmo_m(1, 0) = m_gizmo[1];
        stretch_gizmo_m(2, 0) = m_gizmo[2];
        stretch_gizmo_m(3, 0) = m_gizmo[3];
        stretch_gizmo_m(0, 1) = m_gizmo[4];
        stretch_gizmo_m(1, 1) = m_gizmo[5];
        stretch_gizmo_m(2, 1) = m_gizmo[6];
        stretch_gizmo_m(3, 1) = m_gizmo[7];
        stretch_gizmo_m(0, 2) = m_gizmo[8];
        stretch_gizmo_m(1, 2) = m_gizmo[9];
        stretch_gizmo_m(2, 2) = m_gizmo[10];
        stretch_gizmo_m(3, 2) = m_gizmo[11];
        stretch_gizmo_m(0, 3) = m_gizmo[12];
        stretch_gizmo_m(1, 3) = m_gizmo[13];
        stretch_gizmo_m(2, 3) = m_gizmo[14];
        stretch_gizmo_m(3, 3) = m_gizmo[15];
    }

    if (is_settings_gui)
    {
        lastPar = 0;

        settings_gui();
    }

    if (loRunning)
        progress_window();

    cor_window();

    if (info_gui)
    {
        infoLines[infoLines.size() - 2] = "It saves session file in " + working_directory + "\\lio_result_*";
        info_window(infoLines, appShortcuts);
    }

    if (compass_ruler)
        drawMiniCompassWithRuler();

    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

    glutSwapBuffers();
    glutPostRedisplay();
}

void on_exit()
{
    // remove cache
    std::cout << "remove cache: '" << params.working_directory_cache << "' START" << std::endl;
    std::filesystem::remove_all(params.working_directory_cache);
    std::cout << "remove cache: '" << params.working_directory_cache << "' FINISHED" << std::endl;
}

void mouse(int glut_button, int state, int x, int y)
{
    ImGuiIO& io = ImGui::GetIO();
    io.MousePos = ImVec2((float)x, (float)y);

    int button = -1;
    if (glut_button == GLUT_LEFT_BUTTON)
        button = 0;
    if (glut_button == GLUT_RIGHT_BUTTON)
        button = 1;
    if (glut_button == GLUT_MIDDLE_BUTTON)
        button = 2;
    if (button != -1 && state == GLUT_DOWN)
        io.MouseDown[button] = true;
    if (button != -1 && state == GLUT_UP)
        io.MouseDown[button] = false;

    static int glutMajorVersion = glutGet(GLUT_VERSION) / 10000;
    if (state == GLUT_DOWN && (glut_button == 3 || glut_button == 4) && glutMajorVersion < 3)
        wheel(glut_button, glut_button == 3 ? 1 : -1, x, y);

    if (!io.WantCaptureMouse)
    {
        if ((glut_button == GLUT_MIDDLE_BUTTON || glut_button == GLUT_RIGHT_BUTTON) && state == GLUT_DOWN && io.KeyCtrl)
            setNewRotationCenter(x, y);

        if (state == GLUT_DOWN)
            mouse_buttons |= 1 << glut_button;
        else if (state == GLUT_UP)
            mouse_buttons = 0;

        mouse_old_x = x;
        mouse_old_y = y;
    }
}

int main(int argc, char* argv[])
{
    set_lidar_odometry_default_params(params);

    try
    {
        if (argc == 4) // runnning from command line
        {
            // Load parameters from file using original TomlIO class
            TomlIO toml_io;
            toml_io.LoadParametersFromTomlFile(argv[2], params);
            std::cout << "Parameters loaded OK from: " << argv[2] << std::endl;

            std::string working_directory;
            std::vector<WorkerData> worker_data;

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();

            std::atomic<bool> loPause{ false };
            step1(argv[1], params, pointsPerFile, imu_data, working_directory, trajectory, worker_data, loPause);

            step2(worker_data, params, loPause);

            end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;
            std::time_t end_time = std::chrono::system_clock::to_time_t(end);
            std::cout << "calculations finished computation at " << std::ctime(&end_time)
                      << "Elapsed time: " << formatTime(elapsed_seconds.count()).c_str() << "s\n";

            save_results(false, elapsed_seconds.count(), working_directory, worker_data, params, argv[3]);
        }
        else // full GUI mode
        {
            std::cout << argv[0] << " input_folder parameters(*.toml) output_folder" << std::endl;

            initGL(&argc, argv, winTitle, display, mouse);
            glutCloseFunc(on_exit);

            glutMainLoop();

            ImGui_ImplOpenGL2_Shutdown();
            ImGui_ImplGLUT_Shutdown();
            ImGui::DestroyContext();
        }
    } catch (const std::bad_alloc& e)
    {
        std::cerr << "System is out of memory : " << e.what() << std::endl;
        mandeye::fd::OutOfMemMessage();
    } catch (const std::exception& e)
    {
        std::cout << e.what();
    } catch (...)
    {
        std::cerr << "Unknown fatal error occurred." << std::endl;
    }

    return 0;
}
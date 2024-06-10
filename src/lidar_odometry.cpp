#include "lidar_odometry_utils.h"
#include <registration_plane_feature.h>

// This is LiDAR odometry (step 1)
// This program calculates trajectory based on IMU and LiDAR data provided by MANDEYE mobile mapping system https://github.com/JanuszBedkowski/mandeye_controller
// The output is a session proving trajekctory and point clouds that can be  further processed by "multi_view_tls_registration" program.

#define SAMPLE_PERIOD (1.0 / 200.0)
namespace fs = std::filesystem;

// std::vector<Point3Di> initial_points;
NDT ndt;
// NDT::GridParameters in_out_params;

// NDTBucketMapType buckets;
// NDTBucketMapType reference_buckets;
bool show_reference_buckets = true;

// std::vector<Point3Di> reference_points;
bool show_reference_points = false;
int dec_reference_points = 100;
bool show_initial_points = true;
bool show_trajectory = true;
bool show_trajectory_as_axes = false;
bool show_covs = false;
int dec_covs = 10;
// double filter_threshold_xy = 0.5;
// int nr_iter = 100;
// double sliding_window_trajectory_length_threshold = 50.0;
bool fusionConventionNwu = true;
bool fusionConventionEnu = false;
bool fusionConventionNed = false;
// bool use_motion_from_previous_step = true;
// bool useMultithread = true;
bool simple_gui = true;
bool step_1_done = false;
bool step_2_done = false;
bool step_3_done = false;

std::vector<WorkerData> worker_data;

float rotate_x = 0.0, rotate_y = 0.0;
Eigen::Vector3f rotation_center = Eigen::Vector3f::Zero();
float translate_x, translate_y = 0.0;
float translate_z = -50.0;
const unsigned int window_width = 800;
const unsigned int window_height = 600;
static ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
int mouse_old_x, mouse_old_y;
bool gui_mouse_down{false};
int mouse_buttons = 0;
float mouse_sensitivity = 1.0;
std::string working_directory = "";
// std::string working_directory_preview = "";
// double decimation = 0.1;
int threshold_initial_points = 100000;
bool initial_transformation_gizmo = false;

float m_gizmo[] = {1, 0, 0, 0,
                   0, 1, 0, 0,
                   0, 0, 1, 0,
                   0, 0, 0, 1};
// Eigen::Affine3d m_g = Eigen::Affine3d::Identity();
// double consecutive_distance = 0.0;
float x_displacement = 0.01;
int num_constistency_iter = 10;

int index_from_inclusive = -1;
int index_to_inclusive = -1;
bool gizmo_stretch_interval = false;
Eigen::Affine3d stretch_gizmo_m = Eigen::Affine3d::Identity();

LidarOdometryParams params;
const std::vector<std::string> LAS_LAZ_filter = {"LAS file (*.laz)", "*.laz", "LASzip file (*.las)", "*.las", "All files", "*"};

void alternative_approach();
LaserBeam GetLaserBeam(int x, int y);
Eigen::Vector3d rayIntersection(const LaserBeam &laser_beam, const RegistrationPlaneFeature::Plane &plane);
bool exportLaz(const std::string &filename, const std::vector<Eigen::Vector3d> &pointcloud, const std::vector<unsigned short> &intensity, double offset_x,
               double offset_y,
               double offset_alt);

void lidar_odometry_gui()
{
    if (ImGui::Begin("lidar_odometry_step_1 v0.43"))
    {
        ImGui::Text("This program is first step in MANDEYE process.");
        ImGui::Text("It results trajectory and point clouds as single session for 'multi_view_tls_registration_step_2' program.");
        ImGui::Text("It saves session.json file in 'Working directory'.");
        ImGui::Text("Next step will be to load session.json file with 'multi_view_tls_registration_step_2' program.");
        ImGui::Checkbox("simple_gui", &simple_gui);
        if (!simple_gui)
        {
            ImGui::SliderFloat("mouse_sensitivity_xy", &mouse_sensitivity, 0.1, 10);
        }
        ImGui::Text(("Working directory ('session.json' will be saved here): '" + working_directory + "'").c_str());
        // ImGui::Checkbox("show_all_points", &show_all_points);
        ImGui::InputFloat3("rotation center", rotation_center.data());
        if (!simple_gui)
        {
            ImGui::Checkbox("show_initial_points", &show_initial_points);
            ImGui::Checkbox("show_trajectory", &show_trajectory);
            ImGui::SameLine();
            ImGui::Checkbox("show_trajectory_as_axes", &show_trajectory_as_axes);
            // ImGui::Checkbox("show_covs", &show_covs);
            ImGui::InputDouble("normal distributions transform bucket size X", &params.in_out_params.resolution_X);
            if (params.in_out_params.resolution_X < 0.2)
            {
                params.in_out_params.resolution_X = 0.2;
            }
            ImGui::InputDouble("normal distributions transform bucket size Y", &params.in_out_params.resolution_Y);
            if (params.in_out_params.resolution_Y < 0.2)
            {
                params.in_out_params.resolution_Y = 0.2;
            }
            ImGui::InputDouble("normal distributions transform bucket size Z", &params.in_out_params.resolution_Z);
            if (params.in_out_params.resolution_Z < 0.2)
            {
                params.in_out_params.resolution_Z = 0.2;
            }

            ImGui::InputDouble("filter_threshold_xy (all local points inside lidar xy_circle radius[m] will be removed)", &params.filter_threshold_xy);

            ImGui::InputDouble("decimation (larger value of decimation better performance, but worse accuracy)", &params.decimation);
            ImGui::InputInt("number iterations", &params.nr_iter);
            ImGui::InputDouble("sliding window trajectory length threshold", &params.sliding_window_trajectory_length_threshold);
            ImGui::InputInt("threshold initial points", &threshold_initial_points);
            ImGui::Checkbox("use_multithread", &params.useMultithread);
            ImGui::Checkbox("fusionConventionNwu", &fusionConventionNwu);
            if (fusionConventionNwu)
            {
                // fusionConventionNwu
                fusionConventionEnu = false;
                fusionConventionNed = false;
            }
            ImGui::Checkbox("fusionConventionEnu", &fusionConventionEnu);
            if (fusionConventionEnu)
            {
                fusionConventionNwu = false;
                // fusionConventionEnu
                fusionConventionNed = false;
            }
            ImGui::Checkbox("fusionConventionNed", &fusionConventionNed);
            if (fusionConventionNed)
            {
                fusionConventionNwu = false;
                fusionConventionEnu = false;
                // fusionConventionNed
            }

            if (!fusionConventionNwu && !fusionConventionEnu && !fusionConventionNed)
            {
                fusionConventionNwu = true;
            }

            ImGui::Checkbox("use_motion_from_previous_step", &params.use_motion_from_previous_step);
        }
        if (!step_1_done)
        {
            // if (ImGui::Button("alternative_approach"))
            //{
            //     alternative_approach();
            // }

            if (ImGui::Button("load data (step 1)"))
            {

                static std::shared_ptr<pfd::open_file> open_file;
                std::vector<std::string> input_file_names;
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
                const auto t = [&]()
                {
                    std::vector<std::string> filters;
                    auto sel = pfd::open_file("Load las files", "C:\\", filters, true).result();
                    for (int i = 0; i < sel.size(); i++)
                    {
                        input_file_names.push_back(sel[i]);
                        // std::cout << "las file: '" << input_file_name << "'" << std::endl;
                    }
                };
                std::thread t1(t);
                t1.join();

                std::sort(std::begin(input_file_names), std::end(input_file_names));

                std::vector<std::string> csv_files;
                std::vector<std::string> laz_files;
                std::vector<std::string> sn_files;

                std::for_each(std::begin(input_file_names), std::end(input_file_names), [&](const std::string &fileName)
                              {
                    if (fileName.ends_with(".laz") || fileName.ends_with(".las"))
                    {
                        laz_files.push_back(fileName);
                    }
                    if (fileName.ends_with(".csv"))
                    {
                        csv_files.push_back(fileName);
                    }
                    if (fileName.ends_with(".sn"))
                    {
                        sn_files.push_back(fileName);
                    } });

                if (input_file_names.size() > 0 && laz_files.size() == csv_files.size())
                {
                    working_directory = fs::path(input_file_names[0]).parent_path().string();

                    const auto calibrationFile = (fs::path(working_directory) / "calibration.json").string();
                    const auto preloadedCalibration = MLvxCalib::GetCalibrationFromFile(calibrationFile);
                    const std::string imuSnToUse = MLvxCalib::GetImuSnToUse(calibrationFile);
                    if (!preloadedCalibration.empty())
                    {
                        std::cout << "Loaded calibration for: \n";
                        for (const auto &[sn, _] : preloadedCalibration)
                        {
                            std::cout << " -> " << sn << std::endl;
                        }
                    }
                    else
                    {
                        std::cout << "There is no calibration.json file in folder (check comment in source code) file: " << __FILE__ << " line: " << __LINE__ << std::endl;
                        // example file for 2x livox";
                        /*
                        {
                            "calibration" : {
                                "47MDL9T0020193" : {
                                    "identity" : "true"
                                },
                                "47MDL9S0020300" :
                                    {
                                        "order" : "ROW",
                                        "inverted" : "TRUE",
                                        "data" : [
                                            0.999824, 0.00466397, -0.0181595, -0.00425984,
                                            -0.0181478, -0.00254457, -0.999832, -0.151599,
                                            -0.0047094, 0.999986, -0.00245948, -0.146408,
                                            0, 0, 0, 1
                                        ]
                                    }
                            },
                                            "imuToUse" : "47MDL9T0020193"
                        }*/
                    }

                    fs::path wdp = fs::path(input_file_names[0]).parent_path();
                    wdp /= "preview";
                    if (!fs::exists(wdp))
                    {
                        fs::create_directory(wdp);
                    }

                    params.working_directory_preview = wdp.string();

                    for (size_t i = 0; i < input_file_names.size(); i++)
                    {
                        std::cout << input_file_names[i] << std::endl;
                    }
                    std::cout << "loading imu" << std::endl;
                    std::vector<std::tuple<std::pair<double, double>, FusionVector, FusionVector>> imu_data;

                    for (size_t fileNo = 0; fileNo < csv_files.size(); fileNo++)
                    {
                        const std::string &imufn = csv_files.at(fileNo);
                        const std::string snFn = (fileNo >= sn_files.size()) ? ("") : (sn_files.at(fileNo));
                        const auto idToSn = MLvxCalib::GetIdToSnMapping(snFn);
                        // GetId of Imu to use
                        int imuNumberToUse = MLvxCalib::GetImuIdToUse(idToSn, imuSnToUse);
                        std::cout << "imuNumberToUse  " << imuNumberToUse << " at" << imufn << std::endl;
                        auto imu = load_imu(imufn.c_str(), imuNumberToUse);
                        std::cout << imufn << " with mapping " << snFn << std::endl;
                        imu_data.insert(std::end(imu_data), std::begin(imu), std::end(imu));
                    }

                    std::cout << "loading points" << std::endl;
                    std::vector<std::vector<Point3Di>> pointsPerFile;
                    pointsPerFile.resize(laz_files.size());
                    std::mutex mtx;
                    std::cout << "start std::transform" << std::endl;

                    std::transform(std::execution::par_unseq, std::begin(laz_files), std::end(laz_files), std::begin(pointsPerFile), [&](const std::string &fn)
                                   {
                                       // Load mapping from id to sn
                                       fs::path fnSn(fn);
                                       fnSn.replace_extension(".sn");

                                       // GetId of Imu to use
                                       const auto idToSn = MLvxCalib::GetIdToSnMapping(fnSn.string());
                                       auto calibration = MLvxCalib::CombineIntoCalibration(idToSn, preloadedCalibration);
                                       auto data = load_point_cloud(fn.c_str(), true, params.filter_threshold_xy, calibration);

                                       if (fn == laz_files.front())
                                       {
                                           fs::path calibrationValidtationFile = wdp / "calibrationValidation.asc";

                                           std::ofstream testPointcloud{calibrationValidtationFile.c_str()};
                                           for (const auto &p : data)
                                           {
                                               testPointcloud << p.point.x() << "\t" << p.point.y() << "\t" << p.point.z() << "\t" << p.intensity << "\t" << (int)p.lidarid << "\n";
                                           }
                                       }

                                       std::unique_lock lck(mtx);
                                       for (const auto &[id, calib] : calibration)
                                       {
                                           std::cout << " id : " << id << std::endl;
                                           std::cout << calib.matrix() << std::endl;
                                       }
                                       return data;
                                       // std::cout << fn << std::endl;
                                       //
                                   });
                    std::cout << "std::transform finished" << std::endl;

                    FusionAhrs ahrs;
                    FusionAhrsInitialise(&ahrs);

                    if (fusionConventionNwu)
                    {
                        ahrs.settings.convention = FusionConventionNwu;
                    }
                    if (fusionConventionEnu)
                    {
                        ahrs.settings.convention = FusionConventionEnu;
                    }
                    if (fusionConventionNed)
                    {
                        ahrs.settings.convention = FusionConventionNed;
                    }

                    std::map<double, std::pair<Eigen::Matrix4d, double>> trajectory;

                    int counter = 1;
                    for (const auto &[timestamp_pair, gyr, acc] : imu_data)
                    {
                        const FusionVector gyroscope = {static_cast<float>(gyr.axis.x * 180.0 / M_PI), static_cast<float>(gyr.axis.y * 180.0 / M_PI), static_cast<float>(gyr.axis.z * 180.0 / M_PI)};
                        const FusionVector accelerometer = {acc.axis.x, acc.axis.y, acc.axis.z};

                        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

                        FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);

                        Eigen::Quaterniond d{quat.element.w, quat.element.x, quat.element.y, quat.element.z};
                        Eigen::Affine3d t{Eigen::Matrix4d::Identity()};
                        t.rotate(d);

                        //
                        // TaitBryanPose rot_y;
                        // rot_y.px = rot_y.py = rot_y.pz = rot_y.px = rot_y.py = rot_y.pz;
                        // rot_y.fi = -5 * M_PI / 180.0;
                        // Eigen::Affine3d m_rot_y = affine_matrix_from_pose_tait_bryan(rot_y);
                        // t = t * m_rot_y;
                        //
                        // std::map<double, Eigen::Matrix4d> trajectory;
                        trajectory[timestamp_pair.first] = std::pair(t.matrix(), timestamp_pair.second);
                        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
                        counter++;
                        if (counter % 100 == 0)
                        {
                            printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f [%d of %d]\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw, counter++, imu_data.size());
                        }
                    }

                    int number_of_points = 0;
                    for (const auto &pp : pointsPerFile)
                    {
                        number_of_points += pp.size();
                    }
                    std::cout << "number of points: " << number_of_points << std::endl;
                    std::cout << "start transforming points" << std::endl;

                    int number_of_initial_points = 0;
                    double timestamp_begin;
                    for (const auto &pp : pointsPerFile)
                    {
                        // number_of_points += pp.size();
                        for (const auto &p : pp)
                        {
                            number_of_initial_points++;
                            params.initial_points.push_back(p);
                            if (number_of_initial_points > threshold_initial_points)
                            {
                                timestamp_begin = p.timestamp;
                                break;
                            }
                        }
                        if (number_of_initial_points > threshold_initial_points)
                        {
                            break;
                        }
                    }

                    std::cout << "timestamp_begin: " << timestamp_begin << std::endl;

                    std::vector<std::pair<double, double>> timestamps;
                    std::vector<Eigen::Affine3d> poses;
                    for (const auto &t : trajectory)
                    {
                        if (t.first >= timestamp_begin)
                        {
                            timestamps.emplace_back(t.first, t.second.second);
                            Eigen::Affine3d m;
                            m.matrix() = t.second.first;
                            poses.push_back(m);
                        }
                    }

                    std::cout << "poses.size(): " << poses.size() << std::endl;

                    if (poses.empty())
                    {
                        std::cerr << "Loading poses went wrong! Could not load poses!" << std::endl;
                        return;
                    }

                    int thershold = 20;
                    WorkerData wd;
                    // std::vector<double> temp_ts;
                    // temp_ts.reserve(1000000);

                    // int last_point = 0;
                    int index_begin = 0;

                    for (size_t i = 0; i < poses.size(); i++)
                    {
                        if (i % 1000 == 0)
                        {
                            std::cout << "preparing data " << i + 1 << " of " << poses.size() << std::endl;
                        }
                        wd.intermediate_trajectory.emplace_back(poses[i]);
                        wd.intermediate_trajectory_motion_model.emplace_back(poses[i]);
                        wd.intermediate_trajectory_timestamps.emplace_back(timestamps[i]);

                        //
                        TaitBryanPose tb = pose_tait_bryan_from_affine_matrix(poses[i]);
                        wd.imu_roll_pitch.emplace_back(tb.om, tb.fi);

                        // temp_ts.emplace_back(timestamps[i]);

                        if (wd.intermediate_trajectory.size() >= thershold)
                        {
                            /*auto index_lower = std::lower_bound(points.begin(), points.end(), wd.intermediate_trajectory_timestamps[0],
                                                                [](Point3Di lhs, double rhs) -> bool
                                                                { return lhs.timestamp < rhs; });
                            unsigned long long int i_begin = std::distance(points.begin(), index_lower);

                            auto index_upper = std::lower_bound(points.begin(), points.end(), wd.intermediate_trajectory_timestamps[wd.intermediate_trajectory_timestamps.size() - 1],
                                                                [](Point3Di lhs, double rhs) -> bool
                                                                { return lhs.timestamp < rhs; });
                            unsigned long long int i_end = std::distance(points.begin(), index_upper);*/

                            std::vector<Point3Di> points;
                            // for (const auto &pp : pointsPerFile)
                            //{
                            //     for (const auto &p : pp)
                            //     {
                            //         if (p.timestamp >= wd.intermediate_trajectory_timestamps[0] && p.timestamp <= wd.intermediate_trajectory_timestamps[wd.intermediate_trajectory_timestamps.size() - 1]){
                            //             points.push_back(p);
                            //         }
                            //     }
                            // }
                            bool found = false;

                            for (int index = index_begin; index < pointsPerFile.size(); index++)
                            {
                                for (const auto &p : pointsPerFile[index])
                                {
                                    if (p.timestamp >= wd.intermediate_trajectory_timestamps[0].first && p.timestamp <= wd.intermediate_trajectory_timestamps[wd.intermediate_trajectory_timestamps.size() - 1].first)
                                    {
                                        points.push_back(p);
                                    }
                                    if (p.timestamp >= wd.intermediate_trajectory_timestamps[0].first && !found)
                                    {
                                        index_begin = index;
                                        found = true;
                                    }
                                    if (p.timestamp > wd.intermediate_trajectory_timestamps[wd.intermediate_trajectory_timestamps.size() - 1].first)
                                    {
                                        break;
                                    }
                                }
                            }

                            // for (unsigned long long int k = i_begin; k < i_end; k++)
                            // if (i % 1000 == 0)
                            //{
                            // std::cout << "points.size() " << points.size() << std::endl;
                            //}

                            for (unsigned long long int k = 0; k < points.size(); k++)
                            {
                                // if (points[k].timestamp > wd.intermediate_trajectory_timestamps[0] && points[k].timestamp < wd.intermediate_trajectory_timestamps[wd.intermediate_trajectory_timestamps.size() - 1])
                                //{
                                auto p = points[k];
                                auto lower = std::lower_bound(wd.intermediate_trajectory_timestamps.begin(), wd.intermediate_trajectory_timestamps.end(), p.timestamp,
                                                              [](std::pair<double, double> lhs, double rhs) -> bool
                                                              { return lhs.first < rhs; });

                                p.index_pose = std::distance(wd.intermediate_trajectory_timestamps.begin(), lower);
                                wd.intermediate_points.emplace_back(p);
                                wd.original_points.emplace_back(p);
                                //}
                            }

                            if (params.decimation > 0.0)
                            {
                                wd.intermediate_points = decimate(wd.intermediate_points, params.decimation, params.decimation, params.decimation);
                            }

                            worker_data.push_back(wd);
                            wd.intermediate_points.clear();
                            wd.original_points.clear();
                            wd.intermediate_trajectory.clear();
                            wd.intermediate_trajectory_motion_model.clear();
                            wd.intermediate_trajectory_timestamps.clear();
                            wd.imu_roll_pitch.clear();

                            wd.intermediate_points.reserve(1000000);
                            wd.original_points.reserve(1000000);
                            wd.intermediate_trajectory.reserve(1000);
                            wd.intermediate_trajectory_motion_model.reserve(1000);
                            wd.intermediate_trajectory_timestamps.reserve(1000);
                            wd.imu_roll_pitch.reserve(1000);

                            // temp_ts.clear();
                        }
                    }

                    params.m_g = worker_data[0].intermediate_trajectory[0];
                    step_1_done = true;
                    std::cout << "step_1_done please click 'compute_all (step 2)' to continue calculations" << std::endl; 
                }
                else
                {
                    std::cout << "please select files correctly" << std::endl;
                }
            }
            ImGui::SameLine();
            ImGui::Text("Select all imu *.csv and lidar *.laz files produced by MANDEYE saved in 'continousScanning_*' folder");
        }
        if (step_1_done && !step_2_done)
        {
            if (ImGui::Button("compute_all (step 2)"))
            {
                compute_step_2(worker_data, params);
                step_2_done = true;
            }
            ImGui::SameLine();
            ImGui::Text("Press this button for automatic lidar odometry calculation -> it will produce trajectory");
            // if (ImGui::Button("fix pitch roll"))
            //{
            //     fix_ptch_roll(worker_data);
            // }
        }
        /*if (step_1_done && !step_2_done)
        {
            if (ImGui::Button("compute_all fast forward motion(step 2)"))
            {
                compute_step_2_fast_forward_motion(worker_data, params);
                step_2_done = true;
            }
            ImGui::SameLine();
            ImGui::Text("Press this button for automatic lidar odometry calculation -> it will produce trajectory");
        }*/
        if (step_1_done && step_2_done && !step_3_done)
        {
            ImGui::Text("'Consistency' makes trajectory smooth, point cloud will be more consistent");
            if (ImGui::Button("Consistency"))
            {
                std::cout << "Consistency START" << std::endl;

                for (int i = 0; i < num_constistency_iter; i++)
                {
                    std::cout << "Iteration " << i + 1 << " of " << num_constistency_iter << std::endl;
                    for (int ii = 0; ii < worker_data.size(); ii++)
                    {
                        worker_data[ii].intermediate_trajectory_motion_model = worker_data[ii].intermediate_trajectory;
                    }
                    Consistency(worker_data, params);
                }
                std::cout << "Consistency FINISHED" << std::endl;
            }
            ImGui::SameLine();
            ImGui::Text("Press this button optionally before pressing 'save result (step 3)'");

            if (ImGui::Button("save result (step 3)"))
            {
                // concatenate data
                std::vector<WorkerData> worker_data_concatenated;
                WorkerData wd;
                int counter = 0;
                int pose_offset = 0;
                for (int i = 0; i < worker_data.size(); i++)
                {
                    if (i % 1000 == 0)
                    {
                        printf("processing worker_data [%d] of %d \n", i + 1, worker_data.size());
                    }
                    auto tmp_data = worker_data[i].original_points;

                    for (auto &t : tmp_data)
                    {
                        t.index_pose += pose_offset;
                    }

                    wd.intermediate_trajectory.insert(std::end(wd.intermediate_trajectory),
                                                      std::begin(worker_data[i].intermediate_trajectory), std::end(worker_data[i].intermediate_trajectory));

                    wd.intermediate_trajectory_timestamps.insert(std::end(wd.intermediate_trajectory_timestamps),
                                                                 std::begin(worker_data[i].intermediate_trajectory_timestamps), std::end(worker_data[i].intermediate_trajectory_timestamps));

                    wd.original_points.insert(std::end(wd.original_points),
                                              std::begin(tmp_data), std::end(tmp_data));

                    pose_offset += worker_data[i].intermediate_trajectory.size();

                    counter++;
                    if (counter > 50)
                    {
                        worker_data_concatenated.push_back(wd);
                        wd.intermediate_trajectory.clear();
                        wd.intermediate_trajectory_timestamps.clear();
                        wd.original_points.clear();
                        counter = 0;
                        pose_offset = 0;
                    }
                }

                if (counter > 10)
                {
                    worker_data_concatenated.push_back(wd);
                }

                std::vector<Eigen::Affine3d> m_poses;
                std::vector<std::string> file_names;
                for (int i = 0; i < worker_data_concatenated.size(); i++)
                {
                    std::cout << "------------------------" << std::endl;
                    fs::path path(working_directory);
                    std::string filename = ("scan_lio_" + std::to_string(i) + ".laz");
                    path /= filename;
                    std::cout << "saving to: " << path << std::endl;
                    saveLaz(path.string(), worker_data_concatenated[i]);
                    m_poses.push_back(worker_data_concatenated[i].intermediate_trajectory[0]);
                    file_names.push_back(filename);

                    // save trajectory
                    std::string trajectory_filename = ("trajectory_lio_" + std::to_string(i) + ".csv");
                    fs::path pathtrj(working_directory);
                    pathtrj /= trajectory_filename;
                    std::cout << "saving to: " << pathtrj << std::endl;

                    ///
                    std::ofstream outfile;
                    outfile.open(pathtrj);
                    if (!outfile.good())
                    {
                        std::cout << "can not save file: " << pathtrj << std::endl;
                        return;
                    }

                    outfile << "timestamp_nanoseconds pose00 pose01 pose02 pose03 pose10 pose11 pose12 pose13 pose20 pose21 pose22 pose23 timestampUnix_nanoseconds" << std::endl;
                    for (int j = 0; j < worker_data_concatenated[i].intermediate_trajectory.size(); j++)
                    {
                        auto pose = worker_data_concatenated[i].intermediate_trajectory[0].inverse() * worker_data_concatenated[i].intermediate_trajectory[j];

                        outfile
                            << std::setprecision(20) << worker_data_concatenated[i].intermediate_trajectory_timestamps[j].first * 1e9 << " " << std::setprecision(10)
                            << pose(0, 0) << " "
                            << pose(0, 1) << " "
                            << pose(0, 2) << " "
                            << pose(0, 3) << " "
                            << pose(1, 0) << " "
                            << pose(1, 1) << " "
                            << pose(1, 2) << " "
                            << pose(1, 3) << " "
                            << pose(2, 0) << " "
                            << pose(2, 1) << " "
                            << pose(2, 2) << " "
                            << pose(2, 3) << " "
                            << std::setprecision(20) << worker_data_concatenated[i].intermediate_trajectory_timestamps[j].second * 1e9 << std::endl;
                    }
                    outfile.close();
                    //
                }
                fs::path path(working_directory);
                path /= "lio_initial_poses.reg";
                save_poses(path.string(), m_poses, file_names);
                fs::path path2(working_directory);
                path2 /= "poses.reg";
                save_poses(path2.string(), m_poses, file_names);

                fs::path path3(working_directory);
                path3 /= "session.json";

                // save session file
                std::cout << "saving file: '" << path3 << "'" << std::endl;

                nlohmann::json jj;
                nlohmann::json j;

                j["offset_x"] = 0.0;
                j["offset_y"] = 0.0;
                j["offset_z"] = 0.0;
                j["folder_name"] = working_directory;
                j["out_folder_name"] = working_directory;
                j["poses_file_name"] = path2.string();
                j["initial_poses_file_name"] = path.string();
                j["out_poses_file_name"] = path2.string();
                j["lidar_odometry_version"] = 0.43;

                jj["Session Settings"] = j;

                nlohmann::json jlaz_file_names;
                for (int i = 0; i < worker_data_concatenated.size(); i++)
                {
                    fs::path path(working_directory);
                    std::string filename = ("scan_lio_" + std::to_string(i) + ".laz");
                    path /= filename;
                    std::cout << "adding file: " << path << std::endl;

                    nlohmann::json jfn{
                        {"file_name", path.string()}};
                    jlaz_file_names.push_back(jfn);
                }
                jj["laz_file_names"] = jlaz_file_names;

                std::ofstream fs(path3.string());
                fs << jj.dump(2);
                fs.close();
                step_3_done = true;
            }
            ImGui::SameLine();
            ImGui::Text("Press this button for saving resulting trajectory and point clouds as single session for 'multi_view_tls_registration_step_2' program");
        }
        if (step_3_done)
        {
            ImGui::Text("-------------------------------------------------------------------------------");
            ImGui::Text(std::string("All data is saved in folder '" + working_directory + "' You can close this program.").c_str());
            ImGui::Text("Next step is to load 'session.json' with 'multi_view_tls_registration_step_2' program");
            ImGui::Text("-------------------------------------------------------------------------------");

            if (ImGui::Button("save all point clouds to single '*.las or *.laz' file"))
            {
                std::shared_ptr<pfd::save_file> save_file;
                std::string output_file_name = "";
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
                const auto t = [&]()
                {
                    auto sel = pfd::save_file("Save las or laz file", "C:\\", LAS_LAZ_filter).result();
                    output_file_name = sel;
                    std::cout << "las or laz file to save: '" << output_file_name << "'" << std::endl;
                };
                std::thread t1(t);
                t1.join();

                if (output_file_name.size() > 0)
                {
                    std::vector<Eigen::Vector3d> pointcloud;
                    std::vector<unsigned short> intensity;

                    for (int i = 0; i < worker_data.size(); i++)
                    {
                        for (const auto &p : worker_data[i].intermediate_points)
                        {
                            Eigen::Vector3d pt = worker_data[i].intermediate_trajectory[p.index_pose] * p.point;
                            pointcloud.push_back(pt);
                            intensity.push_back(p.intensity);
                        }
                    }
                    if (!exportLaz(output_file_name, pointcloud, intensity, 0, 0, 0))
                    {
                        std::cout << "problem with saving file: " << output_file_name << std::endl;
                    }
                    /*for (auto &p : session.point_clouds_container.point_clouds)
                    {
                        if (p.visible)
                        {
                            for (int i = 0; i < p.points_local.size(); i++)
                            {
                                const auto &pp = p.points_local[i];
                                Eigen::Vector3d vp;
                                vp = p.m_pose * pp + session.point_clouds_container.offset;

                                pointcloud.push_back(vp);
                                if (i < p.intensities.size())
                                {
                                    intensity.push_back(p.intensities[i]);
                                }
                                else
                                {
                                    intensity.push_back(0);
                                }
                            }
                        }
                    }
                    */
                }
            }
        }
        if (!simple_gui)
        {
            ImGui::SameLine();
            if (ImGui::Button("save trajectory to ascii (x y z)"))
            {
                static std::shared_ptr<pfd::save_file> save_file;
                std::string output_file_name = "";
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
                const auto t = [&]()
                {
                    auto sel = pfd::save_file("Save trajectory", "C:\\").result();
                    output_file_name = sel;
                    std::cout << "file to save: '" << output_file_name << "'" << std::endl;
                };
                std::thread t1(t);
                t1.join();

                if (output_file_name.size() > 0)
                {
                    ofstream file;
                    file.open(output_file_name);
                    for (const auto &wd : worker_data)
                    {
                        for (const auto &it : wd.intermediate_trajectory)
                        {
                            file << it(0, 3) << " " << it(1, 3) << " " << it(2, 3) << std::endl;
                        }
                    }
                    file.close();
                }
            }
            if (ImGui::Button("load reference point clouds (laz)"))
            {
                static std::shared_ptr<pfd::open_file> open_file;
                std::vector<std::string> input_file_names;
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
                const auto t = [&]()
                {
                    std::vector<std::string> filters;
                    auto sel = pfd::open_file("Load las files", "C:\\", filters, true).result();
                    for (int i = 0; i < sel.size(); i++)
                    {
                        input_file_names.push_back(sel[i]);
                    }
                };
                std::thread t1(t);
                t1.join();

                if (input_file_names.size() > 0)
                {
                    params.reference_buckets.clear();
                    params.reference_points.clear();

                    for (size_t i = 0; i < input_file_names.size(); i++)
                    {
                        std::cout << "loading reference point cloud from: " << input_file_names[i] << std::endl;
                        auto pp = load_point_cloud(input_file_names[i].c_str(), false, 0, {});
                        std::cout << "loaded " << pp.size() << " reference points" << std::endl;
                        params.reference_points.insert(std::end(params.reference_points), std::begin(pp), std::end(pp));
                    }

                    update_rgd(params.in_out_params, params.reference_buckets, params.reference_points);
                    show_reference_points = true;
                }
            }

            ImGui::SameLine();
            ImGui::Checkbox("show reference points", &show_reference_points);
            ImGui::Checkbox("show reference buckets", &show_reference_buckets);
            ImGui::InputInt("decimation reference points", &dec_reference_points);

            if (params.initial_points.size() > 0)
            {
                ImGui::Text("-----manipulate initial transformation begin-------");
                ImGui::Checkbox("initial transformation gizmo", &initial_transformation_gizmo);
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
                if (!initial_transformation_gizmo)
                {
                    if (ImGui::Button("Align to reference"))
                    {
                        for (int i = 0; i < 30; i++)
                        {
                            align_to_reference(params.in_out_params, params.initial_points, params.m_g, params.reference_buckets);
                        }
                    }
                }
                ImGui::Text("-----manipulate initial transformation end---------");
            }

            if (ImGui::Button("select all scans"))
            {
                for (int k = 0; k < worker_data.size(); k++)
                {
                    worker_data[k].show = true;
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("unselect all scans"))
            {
                for (int k = 0; k < worker_data.size(); k++)
                {
                    worker_data[k].show = false;
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("select scans from range <index_from_inclusive, index_to_inclusive>"))
            {
                for (int k = 0; k < worker_data.size(); k++)
                {
                    if (k >= index_from_inclusive && k <= index_to_inclusive)
                    {
                        worker_data[k].show = true;
                    }
                    else
                    {
                        worker_data[k].show = false;
                    }
                }
            }
            //
            ImGui::InputInt("index_from_inclusive", &index_from_inclusive);
            if (index_from_inclusive < 0)
            {
                index_from_inclusive = 0;
            }
            if (index_from_inclusive >= worker_data.size())
            {
                index_from_inclusive = worker_data.size() - 1;
            }

            int prev = index_to_inclusive;
            ImGui::InputInt("index_to_inclusive", &index_to_inclusive);
            if (index_to_inclusive < 0)
            {
                index_to_inclusive = 0;
            }
            if (index_to_inclusive >= worker_data.size())
            {
                index_to_inclusive = worker_data.size() - 1;
            }
            if (prev != index_to_inclusive)
            {
                stretch_gizmo_m = worker_data[index_to_inclusive].intermediate_trajectory[0];
            }


            if (index_to_inclusive > index_from_inclusive)
            {
                ImGui::Text("-------------------------");
                ImGui::Checkbox("gizmo_stretch_interval", &gizmo_stretch_interval);
            }

            // gizmo_stretch_interval
            if (gizmo_stretch_interval)
            {
                if (index_to_inclusive < worker_data.size())
                {
                    if (worker_data[index_to_inclusive].intermediate_trajectory.size() > 0)
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
                /*if (index_to_inclusive < worker_data.size())
                {
                    if (index_from_inclusive < index_to_inclusive)
                    {
                        if (ImGui::Button("Accept_Gizmo"))
                        {
                            std::vector<std::vector<Eigen::Affine3d>> all_poses;
                            for (int i = 0; i < worker_data.size(); i++)
                            {
                                std::vector<Eigen::Affine3d> poses;
                                for (int j = 0; j < worker_data[i].intermediate_trajectory.size(); j++)
                                {
                                    poses.push_back(worker_data[i].intermediate_trajectory[j]);
                                }
                                all_poses.push_back(poses);
                            }
                        }
                    }
                }*/

                if (ImGui::Button("Accept_Gizmo (only translation)"))
                {
                    if (index_to_inclusive < worker_data.size())
                    {
                        if (worker_data[index_to_inclusive].intermediate_trajectory.size() > 0)
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

                            auto first_pose = worker_data[index_from_inclusive].intermediate_trajectory[0];

                            Eigen::Vector3d translation = current_gizmo.translation() - first_pose.translation();

                            float number_all_nodes_inside_interval = 0;
                            for (int i = index_from_inclusive; i < index_to_inclusive; i++)
                            {
                                number_all_nodes_inside_interval += (float)worker_data[i].intermediate_trajectory.size();
                            }

                            std::vector<std::vector<Eigen::Affine3d>> all_poses;
                            for (int i = 0; i < worker_data.size(); i++)
                            {
                                std::vector<Eigen::Affine3d> poses;
                                for (int j = 0; j < worker_data[i].intermediate_trajectory.size(); j++)
                                {
                                    poses.push_back(worker_data[i].intermediate_trajectory[j]);
                                }
                                all_poses.push_back(poses);
                            }

                            float counter = 0;

                            Eigen::Affine3d last_m = Eigen::Affine3d::Identity();

                            for (int i = index_from_inclusive; i < index_to_inclusive; i++)
                            {
                                for (int j = 0; j < worker_data[i].intermediate_trajectory.size(); j++)
                                {
                                    TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(worker_data[i].intermediate_trajectory[j]);

                                    pose.px = first_pose.translation().x() + translation.x() * (counter / number_all_nodes_inside_interval);
                                    pose.py = first_pose.translation().y() + translation.y() * (counter / number_all_nodes_inside_interval);
                                    pose.pz = first_pose.translation().z() + translation.z() * (counter / number_all_nodes_inside_interval);

                                    // pose.om += pose_diff.om * (counter / number_all_nodes_inside_interval);
                                    // pose.fi += pose_diff.fi * (counter / number_all_nodes_inside_interval);
                                    // pose.ka += pose_diff.ka * (counter / number_all_nodes_inside_interval);

                                    counter += 1.0f;

                                    worker_data[i].intermediate_trajectory[j] = affine_matrix_from_pose_tait_bryan(pose);

                                    last_m = worker_data[i].intermediate_trajectory[j];
                                }
                            }

                            for (int i = index_to_inclusive; i < worker_data.size(); i++)
                            {
                                for (int j = 0; j < worker_data[i].intermediate_trajectory.size(); j++)
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
                ImGui::Text("-------------------------");
            }

            for (int i = 0; i < worker_data.size(); i++)
            {
                std::string text = "show[" + std::to_string(i) + "]";
                ImGui::Checkbox(text.c_str(), &worker_data[i].show);
            }

            if (ImGui::Button("filter reference buckets"))
            {
                NDTBucketMapType reference_buckets_out;
                for (const auto &b : params.reference_buckets)
                {
                    if (b.second.number_of_points > 10)
                    {
                        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(b.second.cov, Eigen::ComputeEigenvectors);

                        auto eigen_values = eigen_solver.eigenvalues();
                        double sum_ev = eigen_values.x() + eigen_values.y() + eigen_values.z();
                        double ev1 = eigen_values.x();
                        double ev2 = eigen_values.y();
                        double ev3 = eigen_values.z();

                        double planarity = 1 - ((3 * ev1 / sum_ev) * (3 * ev2 / sum_ev) * (3 * ev3 / sum_ev));
                        if (planarity > 0.7)
                        {
                            reference_buckets_out[b.first] = b.second;
                        }
                    }
                }
                params.reference_buckets = reference_buckets_out;
            }
        }
        ImGui::End();
    }
}

void mouse(int glut_button, int state, int x, int y)
{
    ImGuiIO &io = ImGui::GetIO();
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

    if (!io.WantCaptureMouse)
    {
        if (glut_button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN && io.KeyCtrl)
        {
            const auto laser_beam = GetLaserBeam(x, y);

            RegistrationPlaneFeature::Plane pl;

            pl.a = 0;
            pl.b = 0;
            pl.c = 1;
            pl.d = 0;
            auto old_Totation_center = rotation_center;
            rotation_center = rayIntersection(laser_beam, pl).cast<float>();

            std::cout << "setting new rotation center to " << rotation_center << std::endl;

            rotate_x = 0.f;
            rotate_y = 0.f;
        }

        if (state == GLUT_DOWN)
        {
            mouse_buttons |= 1 << glut_button;
        }
        else if (state == GLUT_UP)
        {
            mouse_buttons = 0;
        }
        mouse_old_x = x;
        mouse_old_y = y;
    }
}

void wheel(int button, int dir, int x, int y)
{
    if (dir > 0)
    {
        translate_z -= 0.05f * translate_z;
    }
    else
    {
        translate_z += 0.05f * translate_z;
    }
    return;
}

void reshape(int w, int h)
{
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (GLfloat)w / (GLfloat)h, 0.01, 10000.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void motion(int x, int y)
{
    ImGuiIO &io = ImGui::GetIO();
    io.MousePos = ImVec2((float)x, (float)y);

    if (!io.WantCaptureMouse)
    {
        float dx, dy;
        dx = (float)(x - mouse_old_x);
        dy = (float)(y - mouse_old_y);

        gui_mouse_down = mouse_buttons > 0;
        if (mouse_buttons & 1)
        {
            rotate_x += dy * 0.2f;
            rotate_y += dx * 0.2f;
        }
        if (mouse_buttons & 4)
        {
            translate_x += dx * 0.5f * mouse_sensitivity;
            translate_y -= dy * 0.5f * mouse_sensitivity;
        }

        mouse_old_x = x;
        mouse_old_y = y;
    }
    glutPostRedisplay();
}

void display()
{
    ImGuiIO &io = ImGui::GetIO();
    glViewport(0, 0, (GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);

    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    // reshape((GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);
    // glTranslatef(translate_x, translate_y, translate_z);
    // glRotatef(rotate_x, 1.0, 0.0, 0.0);
    // glRotatef(rotate_y, 0.0, 0.0, 1.0);

    reshape((GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);
    Eigen::Affine3f viewTranslation = Eigen::Affine3f::Identity();
    viewTranslation.translate(rotation_center);
    Eigen::Affine3f viewLocal = Eigen::Affine3f::Identity();
    viewLocal.translate(Eigen::Vector3f(translate_x, translate_y, translate_z));
    viewLocal.rotate(Eigen::AngleAxisf(M_PI * rotate_x / 180.f, Eigen::Vector3f::UnitX()));
    viewLocal.rotate(Eigen::AngleAxisf(M_PI * rotate_y / 180.f, Eigen::Vector3f::UnitZ()));

    Eigen::Affine3f viewTranslation2 = Eigen::Affine3f::Identity();
    viewTranslation2.translate(-rotation_center);

    Eigen::Affine3f result = viewTranslation * viewLocal * viewTranslation2;

    glLoadMatrixf(result.matrix().data());

    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(100, 0.0f, 0.0f);

    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 100, 0.0f);

    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 100);
    glEnd();

    // nv
    /*glBegin(GL_LINES);
    for (const auto &b : buckets)
    {
        glColor3f(b.second.normal_vector.x(), b.second.normal_vector.y(), b.second.normal_vector.z());
        glVertex3f(b.second.mean.x(), b.second.mean.y(), b.second.mean.z());
        glVertex3f(b.second.mean.x() + b.second.normal_vector.x(), b.second.mean.y() + b.second.normal_vector.y(), b.second.mean.z() + b.second.normal_vector.z());
    }
    glEnd();*/

    /*if (show_all_points)
    {
        glColor3d(1.0, 0.0, 0.0);
        glBegin(GL_POINTS);
        for (const auto &p : all_points)
        {
            glVertex3d(p.x(), p.y(), p.z());
        }
        glEnd();
    }*/
    if (show_initial_points)
    {
        glColor3d(0.0, 1.0, 0.0);
        glBegin(GL_POINTS);
        for (const auto &p : params.initial_points)
        {
            auto pp = params.m_g * p.point;
            glVertex3d(pp.x(), pp.y(), pp.z());
        }
        glEnd();
    }
    // if(show_covs){
    //     for(int i = 0; i < means.size(); i += dec_covs){
    //         draw_ellipse(covs[i], means[i], Eigen::Vector3f(0.0f, 0.0f, 1.0f), 3);
    //     }
    // }
    if (show_covs)
    {
        for (const auto &b : params.buckets)
        {
            draw_ellipse(b.second.cov, b.second.mean, Eigen::Vector3f(0.0f, 0.0f, 1.0f), 3);
        }
    }

    for (int i = 0; i < worker_data.size(); i++)
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
            // glLineWidth(1);

            /*glColor3f(0, 0, 1);
            glVertex3f(it(0, 3), it(1, 3), it(2, 3));
            glVertex3f(it(0, 3) + it(0, 2), it(1, 3) + it(1, 2), it(2, 3) + it(2, 2));
            glEnd();
            {
                TaitBryanPose tb = pose_tait_bryan_from_affine_matrix(worker_data[i].intermediate_trajectory[0]);
                // tb.px = tb.py = tb.pz = tb.om = tb.fi = tb.ka = 0.0;
                tb.om = worker_data[i].imu_roll_pitch[0].first;
                tb.fi = worker_data[i].imu_roll_pitch[0].second;
                Eigen::Affine3d it = affine_matrix_from_pose_tait_bryan(tb);
                glLineWidth(3);
                glBegin(GL_LINES);

                glColor3f(1, 0, 0);
                glVertex3f(it(0, 3), it(1, 3), it(2, 3));
                glVertex3f(it(0, 3) + it(0, 0) * 0.5, it(1, 3) + it(1, 0) * 0.5, it(2, 3) + it(2, 0) * 0.5);

                glColor3f(0, 1, 0);
                glVertex3f(it(0, 3), it(1, 3), it(2, 3));
                glVertex3f(it(0, 3) + it(0, 1) * 0.5, it(1, 3) + it(1, 1) * 0.5, it(2, 3) + it(2, 1) * 0.5);

                glColor3f(0, 0, 1);
                glVertex3f(it(0, 3), it(1, 3), it(2, 3));
                glVertex3f(it(0, 3) + it(0, 2) * 0.5, it(1, 3) + it(1, 2) * 0.5, it(2, 3) + it(2, 2) * 0.5);

                glEnd();
                glLineWidth(1);
            }*/
        }
    }

    if (show_reference_points)
    {
        glColor3f(1, 0, 0);
        glBegin(GL_POINTS);
        for (int i = 0; i < params.reference_points.size(); i += dec_reference_points)
        {
            glVertex3f(params.reference_points[i].point.x(), params.reference_points[i].point.y(), params.reference_points[i].point.z());
        }
        glEnd();
    }

    if (show_trajectory_as_axes)
    {
        glColor3f(0, 1, 0);
        // glBegin(GL_LINE_STRIP);
        glBegin(GL_LINES);
        for (const auto &wd : worker_data)
        {
            for (const auto &it : wd.intermediate_trajectory)
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
        for (const auto &wd : worker_data)
        {
            for (const auto &it : wd.intermediate_trajectory)
            {
                glVertex3f(it(0, 3), it(1, 3), it(2, 3));
            }
        }
        glEnd();
        glPointSize(1);
    }

    if (show_reference_buckets)
    {
        glColor3f(1, 0, 0);
        glBegin(GL_POINTS);
        for (const auto &b : params.reference_buckets)
        {
            glVertex3f(b.second.mean.x(), b.second.mean.y(), b.second.mean.z());
            //std::cout << b.second.mean << " ";
        }
        glEnd();
    }

    //
    if (worker_data.size() > 0)
    {
        if (index_from_inclusive < worker_data.size())
        {
            glColor3f(1, 1, 0);
            glBegin(GL_LINES);
            if (worker_data[index_from_inclusive].intermediate_trajectory.size() > 0)
            {
                auto p = worker_data[index_from_inclusive].intermediate_trajectory[0].translation();
                glVertex3f(p.x() - 1, p.y(), p.z());
                glVertex3f(p.x() + 1, p.y(), p.z());

                glVertex3f(p.x(), p.y() - 1, p.z());
                glVertex3f(p.x(), p.y() + 1, p.z());

                glVertex3f(p.x(), p.y(), p.z() - 1);
                glVertex3f(p.x(), p.y(), p.z() + 1);
            }
            glEnd();
        }

        if (index_to_inclusive < worker_data.size())
        {
            glColor3f(0, 1, 1);
            glBegin(GL_LINES);
            if (worker_data[index_to_inclusive].intermediate_trajectory.size() > 0)
            {
                auto p = worker_data[index_to_inclusive].intermediate_trajectory[0].translation();
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
    // int index_from_inclusive = -1;
    // int index_to_inclusive = -1;

    /*{
        for (int k = 0; k < worker_data.size(); k++)
        {
            worker_data[k].show = false;
        }
    }*/

    if (ImGui::GetIO().KeyCtrl)
    {
        glBegin(GL_LINES);
        glColor3f(1.f, 1.f, 1.f);
        glVertex3fv(rotation_center.data());
        glVertex3f(rotation_center.x() + 1.f, rotation_center.y(), rotation_center.z());
        glVertex3fv(rotation_center.data());
        glVertex3f(rotation_center.x() - 1.f, rotation_center.y(), rotation_center.z());
        glVertex3fv(rotation_center.data());
        glVertex3f(rotation_center.x(), rotation_center.y() - 1.f, rotation_center.z());
        glVertex3fv(rotation_center.data());
        glVertex3f(rotation_center.x(), rotation_center.y() + 1.f, rotation_center.z());
        glVertex3fv(rotation_center.data());
        glVertex3f(rotation_center.x(), rotation_center.y(), rotation_center.z() - 1.f);
        glVertex3fv(rotation_center.data());
        glVertex3f(rotation_center.x(), rotation_center.y(), rotation_center.z() + 1.f);
        glEnd();
    }

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGLUT_NewFrame();

    lidar_odometry_gui();

    if (initial_transformation_gizmo)
    {
        ImGuiIO &io = ImGui::GetIO();
        // ImGuizmo -----------------------------------------------
        ImGuizmo::BeginFrame();
        ImGuizmo::Enable(true);
        ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

        GLfloat projection[16];
        glGetFloatv(GL_PROJECTION_MATRIX, projection);

        GLfloat modelview[16];
        glGetFloatv(GL_MODELVIEW_MATRIX, modelview);

        ImGuizmo::Manipulate(&modelview[0], &projection[0], ImGuizmo::TRANSLATE | ImGuizmo::ROTATE_Z | ImGuizmo::ROTATE_X | ImGuizmo::ROTATE_Y, ImGuizmo::LOCAL, m_gizmo, NULL);

        // Eigen::Affine3d m_g = Eigen::Affine3d::Identity();

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
        ImGuiIO &io = ImGui::GetIO();
        // ImGuizmo -----------------------------------------------
        ImGuizmo::BeginFrame();
        ImGuizmo::Enable(true);
        ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

        GLfloat projection[16];
        glGetFloatv(GL_PROJECTION_MATRIX, projection);

        GLfloat modelview[16];
        glGetFloatv(GL_MODELVIEW_MATRIX, modelview);

        ImGuizmo::Manipulate(&modelview[0], &projection[0], ImGuizmo::TRANSLATE | ImGuizmo::ROTATE_Z | ImGuizmo::ROTATE_X | ImGuizmo::ROTATE_Y, ImGuizmo::WORLD, m_gizmo, NULL);

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

    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

    glutSwapBuffers();
    glutPostRedisplay();
}

bool initGL(int *argc, char **argv)
{
    glutInit(argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowSize(window_width, window_height);
    glutCreateWindow("lidar_odometry");
    glutDisplayFunc(display);
    glutMotionFunc(motion);

    // default initialization
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glEnable(GL_DEPTH_TEST);

    // viewport
    glViewport(0, 0, window_width, window_height);

    // projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (GLfloat)window_width / (GLfloat)window_height, 0.01, 10000.0);
    glutReshapeFunc(reshape);
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls

    ImGui::StyleColorsDark();
    ImGui_ImplGLUT_Init();
    ImGui_ImplGLUT_InstallFuncs();
    ImGui_ImplOpenGL2_Init();
    return true;
}

int main(int argc, char *argv[])
{
    params.in_out_params.resolution_X = 0.3;
    params.in_out_params.resolution_Y = 0.3;
    params.in_out_params.resolution_Z = 0.3;
    params.in_out_params.bounding_box_extension = 20.0;

    initGL(&argc, argv);
    glutDisplayFunc(display);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutMouseWheelFunc(wheel);
    glutMainLoop();

    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGLUT_Shutdown();

    ImGui::DestroyContext();
    return 0;
}

std::vector<std::vector<Point3Di>> get_batches_of_points(std::string laz_file, int point_count_threshold, std::vector<Point3Di> prev_points)
{
    std::vector<std::vector<Point3Di>> res_points;
    std::vector<Point3Di> points = load_point_cloud(laz_file, false, 0, {});

    std::vector<Point3Di> tmp_points = prev_points;
    int counter = tmp_points.size();
    for (int i = 0; i < points.size(); i++)
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

void find_best_stretch(std::vector<Point3Di> points, std::vector<double> timestamps, std::vector<Eigen::Affine3d> poses, std::string fn1, std::string fn2)
{
    for (int i = 0; i < points.size(); i++)
    {
        auto lower = std::lower_bound(timestamps.begin(), timestamps.end(), points[i].timestamp);
        points[i].index_pose = std::distance(timestamps.begin(), lower);
        // std::cout << "points[i].timestamp " << points[i].timestamp << " timestamps " << timestamps[points[i].index_pose] << std::endl;
    }

    std::set<int> indexes;

    for (int i = 0; i < points.size(); i++)
    {
        indexes.insert(points[i].index_pose);
    }
    // build trajectory
    std::vector<Eigen::Affine3d> trajectory;
    std::vector<double> ts;

    for (auto &s : indexes)
    {
        trajectory.push_back(poses[s]);
        ts.push_back(timestamps[s]);
    }

    std::cout << "trajectory.size() " << trajectory.size() << std::endl;
    // Sleep(2000);

    std::vector<Point3Di> points_reindexed = points;
    for (int i = 0; i < points_reindexed.size(); i++)
    {
        points_reindexed[i].index_pose = get_index(indexes, points[i].index_pose);
    }
    ///
    std::vector<Eigen::Affine3d> best_trajectory = trajectory;
    int min_buckets = 1000000000000;

    for (double x = 0.0; x < 0.2; x += 0.0005)
    {
        std::vector<Eigen::Affine3d> trajectory_stretched;

        Eigen::Affine3d m_x_offset = Eigen::Affine3d::Identity();
        m_x_offset(0, 3) = x;

        Eigen::Affine3d m = trajectory[0];
        trajectory_stretched.push_back(m);
        for (int i = 1; i < trajectory.size(); i++)
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

        for (auto &p : points_global)
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
    for (int i = 0; i < best_trajectory.size(); i++)
    {
        trajectory_for_interpolation[ts[i]] = best_trajectory[i].matrix();
    }

    std::vector<Point3Di> points_global = points_reindexed;
    for (auto &p : points_global)
    {
        Eigen::Matrix4d pose = getInterpolatedPose(trajectory_for_interpolation, /*ts[p.index_pose]*/ p.timestamp);
        Eigen::Affine3d b;
        b.matrix() = pose;
        p.point = b * p.point;
    }

    std::cout << "saving file: " << fn1 << std::endl;
    saveLaz(fn1, points_global);

    points_global = points_reindexed;
    for (auto &p : points_global)
    {
        p.point = trajectory[p.index_pose] * p.point;
    }
    saveLaz(fn2, points_global);
}

void alternative_approach()
{
    int point_count_threshold = 10000;

    std::cout << "aternative_approach" << std::endl;

    static std::shared_ptr<pfd::open_file> open_file;
    std::vector<std::string> input_file_names;
    ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
    const auto t = [&]()
    {
        std::vector<std::string> filters;
        auto sel = pfd::open_file("Load las files", "C:\\", filters, true).result();
        for (int i = 0; i < sel.size(); i++)
        {
            input_file_names.push_back(sel[i]);
            // std::cout << "las file: '" << input_file_name << "'" << std::endl;
        }
    };
    std::thread t1(t);
    t1.join();

    std::sort(std::begin(input_file_names), std::end(input_file_names));

    std::vector<std::string> csv_files;
    std::vector<std::string> laz_files;
    std::for_each(std::begin(input_file_names), std::end(input_file_names), [&](const std::string &fileName)
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
    for (const auto &fn : csv_files)
    {
        std::cout << fn << std::endl;
    }

    std::cout << "loading imu" << std::endl;
    std::vector<std::tuple<std::pair<double, double>, FusionVector, FusionVector>> imu_data;

    std::for_each(std::begin(csv_files), std::end(csv_files), [&imu_data](const std::string &fn)
                  {
                    auto imu = load_imu(fn.c_str(), 0);
                    std::cout << fn << std::endl;
                    imu_data.insert(std::end(imu_data), std::begin(imu), std::end(imu)); });

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    if (fusionConventionNwu)
    {
        ahrs.settings.convention = FusionConventionNwu;
    }
    if (fusionConventionEnu)
    {
        ahrs.settings.convention = FusionConventionEnu;
    }
    if (fusionConventionNed)
    {
        ahrs.settings.convention = FusionConventionNed;
    }

    std::map<double, Eigen::Matrix4d> trajectory;

    int counter = 1;
    for (const auto &[timestamp_pair, gyr, acc] : imu_data)
    {
        const FusionVector gyroscope = {static_cast<float>(gyr.axis.x * 180.0 / M_PI), static_cast<float>(gyr.axis.y * 180.0 / M_PI), static_cast<float>(gyr.axis.z * 180.0 / M_PI)};
        const FusionVector accelerometer = {acc.axis.x, acc.axis.y, acc.axis.z};

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);

        Eigen::Quaterniond d{quat.element.w, quat.element.x, quat.element.y, quat.element.z};
        Eigen::Affine3d t{Eigen::Matrix4d::Identity()};
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
    for (const auto &fn : laz_files)
    {
        std::cout << fn << std::endl;
    }

    // for (const auto &fn : laz_files)
    //{
    // std::vector<Point3Di> points = load_point_cloud(fn);
    // std::cout << "points.cloud(): " << points.size() << std::endl;
    //}

    /*int current_file_index = 0;
    int current_point_index = 0;

    std::vector<Point3Di> points;

    bool do_not_stop = true;
    while (do_not_stop){
        bool do_not_stop_internal_loop = true;
        while (do_not_stop_internal_loop){
            bool collected_all_points = get_next_batch_of_points(
                point_count_threshold, laz_files, current_file_index, int &current_point_index_offset,
                                                                 std::vector<Point3Di> &points)
        }
    }*/
    // get_next_batch_of_points(point_count_threshold, laz_files, current_file_index, current_point_index, points);

    std::vector<Point3Di> prev_points;
    std::vector<std::vector<Point3Di>> all_points;
    std::vector<std::vector<Point3Di>> tmp_points = get_batches_of_points(laz_files[0], point_count_threshold, prev_points);

    for (size_t i = 0; i < tmp_points.size() - 1; i++)
    {
        all_points.push_back(tmp_points[i]);
    }

    for (int i = 1; i < laz_files.size(); i++)
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
    for (const auto &t : trajectory)
    {
        timestamps.push_back(t.first);
        Eigen::Affine3d m;
        m.matrix() = t.second;
        poses.push_back(m);
    }

    for (int i = 0; i < all_points.size(); i++)
    {
        std::cout << all_points[i].size() << std::endl;
        std::string fn1 = "C:/data/tmp/" + std::to_string(i) + "_best.laz";
        std::string fn2 = "C:/data/tmp/" + std::to_string(i) + "_original.laz";

        find_best_stretch(all_points[i], timestamps, poses, fn1, fn2);
    }
}

LaserBeam GetLaserBeam(int x, int y)
{
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;
    GLdouble posXnear, posYnear, posZnear;
    GLdouble posXfar, posYfar, posZfar;

    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);

    winX = (float)x;
    winY = (float)viewport[3] - (float)y;

    LaserBeam laser_beam;
    gluUnProject(winX, winY, 0, modelview, projection, viewport, &posXnear, &posYnear, &posZnear);
    gluUnProject(winX, winY, -1000, modelview, projection, viewport, &posXfar, &posYfar, &posZfar);

    laser_beam.position.x() = posXnear;
    laser_beam.position.y() = posYnear;
    laser_beam.position.z() = posZnear;

    laser_beam.direction.x() = posXfar - posXnear;
    laser_beam.direction.y() = posYfar - posYnear;
    laser_beam.direction.z() = posZfar - posZnear;

    return laser_beam;
}

Eigen::Vector3d GLWidgetGetOGLPos(int x, int y, const ObservationPicking &observation_picking)
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

float distanceToPlane(const RegistrationPlaneFeature::Plane &plane, const Eigen::Vector3d &p)
{
    return (plane.a * p.x() + plane.b * p.y() + plane.c * p.z() + plane.d);
}

Eigen::Vector3d rayIntersection(const LaserBeam &laser_beam, const RegistrationPlaneFeature::Plane &plane)
{
    float TOLERANCE = 0.0001;
    Eigen::Vector3d out_point;
    out_point.x() = laser_beam.position.x();
    out_point.y() = laser_beam.position.y();
    out_point.z() = laser_beam.position.z();

    float a = plane.a * laser_beam.direction.x() + plane.b * laser_beam.direction.y() + plane.c * laser_beam.direction.z();

    if (a > -TOLERANCE && a < TOLERANCE)
    {
        return out_point;
    }

    float distance = distanceToPlane(plane, out_point);

    out_point.x() = laser_beam.position.x() - laser_beam.direction.x() * (distance / a);
    out_point.y() = laser_beam.position.y() - laser_beam.direction.y() * (distance / a);
    out_point.z() = laser_beam.position.z() - laser_beam.direction.z() * (distance / a);

    return out_point;
}

bool exportLaz(const std::string &filename,
               const std::vector<Eigen::Vector3d> &pointcloud,
               const std::vector<unsigned short> &intensity, double offset_x, double offset_y, double offset_alt)
{

    constexpr float scale = 0.0001f; // one tenth of milimeter
    // find max
    Eigen::Vector3d _max(-1000000000.0, -1000000000.0, -1000000000.0);
    Eigen::Vector3d _min(1000000000.0, 1000000000.0, 1000000000.0);

    for (auto &p : pointcloud)
    {
        if (p.x() < _min.x())
        {
            _min.x() = p.x();
        }
        if (p.y() < _min.y())
        {
            _min.y() = p.y();
        }
        if (p.z() < _min.z())
        {
            _min.z() = p.z();
        }

        if (p.x() > _max.x())
        {
            _max.x() = p.x();
        }
        if (p.y() > _max.y())
        {
            _max.y() = p.y();
        }
        if (p.z() > _max.z())
        {
            _max.z() = p.z();
        }
    }

    // create the writer
    laszip_POINTER laszip_writer;
    if (laszip_create(&laszip_writer))
    {
        fprintf(stderr, "DLL ERROR: creating laszip writer\n");
        return false;
    }

    // get a pointer to the header of the writer so we can populate it

    laszip_header *header;

    if (laszip_get_header_pointer(laszip_writer, &header))
    {
        fprintf(stderr, "DLL ERROR: getting header pointer from laszip writer\n");
        return false;
    }

    // populate the header

    header->file_source_ID = 4711;
    header->global_encoding = (1 << 0); // see LAS specification for details
    header->version_major = 1;
    header->version_minor = 2;
    //    header->file_creation_day = 120;
    //    header->file_creation_year = 2013;
    header->point_data_format = 1;
    header->point_data_record_length = 0;
    header->number_of_point_records = pointcloud.size();
    header->number_of_points_by_return[0] = pointcloud.size();
    header->number_of_points_by_return[1] = 0;
    header->point_data_record_length = 28;
    header->x_scale_factor = scale;
    header->y_scale_factor = scale;
    header->z_scale_factor = scale;

    header->max_x = _max.x() + offset_x;
    header->min_x = _min.x() + offset_x;
    header->max_y = _max.y() + offset_y;
    header->min_y = _min.y() + offset_y;
    header->max_z = _max.z() + offset_alt;
    header->min_z = _min.z() + offset_alt;

    header->x_offset = offset_x;
    header->y_offset = offset_y;
    header->z_offset = offset_alt;

    // optional: use the bounding box and the scale factor to create a "good" offset
    // open the writer
    laszip_BOOL compress = (strstr(filename.c_str(), ".laz") != 0);

    if (laszip_open_writer(laszip_writer, filename.c_str(), compress))
    {
        fprintf(stderr, "DLL ERROR: opening laszip writer for '%s'\n", filename.c_str());
        return false;
    }

    fprintf(stderr, "writing file '%s' %scompressed\n", filename.c_str(), (compress ? "" : "un"));

    // get a pointer to the point of the writer that we will populate and write

    laszip_point *point;
    if (laszip_get_point_pointer(laszip_writer, &point))
    {
        fprintf(stderr, "DLL ERROR: getting point pointer from laszip writer\n");
        return false;
    }

    laszip_I64 p_count = 0;
    laszip_F64 coordinates[3];

    for (int i = 0; i < pointcloud.size(); i++)
    {
        point->intensity = intensity[i];

        const auto &p = pointcloud[i];
        p_count++;
        coordinates[0] = p.x() + offset_x;
        coordinates[1] = p.y() + offset_y;
        coordinates[2] = p.z() + offset_alt;
        if (laszip_set_coordinates(laszip_writer, coordinates))
        {
            fprintf(stderr, "DLL ERROR: setting coordinates for point %I64d\n", p_count);
            return false;
        }

        // p.SetIntensity(pp.intensity);

        // if (i < intensity.size()) {
        //     point->intensity = intensity[i];
        // }
        // laszip_set_point

        if (laszip_write_point(laszip_writer))
        {
            fprintf(stderr, "DLL ERROR: writing point %I64d\n", p_count);
            return false;
        }
    }

    if (laszip_get_point_count(laszip_writer, &p_count))
    {
        fprintf(stderr, "DLL ERROR: getting point count\n");
        return false;
    }

    fprintf(stderr, "successfully written %I64d points\n", p_count);

    // close the writer

    if (laszip_close_writer(laszip_writer))
    {
        fprintf(stderr, "DLL ERROR: closing laszip writer\n");
        return false;
    }

    // destroy the writer

    if (laszip_destroy(laszip_writer))
    {
        fprintf(stderr, "DLL ERROR: destroying laszip writer\n");
        return false;
    }

    std::cout << "exportLaz DONE" << std::endl;

    return true;
}
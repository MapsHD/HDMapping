#include "lidar_odometry.h"
#include <mutex>
#include <toml.hpp>

namespace fs = std::filesystem;

bool load_data(std::vector<std::string> &input_file_names, LidarOdometryParams &params, std::vector<std::vector<Point3Di>> &pointsPerFile, Imu &imu_data)
{
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
    for (int i = 0; i < laz_files.size(); i++)
    {
        fs::path lf(laz_files[i]);
        std::string lfs = lf.filename().stem().string().substr(lf.filename().stem().string().size() - 4);

        bool found = false;
        for (int j = 0; j < csv_files.size(); j++)
        {
            fs::path cf(csv_files[j]);
            std::string cfs = cf.filename().stem().string().substr(cf.filename().stem().string().size() - 4);
            if (lfs.compare(cfs) == 0)
            {
                found = true;
                break;
            }
        }

        if (!found)
        {
            std::cout << "there is no IMU file for: " << laz_files[i] << std::endl;
        }
    }
    for (int i = 0; i < csv_files.size(); i++)
    {
        fs::path lf(csv_files[i]);
        std::string lfs = lf.filename().stem().string().substr(lf.filename().stem().string().size() - 4);

        bool found = false;
        for (int j = 0; j < laz_files.size(); j++)
        {
            fs::path cf(laz_files[j]);
            std::string cfs = cf.filename().stem().string().substr(cf.filename().stem().string().size() - 4);
            if (lfs.compare(cfs) == 0)
            {
                found = true;
                break;
            }
        }

        if (!found)
        {
            std::cout << "there is no LAZ file for: " << csv_files[i] << std::endl;
        }
    }
    std::string working_directory = "";
    std::string imuSnToUse;

    int sn_size = sn_files.size();
    if (sn_size == 0)
    {
        sn_size = laz_files.size();
    }

    if (input_file_names.size() > 0 && laz_files.size() == csv_files.size() && laz_files.size() == sn_size)
    {
        working_directory = fs::path(input_file_names[0]).parent_path().string();

        // check if folder exists!
        if (!fs::exists(working_directory))
        {
            std::cout << "folder '" << working_directory << "' does not exist" << std::endl;

            std::string message_info = "folder '" + working_directory + "' does not exist --> PLEASE REMOVE e.g. POLISH LETTERS from path. !!!PROGRAM WILL SHUT DOWN AFTER THIS MESSAGE!!!";
            return false;
        }

        const auto calibrationFile = (fs::path(working_directory) / "calibration.json").string();
        const auto preloadedCalibration = MLvxCalib::GetCalibrationFromFile(calibrationFile);
        imuSnToUse = MLvxCalib::GetImuSnToUse(calibrationFile);
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
            std::cout << "IGNORE THIS MESSAGE IF YOU HAVE ONLY 1 LIDAR" << std::endl;

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
            std::cout << "trying creating folder: '" << wdp << "'" << std::endl;
            fs::create_directory(wdp);
            std::cout << "folder created" << std::endl;
        }

        params.working_directory_preview = wdp.string();

        for (size_t i = 0; i < input_file_names.size(); i++)
        {
            std::cout << input_file_names[i] << std::endl;
        }
        std::cout << "loading imu" << std::endl;
        for (size_t fileNo = 0; fileNo < csv_files.size(); fileNo++)
        {
            const std::string &imufn = csv_files.at(fileNo);
            const std::string snFn = (fileNo >= sn_files.size()) ? ("") : (sn_files.at(fileNo));
            const auto idToSn = MLvxCalib::GetIdToSnMapping(snFn);
            // GetId of Imu to use
            int imuNumberToUse = MLvxCalib::GetImuIdToUse(idToSn, imuSnToUse);
            std::cout << "imuNumberToUse  " << imuNumberToUse << " at '" << imufn << "'" << std::endl;
            auto imu = load_imu(imufn.c_str(), imuNumberToUse);
            std::cout << imufn << " with mapping " << snFn << std::endl;
            imu_data.insert(std::end(imu_data), std::begin(imu), std::end(imu));
            bool hasError = false;

            if (!preloadedCalibration.empty())
            {
                for (const auto &[id, sn] : idToSn)
                {
                    if (preloadedCalibration.find(sn) == preloadedCalibration.end())
                    {
                        std::cerr << "WRONG CALIBRATION FILE! THE SERIAL NUMBER SHOULD BE " << sn << "!!!\n";
                        hasError = true;
                    }
                }

                if (!hasError && preloadedCalibration.find(imuSnToUse) == preloadedCalibration.end())
                {
                    std::cerr << "MISSING CALIBRATION FOR imuSnToUse: " << imuSnToUse << "!!!\n";
                    std::cerr << "Available serial numbers in calibration file are:\n";
                    for (const auto &[snKey, _] : preloadedCalibration)
                    {
                        std::cerr << "  - " << snKey << "\n";
                    }
                    hasError = true;
                }

                if (hasError)
                {
                    std::cerr << "Press ENTER to exit...\n";
                    std::cin.get();
                    std::exit(EXIT_FAILURE);
                }
            }
        }

        std::sort(imu_data.begin(), imu_data.end(),
                  [](const std::tuple<std::pair<double, double>, FusionVector, FusionVector> &a, const std::tuple<std::pair<double, double>, FusionVector, FusionVector> &b)
                  {
                      return std::get<0>(a).first < std::get<0>(b).first;
                  });

        std::cout << "loading points" << std::endl;
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
                           auto data = load_point_cloud(fn.c_str(), true, params.filter_threshold_xy_inner, params.filter_threshold_xy_outer, calibration);

                           std::sort(data.begin(), data.end(), [](const Point3Di &a, const Point3Di &b)
                                     { return a.timestamp < b.timestamp; });

                           if ((fn == laz_files.front()) && (params.save_calibration_validation))
                           {
                               fs::path calibrationValidtationFile = wdp / "calibrationValidation.asc";
                               std::ofstream testPointcloud{calibrationValidtationFile.c_str()};
                               int row_index = 0;
                               for (const auto &p : data)
                               {
                                   if (row_index++ >= params.calibration_validation_points)
                                   {
                                       break;
                                   }
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

        if (pointsPerFile.size() > 0)
        {
            pointsPerFile[0].clear();
        }
    }
    else
    {
        std::cout << "please select files correctly" << std::endl;
        std::cout << "input_file_names.size(): " << input_file_names.size() << std::endl;
        std::cout << "laz_files.size(): " << laz_files.size() << std::endl;
        std::cout << "csv_files.size(): " << csv_files.size() << std::endl;
        std::cout << "sn_files.size(): " << sn_files.size() << std::endl;

        std::cout
            << "condition: input_file_names.size() > 0 && laz_files.size() == csv_files.size() && laz_files.size() == sn_files.size() NOT SATISFIED!!!" << std::endl;
        return false;
    }
    int number_of_points = 0;
    for (const auto &pp : pointsPerFile)
    {
        number_of_points += pp.size();
    }
    std::cout << "number of points: " << number_of_points << std::endl;
    std::cout << "start transforming points" << std::endl;

    std::cout << "point cloud file names" << std::endl;
    for (const auto &fn : laz_files)
    {
        std::cout << fn << std::endl;
    }
    return true;
}

void calculate_trajectory(
    Trajectory &trajectory, Imu &imu_data, bool fusionConventionNwu, bool fusionConventionEnu, bool fusionConventionNed, double ahrs_gain)
{
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
    ahrs.settings.gain = ahrs_gain;
    int counter = 1;

    double provious_time_stamp = 0.0;

    static bool first = true;
    static double last_ts;

    for (const auto &[timestamp_pair, gyr, acc] : imu_data)
    {
        const FusionVector gyroscope = {static_cast<float>(gyr.axis.x * 180.0 / M_PI), static_cast<float>(gyr.axis.y * 180.0 / M_PI), static_cast<float>(gyr.axis.z * 180.0 / M_PI)};
        const FusionVector accelerometer = {acc.axis.x, acc.axis.y, acc.axis.z};

        /*if (provious_time_stamp == 0.0)
        {
            double SAMPLE_PERIOD = (1.0 / 200.0);
            FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
        }
        else
        {
            double sp = timestamp_pair.first - provious_time_stamp;
            // std::cout << "sp: " << sp << std::endl;
            if (sp > 0.1)
            {
                sp = 0.1;
            }
            FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, sp);
        }*/

        if (first)
        {
            FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, 1.0 / 200.0);
            first = false;
            // last_ts = timestamp_pair.first;
        }
        else
        {
            double curr_ts = timestamp_pair.first;

            double ts_diff = curr_ts - last_ts;

            FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, ts_diff);

            /*if (ts_diff < 0)
            {
                std::cout << "WARNING!!!!" << std::endl;
                std::cout << "WARNING!!!!" << std::endl;
                std::cout << "WARNING!!!!" << std::endl;
                std::cout << "WARNING!!!!" << std::endl;
                std::cout << "WARNING!!!!" << std::endl;
                std::cout << "WARNING!!!!" << std::endl;
            }

            if (ts_diff < 0.01)
            {
                FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, ts_diff);
            }
            else
            {
                FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, 1.0/200.0);
            }*/
        }

        last_ts = timestamp_pair.first;

        FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);

        Eigen::Quaterniond d{quat.element.w, quat.element.x, quat.element.y, quat.element.z};
        Eigen::Affine3d t{Eigen::Matrix4d::Identity()};
        t.rotate(d);

        trajectory[timestamp_pair.first] = std::pair(t.matrix(), timestamp_pair.second);
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        counter++;
        if (counter % 100 == 0)
        {
            printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f [%d of %d]\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw, counter++, (int)imu_data.size());
        }
        provious_time_stamp = timestamp_pair.first;
    }
}

bool compute_step_1(
    std::vector<std::vector<Point3Di>> &pointsPerFile, LidarOdometryParams &params, Trajectory &trajectory, std::vector<WorkerData> &worker_data)
{
    int number_of_initial_points = 0;
    double timestamp_begin = 0.0;
    for (const auto &pp : pointsPerFile)
    {
        // number_of_points += pp.size();
        for (const auto &p : pp)
        {
            number_of_initial_points++;
            params.initial_points.push_back(p);
            if (number_of_initial_points > params.threshold_initial_points)
            {
                timestamp_begin = p.timestamp;
                break;
            }
        }
        if (number_of_initial_points > params.threshold_initial_points)
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
        return false;
    }

    int threshold = params.threshold_nr_poses;

    int index_begin = 0;
    const int n_iter = std::floor(poses.size() / threshold);
    worker_data.reserve(n_iter);
    for (int i = 0; i < n_iter; i++)
    {
        if (i % 50 == 0)
        {
            std::cout << "preparing data " << i + 1 << " of " << n_iter << std::endl;
        }
        WorkerData wd;
        wd.intermediate_trajectory.reserve(threshold);
        wd.intermediate_trajectory_motion_model.reserve(threshold);
        wd.intermediate_trajectory_timestamps.reserve(threshold);
        wd.imu_om_fi_ka.reserve(threshold);
        for (int ii = 0; ii < threshold; ii++)
        {
            int idx = i * threshold + ii;
            wd.intermediate_trajectory.emplace_back(poses[idx]);
            wd.intermediate_trajectory_motion_model.emplace_back(poses[idx]);
            wd.intermediate_trajectory_timestamps.emplace_back(timestamps[idx]);
            TaitBryanPose tb = pose_tait_bryan_from_affine_matrix(poses[idx]);
            // wd.imu_roll_pitch.emplace_back(tb.om, tb.fi);
            wd.imu_om_fi_ka.emplace_back(tb.om, tb.fi, tb.ka);
        }
        std::vector<Point3Di> points;
        bool found = false;
        auto lower = pointsPerFile[0].begin();
        for (int index = index_begin; index < pointsPerFile.size(); index++)
        {
            auto lower = std::lower_bound(
                pointsPerFile[index].begin(), pointsPerFile[index].end(),
                wd.intermediate_trajectory_timestamps[0].first,
                [](const Point3Di &point, double timestamp)
                {
                    return point.timestamp < timestamp;
                });
            auto upper = std::lower_bound(
                pointsPerFile[index].begin(), pointsPerFile[index].end(),
                wd.intermediate_trajectory_timestamps[wd.intermediate_trajectory_timestamps.size() - 1].first,
                [](const Point3Di &point, double timestamp)
                {
                    return point.timestamp < timestamp;
                });
            auto tmp = std::distance(lower, upper);
            points.reserve(points.size() + std::distance(lower, upper));
            if (lower != upper)
            {
                points.insert(points.end(), std::make_move_iterator(lower), std::make_move_iterator(upper));
                found = true;
            }
        }

        wd.original_points = points;

        // correct points timestamps
        if (wd.intermediate_trajectory_timestamps.size() > 2)
        {

            double ts_begin = wd.intermediate_trajectory_timestamps[0].first;
            double ts_step = (wd.intermediate_trajectory_timestamps[wd.intermediate_trajectory_timestamps.size() - 1].first -
                              wd.intermediate_trajectory_timestamps[0].first) /
                             wd.original_points.size();

            // std::cout << "ts_begin " << ts_begin << std::endl;
            // std::cout << "ts_step " << ts_step << std::endl;
            // std::cout << "ts_end " << wd.intermediate_trajectory_timestamps[wd.intermediate_trajectory_timestamps.size() - 1].first << std::endl;

            for (int pp = 0; pp < wd.original_points.size(); pp++)
            {
                wd.original_points[pp].timestamp = ts_begin + pp * ts_step;
            }
        }
        //////////////////////////////////////////

        for (unsigned long long int k = 0; k < wd.original_points.size(); k++)
        {
            Point3Di &p = wd.original_points[k];
            auto lower = std::lower_bound(wd.intermediate_trajectory_timestamps.begin(), wd.intermediate_trajectory_timestamps.end(), p.timestamp,
                                          [](std::pair<double, double> lhs, double rhs) -> bool
                                          { return lhs.first < rhs; });
            // p.index_pose = std::distance(wd.intermediate_trajectory_timestamps.begin(), lower);

            int index_pose = std::distance(wd.intermediate_trajectory_timestamps.begin(), lower) - 1;
            p.index_pose = index_pose;

            if (index_pose >= 0 && index_pose < wd.intermediate_trajectory_timestamps.size())
            {
                if (fabs(p.timestamp - wd.intermediate_trajectory_timestamps[index_pose].first) > 0.01)
                {
                    p.index_pose = -1;
                }
            }
            else
            {
                p.index_pose = -1;
            }
        }

        std::vector<Point3Di> filtered_points;
        for (unsigned long long int k = 0; k < wd.original_points.size(); k++)
        {
            if (wd.original_points[k].index_pose != -1)
            {
                filtered_points.push_back(wd.original_points[k]);
            }
        }
        wd.original_points = filtered_points;

        if (params.decimation > 0.0 && wd.original_points.size() > 1000)
        {
            wd.intermediate_points = decimate(wd.original_points, params.decimation, params.decimation, params.decimation);
        }

        // std::cout << "number of points: " << wd.original_points.size() << std::endl;
        if (wd.original_points.size() > 1000)
        {
            worker_data.push_back(wd);
        }
    }
    params.m_g = worker_data[0].intermediate_trajectory[0];
    return true;
}

void run_consistency(std::vector<WorkerData> &worker_data, LidarOdometryParams &params)
{
    std::cout << "Point cloud consistency and trajectory smoothness START" << std::endl;
    for (int i = 0; i < params.num_constistency_iter; i++)
    {
        std::cout << "Iteration " << i + 1 << " of " << params.num_constistency_iter << std::endl;
        for (int ii = 0; ii < worker_data.size(); ii++)
        {
            worker_data[ii].intermediate_trajectory_motion_model = worker_data[ii].intermediate_trajectory;
        }
        if (!params.use_mutliple_gaussian)
        {
            Consistency(worker_data, params);
        }
        else
        {
            Consistency2(worker_data, params);
        }
    }
    std::cout << "Point cloud consistency and trajectory smoothness FINISHED" << std::endl;
}

void save_result(std::vector<WorkerData> &worker_data, LidarOdometryParams &params, fs::path outwd, double elapsed_time_s)
{
    std::filesystem::create_directory(outwd);
    // concatenate data
    std::vector<WorkerData> worker_data_concatenated;
    WorkerData wd;
    int counter = 0;
    int pose_offset = 0;
    std::vector<int> point_sizes_per_chunk;
    for (int i = 0; i < worker_data.size(); i++)
    {
        if (i % 1000 == 0)
        {
            printf("processing worker_data [%d] of %d \n", i + 1, (int)worker_data.size());
        }
        auto tmp_data = worker_data[i].original_points;
        point_sizes_per_chunk.push_back(tmp_data.size());
        /*// filter data
        std::vector<Point3Di> filtered_local_point_cloud;
        for (auto &t : tmp_data)
        {
            auto pp = worker_data[i].intermediate_trajectory[t.index_pose].inverse() * t.point;
            if(pp.norm() > threshould_output_filter){
                filtered_local_point_cloud.push_back(t);
            }
        }
        tmp_data = filtered_local_point_cloud;*/
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

        wd.imu_om_fi_ka.insert(std::end(wd.imu_om_fi_ka), std::begin(worker_data[i].imu_om_fi_ka), std::end(worker_data[i].imu_om_fi_ka));
        pose_offset += worker_data[i].intermediate_trajectory.size();

        counter++;
        if (counter > 50)
        {
            worker_data_concatenated.push_back(wd);
            wd.intermediate_trajectory.clear();
            wd.intermediate_trajectory_timestamps.clear();
            wd.original_points.clear();
            wd.imu_om_fi_ka.clear();

            counter = 0;
            pose_offset = 0;
        }
    }
    
    if (counter > params.min_counter)
    {
        worker_data_concatenated.push_back(wd);
    }

    fs::path point_sizes_path = outwd / "point_sizes_per_chunk.json";
    nlohmann::json j_point_sizes = point_sizes_per_chunk;
    std::ofstream out_point_sizes(point_sizes_path);
    if (!out_point_sizes)
    {
        std::cerr << "Failed to open " << point_sizes_path << " for writing point sizes per chunk.\n";
    }
    else
    {
        out_point_sizes << j_point_sizes.dump(2);
        out_point_sizes.close();
    }

    std::vector<Eigen::Affine3d> m_poses;
    std::vector<std::string> file_names;
    std::vector<std::vector<int>> index_poses;
    for (int i = 0; i < worker_data_concatenated.size(); i++)
    {
        std::cout << "------------------------" << std::endl;
        fs::path path(outwd);
        std::string filename = ("scan_lio_" + std::to_string(i) + ".laz");
        path /= filename;
        std::cout << "saving to: " << path << std::endl;
        std::vector<int> index_poses_i;
        saveLaz(path.string(), worker_data_concatenated[i], params.threshould_output_filter, &index_poses_i);
        index_poses.push_back(index_poses_i); 
        m_poses.push_back(worker_data_concatenated[i].intermediate_trajectory[0]);
        file_names.push_back(filename);

        // save trajectory
        std::string trajectory_filename = ("trajectory_lio_" + std::to_string(i) + ".csv");
        fs::path pathtrj(outwd);
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

        outfile << "timestamp_nanoseconds pose00 pose01 pose02 pose03 pose10 pose11 pose12 pose13 pose20 pose21 pose22 pose23 timestampUnix_nanoseconds om_rad fi_rad ka_rad" << std::endl;
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
                << std::setprecision(20) << worker_data_concatenated[i].intermediate_trajectory_timestamps[j].second * 1e9 << " "
                << worker_data_concatenated[i].imu_om_fi_ka[j].x() << " "
                << worker_data_concatenated[i].imu_om_fi_ka[j].y() << " "
                << worker_data_concatenated[i].imu_om_fi_ka[j].z() << " "
                << std::endl;
        }
        outfile.close();
        //
    }
    fs::path path(outwd);
    path /= "lio_initial_poses.reg";
    save_poses(path.string(), m_poses, file_names);
    fs::path path2(outwd);
    path2 /= "poses.reg";
    save_poses(path2.string(), m_poses, file_names);

    fs::path index_poses_path = outwd / "index_poses.json";
    nlohmann::json j_index_poses = index_poses;
    std::ofstream out_index(index_poses_path);
    if (!out_index)
    {
        std::cerr << "Failed to open " << index_poses_path << " for writing index poses.\n";
    }
    else
    {
        out_index << j_index_poses.dump(2);
        out_index.close();
    }

    fs::path path3(outwd);
    path3 /= "session.json";

    // save session file
    std::cout << "saving file: '" << path3 << "'" << std::endl;

    nlohmann::json jj;
    nlohmann::json j;

    j["offset_x"] = 0.0;
    j["offset_y"] = 0.0;
    j["offset_z"] = 0.0;
    j["folder_name"] = outwd;
    j["out_folder_name"] = outwd;
    j["poses_file_name"] = path2.string();
    j["initial_poses_file_name"] = path.string();
    j["out_poses_file_name"] = path2.string();
    j["lidar_odometry_version"] = HDMAPPING_VERSION_STRING;
    j["length of trajectory[m]"] = params.total_length_of_calculated_trajectory;
    j["elapsed time seconds"] = elapsed_time_s;
    j["index_poses_path"] = index_poses_path.string();
    j["point_sizes_path"] = point_sizes_path.string();
    j["decimation"] = params.decimation;
    j["threshold_nr_poses"] = params.threshold_nr_poses;

    jj["Session Settings"] = j;

    nlohmann::json jlaz_file_names;
    for (int i = 0; i < worker_data_concatenated.size(); i++)
    {
        fs::path path(outwd);
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
}

void filter_reference_buckets(LidarOdometryParams &params)
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

void save_all_to_las(
    std::vector<WorkerData> &worker_data, LidarOdometryParams &params, std::string output_file_name, Session &session,
    bool export_selected, bool filter_on_export, bool apply_pose, bool add_to_pc_container)
{
    std::vector<Eigen::Vector3d> pointcloud;
    std::vector<unsigned short> intensity;
    std::vector<double> timestamps;
    Eigen::Affine3d pose;
    for (int i = 0; i < worker_data.size(); i++)
    {
        if (!export_selected || worker_data[i].show)
        {
            for (const auto &p : worker_data[i].intermediate_points)
            {
                if (!filter_on_export || (p.point.norm() > params.threshould_output_filter))
                {
                    Eigen::Vector3d pt = p.point;
                    if (apply_pose)
                    {
                        pt = worker_data[i].intermediate_trajectory[p.index_pose] * p.point;
                    }
                    pointcloud.push_back(pt);
                    intensity.push_back(p.intensity);
                    timestamps.push_back(p.timestamp);
                    pose = worker_data[i].intermediate_trajectory[p.index_pose];
                }
            }
        }
    }
    if (!exportLaz(output_file_name, pointcloud, intensity, timestamps, 0, 0, 0))
    {
        std::cout << "problem with saving file: " << output_file_name << std::endl;
    }
    else if (add_to_pc_container)
    {
        PointCloud pc;
        pc.file_name = output_file_name;
        pc.m_pose = pose;
        session.point_clouds_container.point_clouds.push_back(pc);
    }
}

void save_trajectory_to_ascii(std::vector<WorkerData> &worker_data, std::string output_file_name)
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

void load_reference_point_clouds(std::vector<std::string> input_file_names, LidarOdometryParams &params)
{
    params.reference_buckets.clear();
    params.reference_points.clear();

    for (size_t i = 0; i < input_file_names.size(); i++)
    {
        std::cout << "loading reference point cloud from: " << input_file_names[i] << std::endl;
        auto pp = load_point_cloud(input_file_names[i].c_str(), false, 0, 10000, {});
        std::cout << "loaded " << pp.size() << " reference points" << std::endl;
        params.reference_points.insert(std::end(params.reference_points), std::begin(pp), std::end(pp));
    }

    update_rgd(params.in_out_params_indoor, params.reference_buckets, params.reference_points);
    params.buckets_indoor = params.reference_buckets;
}

std::string save_results_automatic(LidarOdometryParams &params, std::vector<WorkerData> &worker_data, std::string working_directory, double elapsed_seconds)
{
    int result = get_next_result_id(working_directory);
    fs::path outwd = working_directory / fs::path("lidar_odometry_result_" + std::to_string(result));
    save_result(worker_data, params, outwd, elapsed_seconds);
    return outwd.string();
}

std::vector<WorkerData> run_lidar_odometry(std::string input_dir, LidarOdometryParams &params)
{
    Session session;
    std::vector<WorkerData> worker_data;
    std::vector<std::string> input_file_names;
    for (const auto &entry : std::filesystem::directory_iterator(input_dir))
    {
        if (fs::is_regular_file(entry))
        {
            input_file_names.push_back(entry.path().string());
        }
    }
    std::vector<std::vector<Point3Di>> pointsPerFile;
    Imu imu_data;
    if (!load_data(input_file_names, params, pointsPerFile, imu_data))
    {
        std::cout << "Calculation failed at data loading, exiting." << std::endl;
        return worker_data;
    }
    Trajectory trajectory;
    calculate_trajectory(trajectory, imu_data, params.fusionConventionNwu, params.fusionConventionEnu, params.fusionConventionNed, params.ahrs_gain);
    
    if (!compute_step_1(pointsPerFile, params, trajectory, worker_data))
    {
        std::cout << "Calculation failed at step 1 of lidar odometry, exiting." << std::endl;
        return worker_data;
    }
    double ts_failure = 0.0;
    
    std::atomic<float> loProgress;
    
    if (!compute_step_2(worker_data, params, ts_failure, loProgress))
    {
        std::cout << "Calculation failed at step 2 of lidar odometry, exiting." << std::endl;
        return worker_data;
    }
    return worker_data;
}

bool SaveParametersToTomlFile(const std::string &filepath, const LidarOdometryParams &params)
{
    try
    {
        std::ofstream file(filepath);
        if (!file.is_open())
        {
            std::cerr << "Cannot open file for writing!" << std::endl;
            return false;
        }

        file << "# This is toml file with lidar odometry params\n";

        file << "[version]\n";
        file << "version = " << HDMAPPING_VERSION_MINOR << "\n\n";

        file << "[nr_iter]\n";
        file << "value = " << params.nr_iter << "\n\n";

        file << std::fixed << std::setprecision(7);

        file << "[decimation] # it remains single point per bucket (dim_x_m, dim_y_m, dim_z_m) = (value, value, value)\n";
        file << "value = " << params.decimation << "\n\n";

        file << "[in_out_params_indoor]\n";
        file << "resolution_X = " << params.in_out_params_indoor.resolution_X << "\n";
        file << "resolution_Y = " << params.in_out_params_indoor.resolution_Y << "\n";
        file << "resolution_Z = " << params.in_out_params_indoor.resolution_Z << "\n\n";

        file << "[in_out_params_outdoor]\n";
        file << "resolution_X = " << params.in_out_params_outdoor.resolution_X << "\n";
        file << "resolution_Y = " << params.in_out_params_outdoor.resolution_Y << "\n";
        file << "resolution_Z = " << params.in_out_params_outdoor.resolution_Z << "\n\n";

        file << "[filter_threshold_xy_inner] # remove all local points from cylider R < value\n";
        file << "value = " << params.filter_threshold_xy_inner << "\n\n";

        file << "[filter_threshold_xy_outer] # remove all local points from cylider R > value\n";
        file << "value = " << params.filter_threshold_xy_outer << "\n\n";

        file << "[threshould_output_filter] # remove all local points from cylider R < value (remove shadow points from operator posture)\n";
        file << "value = " << params.threshould_output_filter << "\n\n";

        file << "[distance_bucket]\n";
        file << "value = " << params.distance_bucket << "\n\n";

        file << "[polar_angle_deg]\n";
        file << "value = " << params.polar_angle_deg << "\n\n";

        file << "[azimutal_angle_deg]\n";
        file << "value = " << params.azimutal_angle_deg << "\n\n";

        file << std::setprecision(0);
        file << "[robust_and_accurate_lidar_odometry_iterations]\n";
        file << "value = " << params.robust_and_accurate_lidar_odometry_iterations << "\n\n";

        file << std::fixed << std::setprecision(7);
        file << "[max_distance_lidar]\n";
        file << "value = " << params.max_distance_lidar << "\n\n";

        file << std::boolalpha;
        file << "[use_robust_and_accurate_lidar_odometry] # this is experimental --> not yet tested well\n";
        file << "value = " << params.use_robust_and_accurate_lidar_odometry << "\n\n";

        file << "[fusionConventionNed]\n";
        file << "Ned = " << params.fusionConventionNed << "\n\n";

        file << "[fusionConventionNwu]\n";
        file << "Nwu = " << params.fusionConventionNwu << "\n\n";

        file << "[fusionConventionEnu]\n";
        file << "Enu = " << params.fusionConventionEnu << "\n\n";

        file << "[use_motion_from_previous_step]\n";
        file << "value = " << params.use_motion_from_previous_step << "\n\n";

        file << "[useMultithread]\n";
        file << "value = " << params.useMultithread << "\n\n";

        file << "[sliding_window_trajectory_length_threshold] # local map size measured as trajectory lenght\n";
        file << "value = " << params.sliding_window_trajectory_length_threshold << "\n\n";

        file << "[save_calibration_validation]\n";
        file << "value = " << params.save_calibration_validation << "\n\n";

        file << "[calibration_validation_points]\n";
        file << "value = " << params.calibration_validation_points << "\n\n";

        file << "[distance_bucket_rigid_sf]\n";
        file << "value = " << params.distance_bucket_rigid_sf << "\n\n";

        file << "[polar_angle_deg_rigid_sf]\n";
        file << "value = " << params.polar_angle_deg_rigid_sf << "\n\n";

        file << "[azimutal_angle_deg_rigid_sf]\n";
        file << "value = " << params.azimutal_angle_deg_rigid_sf << "\n\n";

        file << "[robust_and_accurate_lidar_odometry_rigid_sf_iterations]\n";
        file << "value = " << params.robust_and_accurate_lidar_odometry_rigid_sf_iterations << "\n\n";

        file << "[max_distance_lidar_rigid_sf]\n";
        file << "value = " << params.max_distance_lidar_rigid_sf << "\n\n";

        file << "[rgd_sf_sigma_x_m]\n";
        file << "value = " << params.rgd_sf_sigma_x_m << "\n\n";

        file << "[rgd_sf_sigma_y_m]\n";
        file << "value = " << params.rgd_sf_sigma_y_m << "\n\n";

        file << "[rgd_sf_sigma_z_m]\n";
        file << "value = " << params.rgd_sf_sigma_z_m << "\n\n";
        
        file << "[rgd_sf_sigma_om_deg]\n";
        file << "value = " << params.rgd_sf_sigma_om_deg << "\n\n";

        file << "[rgd_sf_sigma_fi_deg]\n";
        file << "value = " << params.rgd_sf_sigma_fi_deg << "\n\n";

        file << "[rgd_sf_sigma_ka_deg]\n";
        file << "value = " << params.rgd_sf_sigma_ka_deg << "\n\n";

        file << "[threshold_initial_points]\n";
        file << "value = " << params.threshold_initial_points << "\n\n";

        file << "[use_mutliple_gaussian]\n";
        file << "value = " << params.use_mutliple_gaussian << "\n\n";

        file << "[num_constistency_iter]\n";
        file << "value = " << params.num_constistency_iter << "\n\n";

        file << "[ahrs_gain]\n";
        file << "value = " << params.ahrs_gain << "\n\n";

        file << "[threshold_nr_poses]\n";
        file << "value = " << params.threshold_nr_poses << "\n\n";

        file << "[min_counter]\n";
        file << "value = " << params.min_counter << "\n\n";

        file << "[motion_model_correction.px]\n";
        file << "value = " << params.motion_model_correction.px << "\n\n";

        file << "[motion_model_correction.py]\n";
        file << "value = " << params.motion_model_correction.py << "\n\n";

        file << "[motion_model_correction.pz]\n";
        file << "value = " << params.motion_model_correction.pz << "\n\n";

        file << "[motion_model_correction.om]\n";
        file << "value = " << params.motion_model_correction.om << "\n\n";

        file << "[motion_model_correction.fi]\n";
        file << "value = " << params.motion_model_correction.fi << "\n\n";

        file << "[motion_model_correction.ka]\n";
        file << "value = " << params.motion_model_correction.ka << "\n\n";

        file << "[lidar_odometry_motion_model_x_1_sigma_m]\n";
        file << "value = " << params.lidar_odometry_motion_model_x_1_sigma_m << "\n\n";

        file << "[lidar_odometry_motion_model_y_1_sigma_m]\n";
        file << "value = " << params.lidar_odometry_motion_model_y_1_sigma_m << "\n\n";

        file << "[lidar_odometry_motion_model_z_1_sigma_m]\n";
        file << "value = " << params.lidar_odometry_motion_model_z_1_sigma_m << "\n\n";

        file << "[lidar_odometry_motion_model_om_1_sigma_deg]\n";
        file << "value = " << params.lidar_odometry_motion_model_om_1_sigma_deg << "\n\n";

        file << "[lidar_odometry_motion_model_fi_1_sigma_deg]\n";
        file << "value = " << params.lidar_odometry_motion_model_fi_1_sigma_deg << "\n\n";

        file << "[lidar_odometry_motion_model_ka_1_sigma_deg]\n";
        file << "value = " << params.lidar_odometry_motion_model_ka_1_sigma_deg << "\n\n";

        file << "[lidar_odometry_motion_model_fix_origin_x_1_sigma_m]\n";
        file << "value = " << params.lidar_odometry_motion_model_fix_origin_x_1_sigma_m << "\n\n";

        file << "[lidar_odometry_motion_model_fix_origin_y_1_sigma_m]\n";
        file << "value = " << params.lidar_odometry_motion_model_fix_origin_y_1_sigma_m << "\n\n";

        file << "[lidar_odometry_motion_model_fix_origin_z_1_sigma_m]\n";
        file << "value = " << params.lidar_odometry_motion_model_fix_origin_z_1_sigma_m << "\n\n";

        file << "[lidar_odometry_motion_model_fix_origin_om_1_sigma_deg]\n";
        file << "value = " << params.lidar_odometry_motion_model_fix_origin_om_1_sigma_deg << "\n\n";

        file << "[lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg]\n";
        file << "value = " << params.lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg << "\n\n";

        file << "[lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg]\n";
        file << "value = " << params.lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg << "\n\n";

        file << "# Calculations (optimization routine composed of 'nr_iter' iterations) will break once the time exceeds this value (measured in seconds)" << std::endl;
        file << "[real_time_threshold_seconds]\n";
        file << "value = " << params.real_time_threshold_seconds << "\n\n";

        //-----------------------

        file.close();
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error while saving file: " << e.what() << std::endl;
        return false;
    }
}

bool LoadParametersFromTomlFile(const std::string &filepath, LidarOdometryParams &params)
{
    try
    {
        toml::value data = toml::parse(filepath);

        if (data.contains("nr_iter"))
        {
            auto &tbl = toml::find(data, "nr_iter");
            if (tbl.contains("value")){
                params.nr_iter = toml::find<int>(tbl, "value");
                std::cout << "params.nr_iter: " << params.nr_iter << std::endl;
            }
        }

        if (data.contains("decimation"))
        {
            auto &tbl = toml::find(data, "decimation");
            if (tbl.contains("value")){
                params.decimation = toml::find<double>(tbl, "value");
                std::cout << "params.decimation: " << params.decimation << std::endl;
            }
        }

        if (data.contains("in_out_params_indoor"))
        {
            auto &indoor = toml::find(data, "in_out_params_indoor");
            if (indoor.contains("resolution_X")){
                params.in_out_params_indoor.resolution_X = toml::find<double>(indoor, "resolution_X");
                std::cout << "params.in_out_params_indoor.resolution_X: " << params.in_out_params_indoor.resolution_X << std::endl;
            }
            if (indoor.contains("resolution_Y")){
                params.in_out_params_indoor.resolution_Y = toml::find<double>(indoor, "resolution_Y");
                std::cout << "params.in_out_params_indoor.resolution_Y: " << params.in_out_params_indoor.resolution_Y << std::endl;
            }
            if (indoor.contains("resolution_Z")){
                params.in_out_params_indoor.resolution_Z = toml::find<double>(indoor, "resolution_Z");
                std::cout << "params.in_out_params_indoor.resolution_Z: " << params.in_out_params_indoor.resolution_Z << std::endl;
            }
        }

        if (data.contains("in_out_params_outdoor"))
        {
            auto &outdoor = toml::find(data, "in_out_params_outdoor");
            if (outdoor.contains("resolution_X")){
                params.in_out_params_outdoor.resolution_X = toml::find<double>(outdoor, "resolution_X");
                std::cout << "params.in_out_params_outdoor.resolution_X: " << params.in_out_params_outdoor.resolution_X << std::endl;
            }
            if (outdoor.contains("resolution_Y")){
                params.in_out_params_outdoor.resolution_Y = toml::find<double>(outdoor, "resolution_Y");
                std::cout << "params.in_out_params_outdoor.resolution_Y: " << params.in_out_params_outdoor.resolution_Y << std::endl;
            }
            if (outdoor.contains("resolution_Z")){
                params.in_out_params_outdoor.resolution_Z = toml::find<double>(outdoor, "resolution_Z");
                std::cout << "params.in_out_params_outdoor.resolution_Z: " << params.in_out_params_outdoor.resolution_Z << std::endl;
            }
        }

        if (data.contains("filter_threshold_xy_inner"))
        {
            auto &tbl = toml::find(data, "filter_threshold_xy_inner");
            if (tbl.contains("value")){
                params.filter_threshold_xy_inner = toml::find<double>(tbl, "value");
                std::cout << "params.filter_threshold_xy_inner: " << params.filter_threshold_xy_inner << std::endl;
            }
        }

        if (data.contains("filter_threshold_xy_outer"))
        {
            auto &tbl = toml::find(data, "filter_threshold_xy_outer");
            if (tbl.contains("value")){
                params.filter_threshold_xy_outer = toml::find<double>(tbl, "value");
                std::cout << "params.filter_threshold_xy_outer: " << params.filter_threshold_xy_outer << std::endl;
            }
        }

        if (data.contains("threshould_output_filter"))
        {
            auto &tbl = toml::find(data, "threshould_output_filter");
            if (tbl.contains("value")){
                params.threshould_output_filter = toml::find<double>(tbl, "value");
                std::cout << "params.threshould_output_filter: " << params.threshould_output_filter << std::endl;
            }
        }

        if (data.contains("distance_bucket"))
        {
            auto &tbl = toml::find(data, "distance_bucket");
            if (tbl.contains("value")){
                params.distance_bucket = toml::find<double>(tbl, "value");
                std::cout << "params.distance_bucket: " << params.distance_bucket << std::endl;
            }
        }

        if (data.contains("polar_angle_deg"))
        {
            auto &tbl = toml::find(data, "polar_angle_deg");
            if (tbl.contains("value")){
                params.polar_angle_deg = toml::find<double>(tbl, "value");
                std::cout << "params.polar_angle_deg: " << params.polar_angle_deg << std::endl;
            }
        }

        if (data.contains("azimutal_angle_deg"))
        {
            auto &tbl = toml::find(data, "azimutal_angle_deg");
            if (tbl.contains("value")){
                params.azimutal_angle_deg = toml::find<double>(tbl, "value");
                std::cout << "params.azimutal_angle_deg: " << params.azimutal_angle_deg << std::endl;
            }
        }

        if (data.contains("robust_and_accurate_lidar_odometry_iterations"))
        {
            auto &tbl = toml::find(data, "robust_and_accurate_lidar_odometry_iterations");
            if (tbl.contains("value")){
                params.robust_and_accurate_lidar_odometry_iterations = toml::find<int>(tbl, "value");
                std::cout << "params.robust_and_accurate_lidar_odometry_iterations: " << params.robust_and_accurate_lidar_odometry_iterations << std::endl;
            }
        }

        if (data.contains("max_distance_lidar"))
        {
            auto &tbl = toml::find(data, "max_distance_lidar");
            if (tbl.contains("value")){
                params.max_distance_lidar = toml::find<double>(tbl, "value");
                std::cout << "params.max_distance_lidar: " << params.max_distance_lidar << std::endl;
            }
        }

        if (data.contains("use_robust_and_accurate_lidar_odometry"))
        {
            auto &tbl = toml::find(data, "use_robust_and_accurate_lidar_odometry");
            if (tbl.contains("use_robust_and_accurate"))
            {
                params.use_robust_and_accurate_lidar_odometry = toml::find<bool>(tbl, "use_robust_and_accurate");
                std::cout << "params.use_robust_and_accurate_lidar_odometry: " << (int)params.use_robust_and_accurate_lidar_odometry << std::endl;
            }
        }

        if (data.contains("fusionConventionNed"))
        {
            const auto &ned_tbl = toml::find<toml::value>(data, "fusionConventionNed");
            if (ned_tbl.contains("Ned")){
                params.fusionConventionNed = toml::find<bool>(ned_tbl, "Ned");
                std::cout << "params.fusionConventionNed: " << (int)params.fusionConventionNed << std::endl;
            }
        }

        if (data.contains("fusionConventionNwu"))
        {
            const auto &nwu_tbl = toml::find<toml::value>(data, "fusionConventionNwu");
            if (nwu_tbl.contains("Nwu")){
                params.fusionConventionNwu = toml::find<bool>(nwu_tbl, "Nwu");
                std::cout << "params.fusionConventionNwu: " << (int)params.fusionConventionNwu << std::endl;
            }
        }

        if (data.contains("fusionConventionEnu"))
        {
            const auto &enu_tbl = toml::find<toml::value>(data, "fusionConventionEnu");
            if (enu_tbl.contains("Enu")){
                params.fusionConventionEnu = toml::find<bool>(enu_tbl, "Enu");
                std::cout << "params.fusionConventionEnu: " << (int)params.fusionConventionEnu << std::endl;
            }
        }

        if (params.fusionConventionNed)
        {
            params.fusionConventionNwu = false;
            params.fusionConventionEnu = false;
        }
        else if (params.fusionConventionNwu)
        {
            params.fusionConventionNed = false;
            params.fusionConventionEnu = false;
        }
        else if (params.fusionConventionEnu)
        {
            params.fusionConventionNed = false;
            params.fusionConventionNwu = false;
        }

        if (data.contains("use_motion_from_previous_step"))
        {
            auto &tbl = toml::find(data, "use_motion_from_previous_step");
            if (tbl.contains("value"))
            {
                params.use_motion_from_previous_step = toml::find<bool>(tbl, "value");
                std::cout << "params.use_motion_from_previous_step: " << (int)params.use_motion_from_previous_step << std::endl;
            }
        }

        if (data.contains("useMultithread"))
        {
            auto &tbl = toml::find(data, "useMultithread");
            if (tbl.contains("value"))
            {
                params.useMultithread = toml::find<bool>(tbl, "value");
                std::cout << "params.useMultithread: " << (int)params.useMultithread << std::endl;
            }
        }

        if (data.contains("sliding_window_trajectory_length_threshold"))
        {
            auto &tbl = toml::find(data, "sliding_window_trajectory_length_threshold");
            if (tbl.contains("value")){
                params.sliding_window_trajectory_length_threshold = toml::find<double>(tbl, "value");
                std::cout << "params.sliding_window_trajectory_length_threshold: " << params.sliding_window_trajectory_length_threshold << std::endl;
            }
        }

        if (data.contains("save_calibration_validation"))
        {
            auto &tbl = toml::find(data, "save_calibration_validation");
            if (tbl.contains("value")){
                params.save_calibration_validation = toml::find<bool>(tbl, "value");
                std::cout << "params.save_calibration_validation: " << (int)params.save_calibration_validation << std::endl;
            }
        }

        if (data.contains("calibration_validation_points"))
        {
            auto &tbl = toml::find(data, "calibration_validation_points");
            if (tbl.contains("value")){
                params.calibration_validation_points = toml::find<int>(tbl, "value");
                std::cout << "params.calibration_validation_points: " << params.calibration_validation_points << std::endl;
            }
        }

        if (data.contains("distance_bucket_rigid_sf"))
        {
            auto &tbl = toml::find(data, "distance_bucket_rigid_sf");
            if (tbl.contains("value")){
                params.distance_bucket_rigid_sf = toml::find<double>(tbl, "value");
                std::cout << "params.distance_bucket_rigid_sf: " << params.distance_bucket_rigid_sf << std::endl;
            }
        }

        if (data.contains("polar_angle_deg_rigid_sf"))
        {
            auto &tbl = toml::find(data, "polar_angle_deg_rigid_sf");
            if (tbl.contains("value")){
                params.polar_angle_deg_rigid_sf = toml::find<double>(tbl, "value");
                std::cout << "params.polar_angle_deg_rigid_sf: " << params.polar_angle_deg_rigid_sf << std::endl;
            }
        }

        if (data.contains("azimutal_angle_deg_rigid_sf"))
        {
            auto &tbl = toml::find(data, "azimutal_angle_deg_rigid_sf");
            if (tbl.contains("value")){
                params.azimutal_angle_deg_rigid_sf = toml::find<double>(tbl, "value");
                std::cout << "params.azimutal_angle_deg_rigid_sf: " << params.azimutal_angle_deg_rigid_sf << std::endl;
            }
        }

        if (data.contains("robust_and_accurate_lidar_odometry_rigid_sf_iterations"))
        {
            auto &tbl = toml::find(data, "robust_and_accurate_lidar_odometry_rigid_sf_iterations");
            if (tbl.contains("value")){
                params.robust_and_accurate_lidar_odometry_rigid_sf_iterations = toml::find<int>(tbl, "value");
                std::cout << "params.robust_and_accurate_lidar_odometry_rigid_sf_iterations: " << params.robust_and_accurate_lidar_odometry_rigid_sf_iterations << std::endl;
            }
        }

        if (data.contains("max_distance_lidar_rigid_sf"))
        {
            auto &tbl = toml::find(data, "max_distance_lidar_rigid_sf");
            if (tbl.contains("value")){
                params.max_distance_lidar_rigid_sf = toml::find<double>(tbl, "value");
                std::cout << "params.max_distance_lidar_rigid_sf: " << params.max_distance_lidar_rigid_sf << std::endl;
            }
        }

        if (data.contains("rgd_sf_sigma_x_m"))
        {
            auto &tbl = toml::find(data, "rgd_sf_sigma_x_m");
            if (tbl.contains("value")){
                params.rgd_sf_sigma_x_m = toml::find<double>(tbl, "value");
                std::cout << "params.rgd_sf_sigma_x_m: " << params.rgd_sf_sigma_x_m << std::endl;
            }
        }

        if (data.contains("rgd_sf_sigma_y_m"))
        {
            auto &tbl = toml::find(data, "rgd_sf_sigma_y_m");
            if (tbl.contains("value")){
                params.rgd_sf_sigma_y_m = toml::find<double>(tbl, "value");
                std::cout << "params.rgd_sf_sigma_y_m: " << params.rgd_sf_sigma_y_m << std::endl;
            }
        }

        if (data.contains("rgd_sf_sigma_z_m"))
        {
            auto &tbl = toml::find(data, "rgd_sf_sigma_z_m");
            if (tbl.contains("value")){
                params.rgd_sf_sigma_z_m = toml::find<double>(tbl, "value");
                std::cout << "params.rgd_sf_sigma_z_m: " << params.rgd_sf_sigma_z_m << std::endl;
            }
        }

        if (data.contains("rgd_sf_sigma_om_deg"))
        {
            auto &tbl = toml::find(data, "rgd_sf_sigma_om_deg");
            if (tbl.contains("value")){
                params.rgd_sf_sigma_om_deg = toml::find<double>(tbl, "value");
                std::cout << "params.rgd_sf_sigma_om_deg: " << params.rgd_sf_sigma_om_deg << std::endl;
            }
        }

        if (data.contains("rgd_sf_sigma_fi_deg"))
        {
            auto &tbl = toml::find(data, "rgd_sf_sigma_fi_deg");
            if (tbl.contains("value")){
                params.rgd_sf_sigma_fi_deg = toml::find<double>(tbl, "value");
                std::cout << "params.rgd_sf_sigma_fi_deg: " << params.rgd_sf_sigma_fi_deg << std::endl;
            }
        }

        if (data.contains("rgd_sf_sigma_ka_deg"))
        {
            auto &tbl = toml::find(data, "rgd_sf_sigma_ka_deg");
            if (tbl.contains("value")){
                params.rgd_sf_sigma_ka_deg = toml::find<double>(tbl, "value");
                std::cout << "params.rgd_sf_sigma_ka_deg: " << params.rgd_sf_sigma_ka_deg << std::endl;
            }
        }

        if (data.contains("threshold_initial_points"))
        {
            auto &tbl = toml::find(data, "threshold_initial_points");
            if (tbl.contains("value")){
                params.threshold_initial_points = toml::find<int>(tbl, "value");
                std::cout << "params.threshold_initial_points: " << params.threshold_initial_points << std::endl;
            }
        }

        if (data.contains("use_mutliple_gaussian"))
        {
            auto &tbl = toml::find(data, "use_mutliple_gaussian");
            if (tbl.contains("value")){
                params.use_mutliple_gaussian = toml::find<bool>(tbl, "value");
                std::cout << "params.use_mutliple_gaussian: " << (int)params.use_mutliple_gaussian << std::endl;
            }
        }

        if (data.contains("num_constistency_iter"))
        {
            auto &tbl = toml::find(data, "num_constistency_iter");
            if (tbl.contains("value")){
                params.num_constistency_iter = toml::find<int>(tbl, "value");
                std::cout << "params.num_constistency_iter: " << params.num_constistency_iter << std::endl;
            }
        }

        if (data.contains("ahrs_gain"))
        {
            auto &tbl = toml::find(data, "ahrs_gain");
            if (tbl.contains("value")){
                params.ahrs_gain = toml::find<double>(tbl, "value");
                std::cout << "params.ahrs_gain: " << params.ahrs_gain << std::endl;
            }
        }

        if (data.contains("threshold_nr_poses"))
        {
            auto &tbl = toml::find(data, "threshold_nr_poses");
            if (tbl.contains("value")){
                params.threshold_nr_poses = toml::find<int>(tbl, "value");
                std::cout << "params.threshold_nr_poses: " << params.threshold_nr_poses << std::endl;
            }
        }

        if (data.contains("min_counter"))
        {
            auto &tbl = toml::find(data, "min_counter");
            if (tbl.contains("value")){
                params.min_counter = toml::find<int>(tbl, "value");
                std::cout << "params.min_counter: " << params.min_counter << std::endl;
            }
        }

        if (data.contains("motion_model_correction.px"))
        {
            auto &tbl = toml::find(data, "motion_model_correction.px");
            if (tbl.contains("value")){
                params.motion_model_correction.px = toml::find<double>(tbl, "value");
                std::cout << "params.motion_model_correction.px: " << params.motion_model_correction.px << std::endl;
            }
        }

        if (data.contains("motion_model_correction.py"))
        {
            auto &tbl = toml::find(data, "motion_model_correction.py");
            if (tbl.contains("value")){
                params.motion_model_correction.py = toml::find<double>(tbl, "value");
                std::cout << "params.motion_model_correction.py: " << params.motion_model_correction.py << std::endl;
            }
        }

        if (data.contains("motion_model_correction.pz"))
        {
            auto &tbl = toml::find(data, "motion_model_correction.pz");
            if (tbl.contains("value")){
                params.motion_model_correction.pz = toml::find<double>(tbl, "value");
                std::cout << "params.motion_model_correction.pz: " << params.motion_model_correction.pz << std::endl;
            }
        }

        if (data.contains("motion_model_correction.om_deg"))
        {
            auto &tbl = toml::find(data, "motion_model_correction.om");
            if (tbl.contains("value")){
                params.motion_model_correction.om = toml::find<double>(tbl, "value");
                std::cout << "params.motion_model_correction.om_deg: " << params.motion_model_correction.om << std::endl;
            }
        }

        if (data.contains("motion_model_correction.fi_deg"))
        {
            auto &tbl = toml::find(data, "motion_model_correction.fi");
            if (tbl.contains("value")){
                params.motion_model_correction.fi = toml::find<double>(tbl, "value");
                std::cout << "params.motion_model_correction.fi_deg: " << params.motion_model_correction.fi << std::endl;
             }
        }

        if (data.contains("motion_model_correction.ka_deg"))
        {
            auto &tbl = toml::find(data, "motion_model_correction.ka");
            if (tbl.contains("value")){
                params.motion_model_correction.ka = toml::find<double>(tbl, "value");
                std::cout << "params.motion_model_correction.ka_deg: " << params.motion_model_correction.ka << std::endl;
            }
        }

        if (data.contains("lidar_odometry_motion_model_x_1_sigma_m"))
        {
            auto &tbl = toml::find(data, "lidar_odometry_motion_model_x_1_sigma_m");
            if (tbl.contains("value")){
                params.lidar_odometry_motion_model_x_1_sigma_m = toml::find<double>(tbl, "value");
                std::cout << "params.lidar_odometry_motion_model_x_1_sigma_m: " << params.lidar_odometry_motion_model_x_1_sigma_m << std::endl;
            }
        }

        if (data.contains("lidar_odometry_motion_model_y_1_sigma_m"))
        {
            auto &tbl = toml::find(data, "lidar_odometry_motion_model_y_1_sigma_m");
            if (tbl.contains("value")){
                params.lidar_odometry_motion_model_y_1_sigma_m = toml::find<double>(tbl, "value");
                std::cout << "params.lidar_odometry_motion_model_y_1_sigma_m: " << params.lidar_odometry_motion_model_y_1_sigma_m << std::endl;
            }
        }

        if (data.contains("lidar_odometry_motion_model_z_1_sigma_m"))
        {
            auto &tbl = toml::find(data, "lidar_odometry_motion_model_z_1_sigma_m");
            if (tbl.contains("value")){
                params.lidar_odometry_motion_model_z_1_sigma_m = toml::find<double>(tbl, "value");
                std::cout << "params.lidar_odometry_motion_model_z_1_sigma_m: " << params.lidar_odometry_motion_model_z_1_sigma_m << std::endl;
            }
        }

        if (data.contains("lidar_odometry_motion_model_om_1_sigma_deg"))
        {
            auto &tbl = toml::find(data, "lidar_odometry_motion_model_om_1_sigma_deg");
            if (tbl.contains("value")){
                params.lidar_odometry_motion_model_om_1_sigma_deg = toml::find<double>(tbl, "value");
                std::cout << "params.lidar_odometry_motion_model_om_1_sigma_deg: " << params.lidar_odometry_motion_model_om_1_sigma_deg << std::endl;
            }
        }

        if (data.contains("lidar_odometry_motion_model_fi_1_sigma_deg"))
        {
            auto &tbl = toml::find(data, "lidar_odometry_motion_model_fi_1_sigma_deg");
            if (tbl.contains("value")){
                params.lidar_odometry_motion_model_fi_1_sigma_deg = toml::find<double>(tbl, "value");
                std::cout << "params.lidar_odometry_motion_model_fi_1_sigma_deg: " << params.lidar_odometry_motion_model_fi_1_sigma_deg << std::endl;
            }
        }

        if (data.contains("lidar_odometry_motion_model_ka_1_sigma_deg"))
        {
            auto &tbl = toml::find(data, "lidar_odometry_motion_model_ka_1_sigma_deg");
            if (tbl.contains("value")){
                params.lidar_odometry_motion_model_ka_1_sigma_deg = toml::find<double>(tbl, "value");
                std::cout << "params.lidar_odometry_motion_model_ka_1_sigma_deg: " << params.lidar_odometry_motion_model_ka_1_sigma_deg << std::endl;
            }
        }

        if (data.contains("lidar_odometry_motion_model_fix_origin_x_1_sigma_m"))
        {
            auto &tbl = toml::find(data, "lidar_odometry_motion_model_fix_origin_x_1_sigma_m");
            if (tbl.contains("value")){
                params.lidar_odometry_motion_model_fix_origin_x_1_sigma_m = toml::find<double>(tbl, "value");
                std::cout << "params.lidar_odometry_motion_model_fix_origin_x_1_sigma_m: " << params.lidar_odometry_motion_model_fix_origin_x_1_sigma_m << std::endl;
            }
        }

        if (data.contains("lidar_odometry_motion_model_fix_origin_y_1_sigma_m"))
        {
            auto &tbl = toml::find(data, "lidar_odometry_motion_model_fix_origin_y_1_sigma_m");
            if (tbl.contains("value")){
                params.lidar_odometry_motion_model_fix_origin_y_1_sigma_m = toml::find<double>(tbl, "value");
                std::cout << "params.lidar_odometry_motion_model_fix_origin_y_1_sigma_m: " << params.lidar_odometry_motion_model_fix_origin_y_1_sigma_m << std::endl;
            }
        }

        if (data.contains("lidar_odometry_motion_model_fix_origin_z_1_sigma_m"))
        {
            auto &tbl = toml::find(data, "lidar_odometry_motion_model_fix_origin_z_1_sigma_m");
            if (tbl.contains("value")){
                params.lidar_odometry_motion_model_fix_origin_z_1_sigma_m = toml::find<double>(tbl, "value");
                std::cout << "params.lidar_odometry_motion_model_fix_origin_z_1_sigma_m: " << params.lidar_odometry_motion_model_fix_origin_z_1_sigma_m << std::endl;
            }
        }

        if (data.contains("lidar_odometry_motion_model_fix_origin_om_1_sigma_deg"))
        {
            auto &tbl = toml::find(data, "lidar_odometry_motion_model_fix_origin_om_1_sigma_deg");
            if (tbl.contains("value")){
                params.lidar_odometry_motion_model_fix_origin_om_1_sigma_deg = toml::find<double>(tbl, "value");
                std::cout << "params.lidar_odometry_motion_model_fix_origin_om_1_sigma_deg: " << params.lidar_odometry_motion_model_fix_origin_om_1_sigma_deg << std::endl;
            }
        }

        if (data.contains("lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg"))
        {
            auto &tbl = toml::find(data, "lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg");
            if (tbl.contains("value")){
                params.lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg = toml::find<double>(tbl, "value");
                std::cout << "params.lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg: " << params.lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg << std::endl;
            }
        }

        if (data.contains("lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg"))
        {
            auto &tbl = toml::find(data, "lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg");
            if (tbl.contains("value")){
                params.lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg = toml::find<double>(tbl, "value");
                std::cout << "params.lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg: " << params.lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg << std::endl;
            }
        }

        if (data.contains("real_time_threshold_seconds"))
        {
            auto &tbl = toml::find(data, "real_time_threshold_seconds");
            if (tbl.contains("value")){
                params.real_time_threshold_seconds = toml::find<double>(tbl, "value");
                std::cout << "params.real_time_threshold_seconds: " << params.real_time_threshold_seconds << std::endl;
            }
        }

        std::cout << "Parameters loaded from TOML.\n";
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Failed to load parameters: " << e.what() << "\n";
        return false;
    }
}
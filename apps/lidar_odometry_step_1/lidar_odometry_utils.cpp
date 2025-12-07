#include "lidar_odometry_utils.h"
#include <filesystem>
#include "csv.hpp"
#include <algorithm>
#include <regex>
#include <filesystem>
// #include <toml.hpp>

namespace fs = std::filesystem;

// this function provides unique index
unsigned long long int get_index(const int16_t x, const int16_t y, const int16_t z)
{
    return ((static_cast<unsigned long long int>(x) << 32) & (0x0000FFFF00000000ull)) |
           ((static_cast<unsigned long long int>(y) << 16) & (0x00000000FFFF0000ull)) |
           ((static_cast<unsigned long long int>(z) << 0) & (0x000000000000FFFFull));
}

// this function provides unique index for input point p and 3D space decomposition into buckets b
unsigned long long int get_rgd_index(const Eigen::Vector3d p, const Eigen::Vector3d b)
{
    int16_t x = static_cast<int16_t>(p.x() / b.x());
    int16_t y = static_cast<int16_t>(p.y() / b.y());
    int16_t z = static_cast<int16_t>(p.z() / b.z());
    return get_index(x, y, z);
}

std::vector<Point3Di> decimate(const std::vector<Point3Di>& points, double bucket_x, double bucket_y, double bucket_z)
{
    // std::cout << "points.size before decimation: " << points.size() << std::endl;
    Eigen::Vector3d b(bucket_x, bucket_y, bucket_z);
    std::vector<Point3Di> out;

    std::vector<PointCloud::PointBucketIndexPair> ip;
    ip.resize(points.size());
    out.reserve(points.size());

    for (int i = 0; i < points.size(); i++)
    {
        ip[i].index_of_point = i;
        ip[i].index_of_bucket = get_rgd_index(points[i].point, b);
    }
    std::sort(ip.begin(), ip.end(), [](const PointCloud::PointBucketIndexPair& a, const PointCloud::PointBucketIndexPair& b)
        { return a.index_of_bucket < b.index_of_bucket; });

    if (ip.size() != 0)
	    out.emplace_back(points[ip[0].index_of_point]);

    for (int i = 1; i < ip.size(); i++)
        if (ip[i - 1].index_of_bucket != ip[i].index_of_bucket)
            out.emplace_back(points[ip[i].index_of_point]);

    // std::cout << "points.size after decimation: " << out.size() << std::endl;
    return out;
}

Eigen::Matrix4d getInterpolatedPose(const std::map<double, Eigen::Matrix4d> &trajectory, double query_time)
{

    Eigen::Matrix4d ret(Eigen::Matrix4d::Zero());
    auto it_lower = trajectory.lower_bound(query_time);
    auto it_next = it_lower;

    if (it_lower == trajectory.begin())
    {
        // std::cout << "1" << std::endl;
        return ret;
    }
    if (it_lower->first > query_time)
    {
        // std::cout << "2" << std::endl;
        it_lower = std::prev(it_lower);
    }
    if (it_lower == trajectory.begin())
    {
        // std::cout << "3" << std::endl;
        return ret;
    }
    if (it_lower == trajectory.end())
    {
        // std::cout << "4" << std::endl;
        return ret;
    }
    // std::cout << std::setprecision(10);
    // std::cout << it_lower->first << " " << query_time << " " << it_next->first << " " << std::next(it_lower)->first << std::endl;

    double t1 = it_lower->first;
    double t2 = it_next->first;
    double difft1 = t1 - query_time;
    double difft2 = t2 - query_time;
    if (t1 == t2 && std::fabs(difft1) < 0.1)
    {
        // std::cout << "5" << std::endl;
        ret = Eigen::Matrix4d::Identity();
        ret.col(3).head<3>() = it_next->second.col(3).head<3>();
        ret.topLeftCorner(3, 3) = it_lower->second.topLeftCorner(3, 3);
        return ret;
    }

    // std::cout << std::fabs(difft1) << " " << std::fabs(difft2) << std::endl;
    // if (std::fabs(difft1) < 0.15 && std::fabs(difft2) < 0.15)
    {
        // std::cout << "6" << std::endl;
        assert(t2 > t1);
        assert(query_time > t1);
        assert(query_time < t2);
        ret = Eigen::Matrix4d::Identity();
        double res = (query_time - t1) / (t2 - t1);
        Eigen::Vector3d diff = it_next->second.col(3).head<3>() - it_lower->second.col(3).head<3>();
        ret.col(3).head<3>() = it_next->second.col(3).head<3>() + diff * res;
        Eigen::Matrix3d r1 = it_lower->second.topLeftCorner(3, 3).matrix();
        Eigen::Matrix3d r2 = it_next->second.topLeftCorner(3, 3).matrix();
        Eigen::Quaterniond q1(r1);
        Eigen::Quaterniond q2(r2);
        Eigen::Quaterniond qt = q1.slerp(res, q2);
        ret.topLeftCorner(3, 3) = qt.toRotationMatrix();
        return ret;
    }
    // std::cout << "Problem with : " << difft1 << " " << difft2 << "  q : " << query_time << " t1 :" << t1 << " t2: " << t2 << std::endl;
    return ret;
}

void limit_covariance(Eigen::Matrix3d &io_cov)
{
    return;
    // std::cout << "------io_cov in ------------" << std::endl;
    // std::cout << io_cov << std::endl;

    Eigen::EigenSolver<Eigen::Matrix3d> eigensolver;
    eigensolver.compute(io_cov);

    Eigen::Vector3d eigenValues = eigensolver.eigenvalues().real();
    Eigen::Matrix3d eigenVectors = eigensolver.eigenvectors().real();

    // modify eigen values
    for (int k = 0; k < 3; ++k)
    {
        eigenValues(k) = std::max(eigenValues(k), 0.0001);
    }

    // create diagonal matrix
    Eigen::DiagonalMatrix<double, 3> diagonal_matrix(eigenValues(0), eigenValues(1), eigenValues(2));

    // update covariance
    io_cov = eigenVectors * diagonal_matrix * eigenVectors.inverse();
    // std::cout << "------io_cov out ------------" << std::endl;
    // std::cout << io_cov << std::endl;
}

void update_rgd(NDT::GridParameters &rgd_params, NDTBucketMapType &buckets,
                std::vector<Point3Di> &points_global, Eigen::Vector3d viewport)
{
    Eigen::Vector3d b(rgd_params.resolution_X, rgd_params.resolution_Y, rgd_params.resolution_Z);

    for (int i = 0; i < points_global.size(); i++)
    {
        auto index_of_bucket = get_rgd_index(points_global[i].point, b);

        auto bucket_it = buckets.find(index_of_bucket);

        if (bucket_it != buckets.end())
        {
            auto &this_bucket = bucket_it->second;
            this_bucket.number_of_points++;
            const auto &curr_mean = points_global[i].point;
            const auto &mean = this_bucket.mean;
            // buckets[index_of_bucket].mean += (mean - curr_mean) / buckets[index_of_bucket].number_of_points;

            auto mean_diff = mean - curr_mean;
            Eigen::Matrix3d cov_update;
            cov_update.row(0) = mean_diff.x() * mean_diff;
            cov_update.row(1) = mean_diff.y() * mean_diff;
            cov_update.row(2) = mean_diff.z() * mean_diff;

            // this_bucket.cov = this_bucket.cov * (this_bucket.number_of_points - 1) / this_bucket.number_of_points +
            //                   cov_update * (this_bucket.number_of_points - 1) / (this_bucket.number_of_points * this_bucket.number_of_points);

            if (this_bucket.number_of_points == 2)
            {
                this_bucket.cov = this_bucket.cov * (this_bucket.number_of_points - 1) / this_bucket.number_of_points +
                                  cov_update * (this_bucket.number_of_points - 1) / (this_bucket.number_of_points * this_bucket.number_of_points);

                // limit_covariance(this_bucket.cov);
            }

            if (this_bucket.number_of_points == 3)
            {
                this_bucket.cov = this_bucket.cov * (this_bucket.number_of_points - 1) / this_bucket.number_of_points +
                                  cov_update * (this_bucket.number_of_points - 1) / (this_bucket.number_of_points * this_bucket.number_of_points);
                // limit_covariance(this_bucket.cov);
                //  calculate normal vector
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(this_bucket.cov, Eigen::ComputeEigenvectors);
                Eigen::Matrix3d eigenVectorsPCA = eigen_solver.eigenvectors();

                Eigen::Vector3d nv = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
                nv.normalize();

                // flip towards viewport
                if (nv.dot(viewport - this_bucket.mean) < 0.0)
                {
                    nv *= -1.0;
                }
                this_bucket.normal_vector = nv;
            }

            if (this_bucket.number_of_points > 3)
            {
                Eigen::Vector3d &nv = this_bucket.normal_vector;

                if (nv.dot(viewport - this_bucket.mean) >= 0.0)
                {
                    this_bucket.cov = this_bucket.cov * (this_bucket.number_of_points - 1) / this_bucket.number_of_points +
                                      cov_update * (this_bucket.number_of_points - 1) / (this_bucket.number_of_points * this_bucket.number_of_points);
                    // limit_covariance(this_bucket.cov);
                }
            }
        }
        else
        {
            NDT::Bucket bucket_to_add;
            bucket_to_add.mean = points_global[i].point;
            bucket_to_add.cov = Eigen::Matrix3d::Identity() * 0.03 * 0.03;
            bucket_to_add.number_of_points = 1;
            buckets.emplace(index_of_bucket, bucket_to_add);
        }
    }
}

void update_rgd_spherical_coordinates(NDT::GridParameters &rgd_params, NDTBucketMapType &buckets,
                                      std::vector<Point3Di> &points_global, std::vector<Eigen::Vector3d> &points_global_spherical)
{
    Eigen::Vector3d b(rgd_params.resolution_X, rgd_params.resolution_Y, rgd_params.resolution_Z);

    for (int i = 0; i < points_global.size(); i++)
    {
        auto index_of_bucket = get_rgd_index(points_global_spherical[i], b);

        auto bucket_it = buckets.find(index_of_bucket);

        if (bucket_it != buckets.end())
        {
            auto &this_bucket = bucket_it->second;
            this_bucket.number_of_points++;
            const auto &curr_mean = points_global[i].point;
            const auto &mean = this_bucket.mean;
            // buckets[index_of_bucket].mean += (mean - curr_mean) / buckets[index_of_bucket].number_of_points;

            auto mean_diff = mean - curr_mean;
            Eigen::Matrix3d cov_update;
            cov_update.row(0) = mean_diff.x() * mean_diff;
            cov_update.row(1) = mean_diff.y() * mean_diff;
            cov_update.row(2) = mean_diff.z() * mean_diff;

            // this_bucket.cov = this_bucket.cov * (this_bucket.number_of_points - 1) / this_bucket.number_of_points +
            //                   cov_update * (this_bucket.number_of_points - 1) / (this_bucket.number_of_points * this_bucket.number_of_points);

            if (this_bucket.number_of_points == 2)
            {
                this_bucket.cov = this_bucket.cov * (this_bucket.number_of_points - 1) / this_bucket.number_of_points +
                                  cov_update * (this_bucket.number_of_points - 1) / (this_bucket.number_of_points * this_bucket.number_of_points);
                // limit_covariance(this_bucket.cov);
            }

            if (this_bucket.number_of_points >= 3)
            {
                this_bucket.cov = this_bucket.cov * (this_bucket.number_of_points - 1) / this_bucket.number_of_points +
                                  cov_update * (this_bucket.number_of_points - 1) / (this_bucket.number_of_points * this_bucket.number_of_points);
                // limit_covariance(this_bucket.cov);
                //  calculate normal vector
                // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(this_bucket.cov, Eigen::ComputeEigenvectors);
                // Eigen::Matrix3d eigenVectorsPCA = eigen_solver.eigenvectors();

                // Eigen::Vector3d nv = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
                // nv.normalize();

                // flip towards viewport
                // if (nv.dot(viewport - this_bucket.mean) < 0.0)
                //{
                //    nv *= -1.0;
                //}
                // this_bucket.normal_vector = nv;
            }

            // if (this_bucket.number_of_points > 3)
            //{
            // Eigen::Vector3d &nv = this_bucket.normal_vector;

            // if (nv.dot(viewport - this_bucket.mean) >= 0.0)
            //{
            //         this_bucket.cov = this_bucket.cov * (this_bucket.number_of_points - 1) / this_bucket.number_of_points +
            //                           cov_update * (this_bucket.number_of_points - 1) / (this_bucket.number_of_points * this_bucket.number_of_points);
            // limit_covariance(this_bucket.cov);
            // }
            // }
        }
        else
        {
            NDT::Bucket bucket_to_add;
            bucket_to_add.mean = points_global[i].point;
            bucket_to_add.cov = Eigen::Matrix3d::Identity() * 0.03 * 0.03;
            bucket_to_add.number_of_points = 1;
            buckets.emplace(index_of_bucket, bucket_to_add);
        }
    }
}

bool save_poses(const std::string file_name, std::vector<Eigen::Affine3d> m_poses, std::vector<std::string> filenames)
{
    std::ofstream outfile;
    outfile.open(file_name);
    if (!outfile.good())
    {
        std::cout << "can not save file: '" << file_name << "'" << std::endl;
        std::cout << "if You can see only '' it means there is no filename assigned to poses, please read manual or contact me januszbedkowski@gmail.com" << std::endl;
        std::cout << "To assign filename to poses please use following two buttons in multi_view_tls_registration_step_2" << std::endl;
        std::cout << "1: update initial poses from RESSO file" << std::endl;
        std::cout << "2: update poses from RESSO file" << std::endl;
        return false;
    }

    outfile << m_poses.size() << std::endl;
    for (size_t i = 0; i < m_poses.size(); i++)
    {
        outfile << filenames[i] << std::endl;
        outfile << m_poses[i](0, 0) << " " << m_poses[i](0, 1) << " " << m_poses[i](0, 2) << " " << m_poses[i](0, 3) << std::endl;
        outfile << m_poses[i](1, 0) << " " << m_poses[i](1, 1) << " " << m_poses[i](1, 2) << " " << m_poses[i](1, 3) << std::endl;
        outfile << m_poses[i](2, 0) << " " << m_poses[i](2, 1) << " " << m_poses[i](2, 2) << " " << m_poses[i](2, 3) << std::endl;
        outfile << "0 0 0 1" << std::endl;
    }
    outfile.close();

    return true;
}

std::vector<std::tuple<std::pair<double, double>, FusionVector, FusionVector>> load_imu(const std::string &imu_file, int imuToUse)
{
    std::vector<std::tuple<std::pair<double, double>, FusionVector, FusionVector>> all_data;

    csv::CSVFormat format;
    format.delimiter({' ', ',', '\t'});

    try
    {
        csv::CSVReader reader(imu_file, format);

        const auto columns = reader.get_col_names();
        const std::set<std::string> columnsSet(columns.begin(), columns.end());

        // mandatory columns
        const bool hasTsColumn = columnsSet.contains("timestamp");
        const bool hasGyrosColumns = columnsSet.contains("gyroX") && columnsSet.contains("gyroY") && columnsSet.contains("gyroZ");
        const bool hasAccsColumns = columnsSet.contains("accX") && columnsSet.contains("accY") && columnsSet.contains("accZ");

        // optional
        const bool hasImuIdColumn = columnsSet.contains("imuId");
        const bool hasUnixTimestampColumn = columnsSet.contains("timestampUnix");

        // check if legacy
        bool is_legacy = true;
        if (hasTsColumn)
        {
            is_legacy = false;
            if (!hasAccsColumns && !hasGyrosColumns && !hasUnixTimestampColumn)
            {
                std::cerr << "Input csv file is missing one of the mandatory columns :\n";
                std::cerr << "timestamp,timestampUnix,gyroX,gyroY,gyroZ,accX,accY,accZ";
                return all_data;
            }
        }

        if (!is_legacy)
        {
            // check if all needed columns are in csv
            for (auto row : reader)
            {
                int imu_id = -1;
                if (hasImuIdColumn)
                {
                    imu_id = row["imuId"].get<int>();
                }
                if (imu_id < 0 || imuToUse == imu_id)
                {
                    double timestamp = row["timestamp"].get<double>();
                    double timestampUnix = row["timestampUnix"].get<double>();
                    FusionVector gyr;
                    gyr.axis.x = row["gyroX"].get<double>();
                    gyr.axis.y = row["gyroY"].get<double>();
                    gyr.axis.z = row["gyroZ"].get<double>();
                    FusionVector acc;
                    acc.axis.x = row["accX"].get<double>();
                    acc.axis.y = row["accY"].get<double>();
                    acc.axis.z = row["accZ"].get<double>();
                    all_data.emplace_back(std::pair(timestamp / 1e9, timestampUnix / 1e9), gyr, acc);
                    // std::cout << "acc.axis.x: " << acc.axis.x << " acc.axis.y: " << acc.axis.y << " acc.axis.z " << acc.axis.z << " imu_id: " << imu_id << std::endl;
                }
            }
        }
        if (is_legacy)
        {
            std::ifstream myfile(imu_file);
            if (myfile.is_open())
            {
                while (myfile)
                {
                    double data[7];
                    double timestampUnix = 0.0;
                    int imuId = 0;
                    std::string line;
                    std::getline(myfile, line);
                    std::istringstream iss(line);
                    iss >> data[0] >> data[1] >> data[2] >> data[3] >> data[4] >> data[5] >> data[6];
                    if (!iss.eof())
                    {
                        iss >> imuId;
                    }
                    if (!iss.eof())
                    {
                        iss >> timestampUnix;
                    }
                    // std::cout << data[0] << " " << data[1] << " " << data[2] << " " << data[3] << " " << data[4] << " " << data[5] << " " << data[6] << std::endl;
                    if (data[0] > 0 && imuId == imuToUse)
                    {
                        FusionVector gyr;
                        gyr.axis.x = data[1];
                        gyr.axis.y = data[2];
                        gyr.axis.z = data[3];

                        FusionVector acc;
                        acc.axis.x = data[4];
                        acc.axis.y = data[5];
                        acc.axis.z = data[6];

                        all_data.emplace_back(std::pair(data[0] / 1e9, timestampUnix / 1e9), gyr, acc);
                    }
                }
                myfile.close();
            }
        }
    }
    catch (...)
    {
        std::cout << "load_imu error for file: '" << imu_file << "'" << std::endl;
        //return all_data;
    }

    return all_data;
}

std::vector<Point3Di> load_point_cloud(const std::string &lazFile, bool ommit_points_with_timestamp_equals_zero, double filter_threshold_xy_inner, double filter_threshold_xy_outer, const std::unordered_map<int, Eigen::Affine3d> &calibrations)
{
    std::vector<Point3Di> points;
    laszip_POINTER laszip_reader;
    if (laszip_create(&laszip_reader))
    {
        fprintf(stderr, "DLL ERROR: creating laszip reader\n");
        std::abort();
    }

    laszip_BOOL is_compressed = 0;
    if (laszip_open_reader(laszip_reader, lazFile.c_str(), &is_compressed))
    {
        fprintf(stderr, "DLL ERROR: opening laszip reader for '%s'\n", lazFile.c_str());
        std::abort();
    }
    // std::cout << "compressed : " << is_compressed << std::endl;
    laszip_header *header;

    if (laszip_get_header_pointer(laszip_reader, &header))
    {
        fprintf(stderr, "DLL ERROR: getting header pointer from laszip reader\n");
        std::abort();
    }
    // fprintf(stderr, "file '%s' contains %u points\n", lazFile.c_str(), header->number_of_point_records);
    laszip_point *point;
    if (laszip_get_point_pointer(laszip_reader, &point))
    {
        fprintf(stderr, "DLL ERROR: getting point pointer from laszip reader\n");
        std::abort();
    }

    int counter_ts0 = 0;
    int counter_filtered_points = 0;

    for (laszip_U32 j = 0; j < header->number_of_point_records; j++)
    {

        if (laszip_read_point(laszip_reader))
        {
            fprintf(stderr, "DLL ERROR: reading point %u\n", j);
            laszip_close_reader(laszip_reader);
            return points;
            // std::abort();
        }

        Point3Di p;
        int id = point->user_data;

        if (!calibrations.empty())
        {
            if (!calibrations.contains(id))
            {
                continue;
            }
        }

        Eigen::Affine3d calibration = calibrations.empty() ? Eigen::Affine3d::Identity() : calibrations.at(id);
        const Eigen::Vector3d pf(header->x_offset + header->x_scale_factor * static_cast<double>(point->X), header->y_offset + header->y_scale_factor * static_cast<double>(point->Y), header->z_offset + header->z_scale_factor * static_cast<double>(point->Z));

        p.point = calibration * (pf);
        p.lidarid = id;
        p.timestamp = point->gps_time;
        p.intensity = point->intensity;

        // add z correction
        // if (p.point.z() > 0)
        //{
        //    double dist = sqrt(p.point.x() * p.point.x() + p.point.y() * p.point.y());
        //    double correction = dist * asin(0.08 / 10.0);

        //    p.point.z() += correction;
        //}
        /*if (p.point.z() > 0)
        {
            double dist = sqrt(p.point.x() * p.point.x() + p.point.y() * p.point.y());
            double correction = 0;//dist * asin(0.08 / 10.0);

            if (dist < 11.0){
                correction = 0.005;
            }else{
                correction = -0.015;
            }

            p.point.z() += correction;
        }*/

        if (p.timestamp == 0 && ommit_points_with_timestamp_equals_zero)
        {
            counter_ts0++;
        }
        else
        {
            /* underground mining
            if (sqrt(pf.x() * pf.x()) < 4.5 && sqrt(pf.y() * pf.y()) < 2){
                counter_filtered_points++;
            }else{


                points.emplace_back(p);
            }
            */

            if (sqrt(pf.x() * pf.x() + pf.y() * pf.y()) > filter_threshold_xy_inner && sqrt(pf.x() * pf.x() + pf.y() * pf.y()) < filter_threshold_xy_outer)
            {
                points.emplace_back(p);
            }
            else
            {
                counter_filtered_points++;
            }
        }
    }

    std::cout << header->number_of_point_records << " - " << counter_filtered_points << " = " << points.size();
    if (counter_ts0 > 0)
    {
        std::cout << " (points with 0 timestamp: " << counter_ts0 << ")";
    }
    std::cout << std::endl;

    laszip_close_reader(laszip_reader);
    return points;
}

std::unordered_map<int, std::string> MLvxCalib::GetIdToSnMapping(const std::string &filename)
{
    if (!std::filesystem::exists(filename))
    {
        std::cout << "!std::filesystem::exists(filename) '" << filename << "'" << std::endl;
        return std::unordered_map<int, std::string>();
    }
    std::unordered_map<int, std::string> dataMap;
    std::ifstream fst(filename);
    std::string line;
    while (std::getline(fst, line))
    {
        std::istringstream iss(line);
        int key;
        std::string value;

        if (iss >> key >> value)
        {
            dataMap[key] = value;
        }
        else
        {
            std::cerr << "Failed to parse line: " << line << std::endl;
        }
    }
    return dataMap;
}

std::unordered_map<std::string, Eigen::Affine3d> MLvxCalib::GetCalibrationFromFile(const std::string &filename)
{
    if (!std::filesystem::exists(filename))
    {
        return std::unordered_map<std::string, Eigen::Affine3d>();
    }
    std::unordered_map<std::string, Eigen::Affine3d> dataMap;
    std::ifstream file(filename);

    using json = nlohmann::json;
    json jsonData = json::parse(file);

    // Iterate through the JSON object and parse each value into Eigen::Affine3d

    for (auto &calibrationEntry : jsonData["calibration"].items())
    {
        const std::string &lidarSn = calibrationEntry.key();
        Eigen::Matrix4d value;
        // std::cout << "lidarSn : " << lidarSn << std::endl;

        if (calibrationEntry.value().contains("identity"))
        {
            std::string identity = calibrationEntry.value()["identity"].get<std::string>();
            // std::cout << "identity : " << identity << std::endl;
            std::transform(identity.begin(), identity.end(), identity.begin(), ::toupper);
            if (identity == "TRUE")
            {
                dataMap[lidarSn] = Eigen::Matrix4d::Identity();
                continue;
                continue;
            }
        }

        assert(calibrationEntry.value().contains("data"));
        const auto matrixRawData = calibrationEntry.value()["data"];
        assert(matrixRawData.size() == 16);
        // Populate the Eigen::Affine3d matrix from the JSON array
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                value(i, j) = matrixRawData[i * 4 + j]; // default is column-major order
            }
        }

        if (calibrationEntry.value().contains("order"))
        {
            std::string order = calibrationEntry.value()["order"].get<std::string>();
            // std::cout << "order : " << order << std::endl;
            std::transform(order.begin(), order.end(), order.begin(), ::toupper);
            if (order == "COLUMN")
            {
                Eigen::Matrix4d valueT = value.transpose();
                value = valueT;
            }
        }

        if (calibrationEntry.value().contains("inverted"))
        {
            std::string inverted = calibrationEntry.value()["inverted"].get<std::string>();
            // std::cout << "inverted : " << inverted << std::endl;
            std::transform(inverted.begin(), inverted.end(), inverted.begin(), ::toupper);
            if (inverted == "TRUE")
            {
                Eigen::Matrix4d valueI = value.inverse();
                value = valueI;
            }
        }

        Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

        // std::cout << "Calibration for " << lidarSn << std::endl;
        // std::cout << value.format(HeavyFmt) << std::endl;
        //  Insert into the map
        dataMap[lidarSn] = value;
    }
    // check for blacklisted
    if (jsonData.contains("blacklist"))
    {
        json blacklist = jsonData["blacklist"];
        for (const auto &item : blacklist)
        {
            std::string blacklistedSn = item.get<std::string>();
            dataMap[blacklistedSn] = Eigen::Matrix4d::Zero();
        }
    }
    return dataMap;
}

std::string MLvxCalib::GetImuSnToUse(const std::string &filename)
{
    if (!std::filesystem::exists(filename))
    {
        return "";
    }
    std::unordered_map<std::string, Eigen::Affine3d> dataMap;
    std::ifstream file(filename);

    using json = nlohmann::json;
    json jsonData = json::parse(file);

    return jsonData["imuToUse"];
}

std::unordered_map<int, Eigen::Affine3d> MLvxCalib::CombineIntoCalibration(const std::unordered_map<int, std::string> &idToSn, const std::unordered_map<std::string, Eigen::Affine3d> &calibration)
{
    if (calibration.empty())
    {
        return std::unordered_map<int, Eigen::Affine3d>();
    }
    std::unordered_map<int, Eigen::Affine3d> dataMap;
    for (const auto &[id, sn] : idToSn)
    {
        // std::cout << "XXX: "<< id << " " << sn << std::endl;
        const auto &affine = calibration.at(sn);
        dataMap[id] = affine;
    }
    return dataMap;
}

int MLvxCalib::GetImuIdToUse(const std::unordered_map<int, std::string> &idToSn, const std::string &snToUse)
{
    if (snToUse.empty() || idToSn.empty())
    {
        std::cout << "snToUse.empty() || idToSn.empty()" << std::endl;
        std::cout << "(int)snToUse.empty()" << (int)snToUse.empty() << std::endl;
        std::cout << "(int)idToSn.empty()" << (int)idToSn.empty() << std::endl;
        std::cout << __FILE__ << " " << __LINE__ << std::endl;
        return 0;
    }
    for (const auto &[id, sn] : idToSn)
    {
        // std::cout << "snToUse " << snToUse << " sn " << sn << std::endl;
        if (snToUse == sn)
        {
            return id;
        }
    }
    return 0;
}

fs::path get_next_result_path(const std::string working_directory)
{
    std::regex pattern(R"(lio_result_(\d+))");
    int max_number = -1;
    for (const auto &entry : fs::directory_iterator(working_directory))
    {
        if (entry.is_directory())
        {
            std::smatch match;
            std::string folder_name = entry.path().filename().string();

            if (std::regex_match(folder_name, match, pattern))
            {
                int folder_number = std::stoi(match[1].str());
                max_number = std::max(max_number, folder_number);
            }
        }
    }
    return (working_directory / fs::path("lio_result_" + std::to_string(max_number + 1)));
}

bool loadLaz(const std::string &filename, std::vector<Point3Di> &points_out, std::vector<int> index_poses_i, std::vector<Eigen::Affine3d> &intermediate_trajectory, const Eigen::Affine3d &m_pose)
{
    if (!std::filesystem::exists(filename))
    {
        std::cerr << "File does not exist: " << filename << std::endl;
        return false;
    }

    laszip_POINTER laszip_reader;
    if (laszip_create(&laszip_reader))
    {
        std::cerr << "DLL ERROR: creating laszip reader\n";
        return false;
    }

    laszip_BOOL is_compressed = 0;
    if (laszip_open_reader(laszip_reader, filename.c_str(), &is_compressed))
    {
        std::cerr << "ERROR: cannot open LAZ file: " << filename << std::endl;
        return false;
    }

    laszip_header *header;
    if (laszip_get_header_pointer(laszip_reader, &header))
    {
        std::cerr << "DLL ERROR: getting header pointer from laszip reader\n";
        return false;
    }

    laszip_I64 num_points = (header->number_of_point_records ? header->number_of_point_records : header->extended_number_of_point_records);

    laszip_point *point;
    if (laszip_get_point_pointer(laszip_reader, &point))
    {
        std::cerr << "DLL ERROR: getting point pointer from laszip reader\n";
        return false;
    }

    for (laszip_I64 i = 0; i < num_points; i++)
    {
        if (laszip_read_point(laszip_reader))
        {
            std::cerr << "DLL ERROR: reading point " << i << "\n";
            return false;
        }
        double coordinates[3];
        if (laszip_get_coordinates(laszip_reader, coordinates))
        {
            std::cerr << "DLL ERROR: getting coordinates\n";
            return false;
        }
        double x = coordinates[0];
        double y = coordinates[1];
        double z = coordinates[2];
        Point3Di p;
        p.point = intermediate_trajectory[index_poses_i[i]].inverse() * m_pose * Eigen::Vector3d(x, y, z);
        p.timestamp = point->gps_time * 1e-9;
        p.intensity = point->intensity;
        p.index_pose = index_poses_i[i];
        points_out.push_back(p);
    }

    if (laszip_close_reader(laszip_reader))
    {
        std::cerr << "DLL ERROR: closing reader\n";
        return false;
    }

    if (laszip_destroy(laszip_reader))
    {
        std::cerr << "DLL ERROR: destroying reader\n";
        return false;
    }

    return true;
}

bool load_poses(const fs::path &poses_file, std::vector<Eigen::Affine3d> &out_poses)
{
    std::ifstream infile(poses_file);
    if (!infile.is_open())
        return false;

    int N = 0;
    infile >> N;
    out_poses.resize(N);

    std::string filename_dummy;
    for (int i = 0; i < N; ++i)
    {
        std::getline(infile >> std::ws, filename_dummy);
        Eigen::Matrix4d mat;
        for (int r = 0; r < 4; ++r)
        {
            infile >> mat(r, 0) >> mat(r, 1) >> mat(r, 2) >> mat(r, 3);
        }
        out_poses[i] = Eigen::Affine3d(mat);
    }
    return true;
}

bool load_trajectory_csv(const std::string &filename, const Eigen::Affine3d &m_pose,
                         std::vector<std::pair<double, double>> &intermediate_trajectory_timestamps,
                         std::vector<Eigen::Affine3d> &intermediate_trajectory,
                         std::vector<Eigen::Vector3d> &imu_om_fi_ka)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open trajectory file: " << filename << std::endl;
        return false;
    }

    std::string line;
    std::getline(file, line); // skip header
    while (std::getline(file, line))
    {
        std::istringstream ss(line);
        double ts1, ts2;
        double p00, p01, p02, p03;
        double p10, p11, p12, p13;
        double p20, p21, p22, p23;
        double imu_x, imu_y, imu_z;

        ss >> ts1 >> p00 >> p01 >> p02 >> p03 >> p10 >> p11 >> p12 >> p13 >> p20 >> p21 >> p22 >> p23 >> ts2 >> imu_x >> imu_y >> imu_z;

        Eigen::Matrix4d rel_mat;
        rel_mat << p00, p01, p02, p03,
            p10, p11, p12, p13,
            p20, p21, p22, p23,
            0, 0, 0, 1;

        Eigen::Affine3d relative_pose(rel_mat);
        Eigen::Affine3d global_pose = m_pose * relative_pose;

        intermediate_trajectory_timestamps.emplace_back(ts1 * 1e-9, ts2 * 1e-9);
        intermediate_trajectory.push_back(global_pose);
        imu_om_fi_ka.emplace_back(imu_x, imu_y, imu_z);
    }

    return true;
}

bool load_point_sizes(const std::filesystem::path &path, std::vector<int> &vector)
{
    std::ifstream in_file(path);
    if (!in_file)
    {
        std::cerr << "Failed to open index poses file: " << path << std::endl;
        return false;
    }
    nlohmann::json j;
    try
    {
        in_file >> j;
        if (!j.is_array())
        {
            std::cerr << "Invalid format: top-level JSON element is not an array in " << path << std::endl;
            return false;
        }
        vector.clear();
        for (const auto &val : j)
        {
            if (!val.is_number_integer())
            {
                std::cerr << "Invalid value: index_pose is not an integer.\n";
                return false;
            }
            vector.push_back(val.get<int>());
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error parsing index poses JSON: " << e.what() << std::endl;
        return false;
    }
    return true;
}

bool load_index_poses(const std::filesystem::path &path, std::vector<std::vector<int>> &index_poses_out)
{
    std::ifstream in_file(path);
    if (!in_file)
    {
        std::cerr << "Failed to open index poses file: " << path << std::endl;
        return false;
    }
    nlohmann::json j;
    try
    {
        in_file >> j;
        if (!j.is_array())
        {
            std::cerr << "Invalid format: top-level JSON element is not an array in " << path << std::endl;
            return false;
        }
        index_poses_out.clear();
        int tmp = 0;
        for (const auto &chunk : j)
        {
            if (!chunk.is_array())
            {
                std::cerr << "Invalid format: one of the chunks is not an array.\n";
                return false;
            }
            std::vector<int> chunk_indices;
            for (const auto &val : chunk)
            {
                if (!val.is_number_integer())
                {
                    std::cerr << "Invalid value: index_pose is not an integer.\n";
                    return false;
                }
                chunk_indices.push_back(val.get<int>());
            }
            index_poses_out.push_back(std::move(chunk_indices));
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error parsing index poses JSON: " << e.what() << std::endl;
        return false;
    }
    return true;
}

bool load_worker_data_from_results(const fs::path &session_file, std::vector<WorkerData> &worker_data_out)
{
#if 0
    std::ifstream f(session_file);
    if (!f.is_open()) {
        std::cerr << "Cannot open session file: " << session_file << std::endl;
        return false;
    }
    nlohmann::json jj;
    f >> jj;
    f.close();
    if (!jj.contains("Session Settings") || !jj.contains("laz_file_names")) {
        std::cerr << "Missing required session data!" << std::endl;
        return false;
    }
    auto j = jj["Session Settings"];
    auto laz_file_names = jj["laz_file_names"];  
    auto threshold_nr_poses = j["threshold_nr_poses"].get<int>();
    auto poses_path = j["poses_file_name"].get<std::string>();
    auto index_poses_path = j["index_poses_path"].get<std::string>();
    auto point_sizes_path = j["point_sizes_path"].get<std::string>();
    auto decimation = j["decimation"].get<double>();
    
    std::vector<Eigen::Affine3d> m_poses;
    if (!load_poses(poses_path, m_poses))
    {
        std::cerr << "Failed to load poses from " << poses_path << std::endl;
        return false;
    }

    std::vector<std::vector<int>> index_poses;
    if (!load_index_poses(index_poses_path, index_poses))
    {
        std::cerr << "Failed to load poses from " << poses_path << std::endl;
        return false;
    }
    std::vector<int> point_sizes;
    if (!load_point_sizes(point_sizes_path, point_sizes))
    {
        std::cerr << "Failed to load point sizes from " << poses_path << std::endl;
        return false;
    }
    std::vector<WorkerData> concatenated_worker_data;
    int i = 0;
    for (const auto& entry : laz_file_names)
    {
        if (!entry.contains("file_name"))
        {
            std::cerr << "Malformed entry in laz_file_names.\n";
            return false;
        }
        
        fs::path laz_path = entry["file_name"];
        fs::path base_dir = laz_path.parent_path();
        std::string base_name = laz_path.stem().string();
        std::string index_str = base_name.substr(base_name.find_last_of('_') + 1);
        std::string traj_name = "trajectory_lio_" + index_str + ".csv";
        fs::path traj_path = base_dir / traj_name;

        WorkerData wd;
        if (!load_trajectory_csv(traj_path.string(), m_poses[i],
                                 wd.intermediate_trajectory_timestamps,
                                 wd.intermediate_trajectory,
                                 wd.imu_om_fi_ka))
        {
            std::cerr << "Failed to load trajectory from " << traj_path << std::endl;
            return false;
        }
        if (!loadLaz(laz_path.string(), wd.original_points, index_poses[i], wd.intermediate_trajectory, m_poses[i]))
        {
            std::cerr << "Failed to load laz from " << laz_path << std::endl;
            return false;
        }
        concatenated_worker_data.push_back(wd);
        i++;
    }
    size_t concat_idx = 0;
    for (auto wd : concatenated_worker_data)  
    {
        size_t offset = 0;
        size_t points_offset = 0;
        int count = wd.intermediate_trajectory.size(); 
        int points_count = wd.original_points.size(); 
        while (count > 0)
        {
            int to_copy = std::min(threshold_nr_poses, count);
            WorkerData chunk_wd;
            chunk_wd.intermediate_trajectory.assign(wd.intermediate_trajectory.begin() + offset,
                                                    wd.intermediate_trajectory.begin() + offset + to_copy);
            chunk_wd.intermediate_trajectory_timestamps.assign(wd.intermediate_trajectory_timestamps.begin() + offset,
                                                               wd.intermediate_trajectory_timestamps.begin() + offset + to_copy);
            chunk_wd.imu_om_fi_ka.assign(wd.imu_om_fi_ka.begin() + offset,
                                         wd.imu_om_fi_ka.begin() + offset + to_copy);
            int to_copy_points = std::min(point_sizes[concat_idx++], points_count);
            chunk_wd.original_points.assign(wd.original_points.begin() + points_offset,
                                            wd.original_points.begin() + points_offset + to_copy_points);
            for (auto& p: chunk_wd.original_points)
            {
                p.index_pose -= offset;
            }
            offset += to_copy;
            count -= to_copy;
            points_offset += to_copy_points;
            points_count -= to_copy_points;
            chunk_wd.intermediate_points = decimate(chunk_wd.original_points, decimation, decimation, decimation);
            worker_data_out.push_back(chunk_wd);
        }
    }
    for (auto wd: worker_data_out)
    {
        int max_index_pose = 0;
        for (auto p : wd.original_points)
        {
            max_index_pose =  max_index_pose >  p.index_pose ? max_index_pose : p.index_pose;
        }
    }
    return true;
#endif
    return false;
}

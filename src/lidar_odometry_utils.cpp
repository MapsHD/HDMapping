#include "lidar_odometry_utils.h"
#include <filesystem>
#include "csv.hpp"

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

Eigen::Matrix4d getInterpolatedPose(const std::map<double, Eigen::Matrix4d> &trajectory, double query_time)
{
    Eigen::Matrix4d ret(Eigen::Matrix4d::Zero());
    auto it_lower = trajectory.lower_bound(query_time);
    auto it_next = it_lower;

    if (it_lower == trajectory.begin())
    {
        return ret;
    }
    if (it_lower->first > query_time)
    {
        it_lower = std::prev(it_lower);
    }
    if (it_lower == trajectory.begin())
    {
        return ret;
    }
    if (it_lower == trajectory.end())
    {
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
        ret = Eigen::Matrix4d::Identity();
        ret.col(3).head<3>() = it_next->second.col(3).head<3>();
        ret.topLeftCorner(3, 3) = it_lower->second.topLeftCorner(3, 3);
        return ret;
    }
    if (std::fabs(difft1) < 0.15 && std::fabs(difft2) < 0.15)
    {
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

std::vector<Point3Di> decimate(const std::vector<Point3Di> &points, double bucket_x, double bucket_y, double bucket_z)
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

    std::sort(ip.begin(), ip.end(), [](const PointCloud::PointBucketIndexPair &a, const PointCloud::PointBucketIndexPair &b)
              { return a.index_of_bucket < b.index_of_bucket; });

    for (int i = 1; i < ip.size(); i++)
    {
        // std::cout << ip[i].index_of_bucket << " ";
        if (ip[i - 1].index_of_bucket != ip[i].index_of_bucket)
        {
            out.emplace_back(points[ip[i].index_of_point]);
        }
    }
    // std::cout << "points.size after decimation: " << out.size() << std::endl;
    return out;
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
            }

            if (this_bucket.number_of_points == 3)
            {
                this_bucket.cov = this_bucket.cov * (this_bucket.number_of_points - 1) / this_bucket.number_of_points +
                                  cov_update * (this_bucket.number_of_points - 1) / (this_bucket.number_of_points * this_bucket.number_of_points);

                // calculate normal vector
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

bool saveLaz(const std::string &filename, const WorkerData &data)
{
    constexpr float scale = 0.0001f; // one tenth of milimeter
    // find max
    double max_x{std::numeric_limits<double>::lowest()};
    double max_y{std::numeric_limits<double>::lowest()};
    double max_z{std::numeric_limits<double>::lowest()};
    double min_x = 1000000000000.0;
    double min_y = 1000000000000.0;
    double min_z = 1000000000000.0;

    std::vector<Point3Di> points;
    Eigen::Affine3d m_pose = data.intermediate_trajectory[0].inverse();
    for (const auto &org_p : data.original_points)
    {
        Point3Di p = org_p;
        p.point = m_pose * (data.intermediate_trajectory[org_p.index_pose] * org_p.point);
        points.push_back(p);
    }

    for (auto &p : points)
    {
        if (p.point.x() < min_x)
        {
            min_x = p.point.x();
        }
        if (p.point.x() > max_x)
        {
            max_x = p.point.x();
        }

        if (p.point.y() < min_y)
        {
            min_y = p.point.y();
        }
        if (p.point.y() > max_y)
        {
            max_y = p.point.y();
        }

        if (p.point.z() < min_z)
        {
            min_z = p.point.z();
        }
        if (p.point.z() > max_z)
        {
            max_z = p.point.z();
        }
    }

    std::cout << "processing: " << filename << "points " << points.size() << std::endl;

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
    header->number_of_point_records = points.size();
    header->number_of_points_by_return[0] = points.size();
    header->number_of_points_by_return[1] = 0;
    header->point_data_record_length = 28;
    header->x_scale_factor = scale;
    header->y_scale_factor = scale;
    header->z_scale_factor = scale;

    header->max_x = max_x;
    header->min_x = min_x;
    header->max_y = max_y;
    header->min_y = min_y;
    header->max_z = max_z;
    header->min_z = min_z;

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

    for (int i = 0; i < points.size(); i++)
    {

        const auto &p = points[i];
        point->intensity = p.intensity;
        point->gps_time = p.timestamp * 1e9;
        // point->user_data = 0;//p.line_id;
        // point->classification = p.point.tag;
        p_count++;
        coordinates[0] = p.point.x();
        coordinates[1] = p.point.y();
        coordinates[2] = p.point.z();
        if (laszip_set_coordinates(laszip_writer, coordinates))
        {
            fprintf(stderr, "DLL ERROR: setting coordinates for point %I64d\n", p_count);
            return false;
        }

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

bool saveLaz(const std::string &filename, const std::vector<Point3Di> &points_global)
{

    constexpr float scale = 0.0001f; // one tenth of milimeter
    // find max
    double max_x{std::numeric_limits<double>::lowest()};
    double max_y{std::numeric_limits<double>::lowest()};
    double max_z{std::numeric_limits<double>::lowest()};
    double min_x = 1000000000000.0;
    double min_y = 1000000000000.0;
    double min_z = 1000000000000.0;

    for (auto &p : points_global)
    {
        if (p.point.x() < min_x)
        {
            min_x = p.point.x();
        }
        if (p.point.x() > max_x)
        {
            max_x = p.point.x();
        }

        if (p.point.y() < min_y)
        {
            min_y = p.point.y();
        }
        if (p.point.y() > max_y)
        {
            max_y = p.point.y();
        }

        if (p.point.z() < min_z)
        {
            min_z = p.point.z();
        }
        if (p.point.z() > max_z)
        {
            max_z = p.point.z();
        }
    }

    std::cout << "processing: " << filename << "points " << points_global.size() << std::endl;

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
    header->number_of_point_records = points_global.size();
    header->number_of_points_by_return[0] = points_global.size();
    header->number_of_points_by_return[1] = 0;
    header->point_data_record_length = 28;
    header->x_scale_factor = scale;
    header->y_scale_factor = scale;
    header->z_scale_factor = scale;

    header->max_x = max_x;
    header->min_x = min_x;
    header->max_y = max_y;
    header->min_y = min_y;
    header->max_z = max_z;
    header->min_z = min_z;

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

    for (int i = 0; i < points_global.size(); i++)
    {
        const auto &p = points_global[i];
        point->intensity = p.intensity;
        point->gps_time = p.timestamp * 1e9;
        // std::cout << p.timestamp << std::endl;
        //  point->user_data = 0;//p.line_id;
        //  point->classification = p.point.tag;
        p_count++;
        coordinates[0] = p.point.x();
        coordinates[1] = p.point.y();
        coordinates[2] = p.point.z();
        if (laszip_set_coordinates(laszip_writer, coordinates))
        {
            fprintf(stderr, "DLL ERROR: setting coordinates for point %I64d\n", p_count);
            return false;
        }

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

bool save_poses(const std::string file_name, std::vector<Eigen::Affine3d> m_poses, std::vector<std::string> filenames)
{
    std::ofstream outfile;
    outfile.open(file_name);
    if (!outfile.good())
    {
        std::cout << "can not save file: " << file_name << std::endl;
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

// this function draws ellipse for each bucket
void draw_ellipse(const Eigen::Matrix3d &covar, const Eigen::Vector3d &mean, Eigen::Vector3f color, float nstd)
{
    Eigen::LLT<Eigen::Matrix<double, 3, 3>> cholSolver(covar);
    Eigen::Matrix3d transform = cholSolver.matrixL();

    const double pi = 3.141592;
    const double di = 0.02;
    const double dj = 0.04;
    const double du = di * 2 * pi;
    const double dv = dj * pi;
    glColor3f(color.x(), color.y(), color.z());

    for (double i = 0; i < 1.0; i += di) // horizonal
    {
        for (double j = 0; j < 1.0; j += dj) // vertical
        {
            double u = i * 2 * pi;     // 0     to  2pi
            double v = (j - 0.5) * pi; //-pi/2 to pi/2

            const Eigen::Vector3d pp0(cos(v) * cos(u), cos(v) * sin(u), sin(v));
            const Eigen::Vector3d pp1(cos(v) * cos(u + du), cos(v) * sin(u + du), sin(v));
            const Eigen::Vector3d pp2(cos(v + dv) * cos(u + du), cos(v + dv) * sin(u + du), sin(v + dv));
            const Eigen::Vector3d pp3(cos(v + dv) * cos(u), cos(v + dv) * sin(u), sin(v + dv));
            Eigen::Vector3d tp0 = transform * (nstd * pp0) + mean;
            Eigen::Vector3d tp1 = transform * (nstd * pp1) + mean;
            Eigen::Vector3d tp2 = transform * (nstd * pp2) + mean;
            Eigen::Vector3d tp3 = transform * (nstd * pp3) + mean;

            glBegin(GL_LINE_LOOP);
            glVertex3dv(tp0.data());
            glVertex3dv(tp1.data());
            glVertex3dv(tp2.data());
            glVertex3dv(tp3.data());
            glEnd();
        }
    }
}

std::vector<std::tuple<double, FusionVector, FusionVector>> load_imu(const std::string& imu_file, int imuToUse)
{
    std::vector<std::tuple<double, FusionVector, FusionVector>> all_data;

    csv::CSVFormat format;
    format.delimiter({' ', ',', '\t'});

    csv::CSVReader reader(imu_file, format);
    const auto columns = reader.get_col_names();
    const std::set<std::string> columnsSet(columns.begin(), columns.end());
    
    //mandatory columns
    const bool hasTsColumn = columnsSet.contains("timestamp");
    const bool hasGyrosColumns = columnsSet.contains("gyroX")&& columnsSet.contains("gyroY") && columnsSet.contains("gyroZ");
    const bool hasAccsColumns = columnsSet.contains("accX") && columnsSet.contains("accY") && columnsSet.contains("accZ");

    //optional
    const bool hasImuIdColumn = columnsSet.contains("imuId");
    const bool hasUnixTimestampColumn = columnsSet.contains("timestampUnix");

    //check if legacy
    bool is_legacy = true;
    if (hasTsColumn)
    {
        is_legacy = false;
        if (!hasAccsColumns && !hasGyrosColumns)
        {
            std::cerr << "Input csv file is missing one of the mandatory columns :\n";
            std::cerr << "timestamp,gyroX,gyroY,gyroZ,accX,accY,accZ";
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
                int imu_id = row["imuId"].get<int>();
            }
            if (imu_id < 0 || imuToUse == imu_id)
            {
                double timestamp = row["timestamp"].get<double>();
                FusionVector gyr;
                gyr.axis.x = row["gyroX"].get<double>();
                gyr.axis.y = row["gyroY"].get<double>();
                gyr.axis.z = row["gyroZ"].get<double>();
                FusionVector acc;
                acc.axis.x = row["accX"].get<double>();
                acc.axis.y = row["accY"].get<double>();
                acc.axis.z = row["accZ"].get<double>();
                all_data.emplace_back(timestamp / 1e9, gyr, acc);
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
                int imuId = 0;
                std::string line;
                std::getline(myfile, line);
                std::istringstream iss(line);
                iss >> data[0] >> data[1] >> data[2] >> data[3] >> data[4] >> data[5] >> data[6];
                if (!iss.eof())
                {
                    iss >> imuId;
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

                    all_data.emplace_back(data[0] / 1e9, gyr, acc);
                }
            }
            myfile.close();
        }


    }

    return all_data;
}

std::vector<Point3Di> load_point_cloud(const std::string &lazFile, bool ommit_points_with_timestamp_equals_zero, double filter_threshold_xy, const std::unordered_map<int, Eigen::Affine3d>& calibrations)
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
    std::cout << "compressed : " << is_compressed << std::endl;
    laszip_header *header;

    if (laszip_get_header_pointer(laszip_reader, &header))
    {
        fprintf(stderr, "DLL ERROR: getting header pointer from laszip reader\n");
        std::abort();
    }
    fprintf(stderr, "file '%s' contains %u points\n", lazFile.c_str(), header->number_of_point_records);
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

            if (sqrt(pf.x() * pf.x() + pf.y() * pf.y()) > filter_threshold_xy)
            {
                points.emplace_back(p);
            }
            else
            {
                counter_filtered_points++;
            }
        }
    }

    std::cout << "number points with ts == 0: " << counter_ts0 << std::endl;
    std::cout << "counter_filtered_points: " << counter_filtered_points << std::endl;
    std::cout << "total number points: " << points.size() << std::endl;
    laszip_close_reader(laszip_reader);
    return points;
}

std::unordered_map<int, std::string> MLvxCalib::GetIdToSnMapping(const std::string& filename)
{
    if (!std::filesystem::exists(filename))
    {
        return std::unordered_map<int, std::string>();
    }
    std::unordered_map<int, std::string> dataMap;
    std::ifstream fst(filename);
    std::string line;
    while (std::getline(fst, line)) {
        std::istringstream iss(line);
        int key;
        std::string value;

        if (iss >> key >> value) {
            dataMap[key] = value;
        }
        else {
            std::cerr << "Failed to parse line: " << line << std::endl;
        }
    }
    return dataMap;
}

std::unordered_map<std::string, Eigen::Affine3d> MLvxCalib::GetCalibrationFromFile(const std::string& filename)
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

    for (auto& calibrationEntry : jsonData["calibration"].items()) {
        const std::string& lidarSn = calibrationEntry.key();
        Eigen::Matrix4d value;
        std::cout << "lidarSn : " << lidarSn << std::endl;

        if (calibrationEntry.value().contains("identity"))
        {
            std::string identity = calibrationEntry.value()["identity"].get<std::string>();
            std::cout << "identity : " << identity << std::endl;
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
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                value(i, j) = matrixRawData[i * 4 + j]; // default is column-major order
            }
        }

        if (calibrationEntry.value().contains("order"))
        {
            std::string order = calibrationEntry.value()["order"].get<std::string>();
            std::cout << "order : " << order << std::endl;
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
            std::cout << "inverted : " << inverted << std::endl;
            std::transform(inverted.begin(), inverted.end(), inverted.begin(), ::toupper);
            if (inverted == "TRUE")
            {
                Eigen::Matrix4d valueI = value.inverse();
                value = valueI;
            }
        }

        Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

        std::cout << "Calibration for " << lidarSn << std::endl;
        std::cout << value.format(HeavyFmt) << std::endl;
        // Insert into the map
        dataMap[lidarSn] = value;
    }
    // check for blacklisted
    if (jsonData.contains("blacklist"))
    {
        json blacklist = jsonData["blacklist"];
        for (const auto& item : blacklist)
        {
            std::string blacklistedSn = item.get<std::string>();
            dataMap[blacklistedSn] = Eigen::Matrix4d::Zero();
        }

    }
    return dataMap;
}


std::string MLvxCalib::GetImuSnToUse(const std::string& filename)
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

std::unordered_map<int, Eigen::Affine3d> MLvxCalib::CombineIntoCalibration(const std::unordered_map<int, std::string>& idToSn, const std::unordered_map<std::string, Eigen::Affine3d>& calibration)
{
    if (calibration.empty())
    {
        return std::unordered_map<int, Eigen::Affine3d>();
    }
    std::unordered_map<int, Eigen::Affine3d> dataMap;
    for (const auto& [id, sn] : idToSn)
    {
        const auto& affine = calibration.at(sn);
        dataMap[id] = affine;
    }
    return dataMap;
}

int MLvxCalib::GetImuIdToUse(const std::unordered_map<int, std::string>& idToSn, const std::string& snToUse )
{
    if (snToUse.empty() || idToSn.empty()) {
        return 0;
    }
    for (const auto& [id, sn] : idToSn)
    {
        if (snToUse == sn)
        {
            return id;
        }
    }
    return 0;
}


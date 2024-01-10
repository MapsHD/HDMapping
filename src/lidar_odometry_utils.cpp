#include "lidar_odometry_utils.h"

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

std::vector<std::tuple<double, FusionVector, FusionVector>> load_imu(const std::string &imu_file)
{
    std::vector<std::tuple<double, FusionVector, FusionVector>> all_data;
    std::ifstream myfile(imu_file);
    if (myfile.is_open())
    {
        while (myfile)
        {
            double data[7];
            myfile >> data[0] >> data[1] >> data[2] >> data[3] >> data[4] >> data[5] >> data[6];
            // std::cout << data[0] << " " << data[1] << " " << data[2] << " " << data[3] << " " << data[4] << " " << data[5] << " " << data[6] << std::endl;
            if (data[0] > 0)
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
    return all_data;
}
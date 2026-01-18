#pragma once

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include <filesystem>

struct PerspectiveCameraParams
{
    double cx;
    double cy;
    double fx;
    double fy;
};

struct MetricCameraParams
{
    double c;
    double ksi_0;
    double eta_0;
};

struct TaitBryanPose
{
    double px = 0.0;
    double py = 0.0;
    double pz = 0.0;
    double om = 0.0;
    double fi = 0.0;
    double ka = 0.0;
};

struct RodriguesPose
{
    double px;
    double py;
    double pz;
    double sx;
    double sy;
    double sz;
};

struct QuaternionPose
{
    double px;
    double py;
    double pz;
    double q0;
    double q1;
    double q2;
    double q3;
};

struct Point3D
{
    float x;
    float y;
    float z;
    int index_pose;
};

struct Point3Di
{
    Eigen::Vector3d point;
    double timestamp;
    float intensity;
    int index_pose;
    uint8_t lidarid;
    int index_point;
};

// Point3Di POD (Plain Old Data) variant
// Used for safe binary I/O via read()/write() or memcpy
// Contains only trivially copyable members
#pragma pack(push, 1)
struct Point3DiDisk
{
    double x, y, z;
    double timestamp;
    float intensity;
    int32_t index_pose;
    uint8_t lidarid;
    int32_t index_point;
};
#pragma pack(pop)

struct Point
{
    double x = 0.0f;
    double y = 0.0f;
    double z = 0.0f;
    uint32_t intensity = 0;
    double time = 0.0;
};

struct PointMesh
{
    Eigen::Vector3d coordinates;
    Eigen::Vector3d normal_vector;
    int index_pose;
};

struct ChunkFile
{
    int filename;
    double time_begin_inclusive;
    double time_end_inclusive;
};

// Extra measures are needed when writting/reading non-trivial types
// Point3Di containing Eigen::Vector3d to avoid:
//- structure alignment & padding written to file
//- unreadable file on another build
//- undefined behavior if read back via raw load

inline void convert_to_disk(const Point3Di& src, Point3DiDisk& dst)
{
    dst.x = src.point.x();
    dst.y = src.point.y();
    dst.z = src.point.z();
    dst.timestamp = src.timestamp;
    dst.intensity = src.intensity;
    dst.index_pose = src.index_pose;
    dst.lidarid = src.lidarid;
    dst.index_point = src.index_point;
}

inline void convert_from_disk(const Point3DiDisk& src, Point3Di& dst)
{
    dst.point = Eigen::Vector3d(src.x, src.y, src.z);
    dst.timestamp = src.timestamp;
    dst.intensity = src.intensity;
    dst.index_pose = src.index_pose;
    dst.lidarid = src.lidarid;
    dst.index_point = src.index_point;
}

/*template<typename T>
inline bool save_vector_data(const std::string& file_name, std::vector<T>& vector_data) {
        std::ofstream ofs(file_name, std::ios::binary);
        if (!ofs.good()) {
                return false;
        }
        ofs.write(reinterpret_cast<char*>(vector_data.data()), vector_data.size()* sizeof(T));
        return true;
}*/

template<typename T>
bool save_vector_data(const std::string& file_name, const std::vector<T>& out)
{
    static_assert(std::is_trivially_copyable_v<T> || std::is_same_v<T, Point3Di>, "Binary loading only supported for POD or Point3Di");

    std::ofstream file(file_name, std::ios::binary);
    if (!file)
        return false;

    if constexpr (std::is_trivially_copyable_v<T>)
        // direct write for POD
        return file.write(reinterpret_cast<const char*>(out.data()), out.size() * sizeof(T)).good();
    else
    {
        // convert to disk type first
        std::vector<Point3DiDisk> disk(out.size());
        for (size_t i = 0; i < out.size(); ++i)
            convert_to_disk(out[i], disk[i]);

        return file.write(reinterpret_cast<const char*>(disk.data()), disk.size() * sizeof(Point3DiDisk)).good();
    }
}

/*template<typename T>
inline bool load_vector_data(const std::string& file_name, std::vector<T>& vector_data) {
        std::basic_ifstream<char> vd_str(file_name, std::ios::binary);
        if (!vd_str.good()) {
                return false;
        }
        std::vector<char> data((std::istreambuf_iterator<char>(vd_str)), std::istreambuf_iterator<char>());
        std::vector<T> v(data.size() / sizeof(T));
        memcpy(v.data(), data.data(), data.size());
        vector_data = v;
        return true;
}*/

/*template<typename T>
bool load_vector_data(const std::string& file_name, std::vector<T>& out)
{
        //static_assert(std::is_trivially_copyable_v<T>,
        //	"T must be trivially copyable");

        std::ifstream file(file_name, std::ios::binary | std::ios::ate);
        if (!file)
                return false;

        const std::streamsize size = file.tellg();
        if (size < 0 || size % sizeof(T) != 0)
                return false;

        const size_t count = size / sizeof(T);
        out.resize(count);

        file.seekg(0, std::ios::beg);
        if (!file.read(reinterpret_cast<char*>(out.data()), size))
                return false;

        return true;
}*/

template<typename T>
bool load_vector_data(const std::string& file_name, std::vector<T>& out)
{
    static_assert(std::is_trivially_copyable_v<T> || std::is_same_v<T, Point3Di>, "Binary loading only supported for POD or Point3Di");

    std::ifstream file(file_name, std::ios::binary | std::ios::ate);
    if (!file)
        return false;

    const std::streamsize size = file.tellg();
    if (size < 0)
        return false;

    file.seekg(0, std::ios::beg);

    if constexpr (std::is_trivially_copyable_v<T>)
    {
        if (size % sizeof(T) != 0)
            return false;

        const size_t count = size / sizeof(T);
        out.resize(count);

        return file.read(reinterpret_cast<char*>(out.data()), size).good();
    }
    else
    {
        // Non-trivial type path (Point3Di)
        if (size % sizeof(Point3DiDisk) != 0)
            return false;

        const size_t count = size / sizeof(Point3DiDisk);
        std::vector<Point3DiDisk> disk(count);

        if (!file.read(reinterpret_cast<char*>(disk.data()), size))
            return false;

        out.resize(count);
        for (size_t i = 0; i < count; ++i)
            convert_from_disk(disk[i], out[i]);

        return true;
    }
}

struct LaserBeam
{
    Eigen::Vector3d position;
    Eigen::Vector3d direction;
    float distance;
    float range;
};

struct Plane
{
    Plane()
    {
        a = b = c = d = 0.0;
    }
    double a;
    double b;
    double c;
    double d;

    Plane(Eigen::Vector3d p, Eigen::Vector3d nv)
    {
        a = nv.x();
        b = nv.y();
        c = nv.z();
        d = -a * p.x() - b * p.y() - c * p.z();
    }
};

struct CommonData
{
    Eigen::Vector3d roi = Eigen::Vector3d(0, 0, 0);
    float roi_size = 50;
    float rotate_x = 0.0, rotate_y = 0.0;
    float translate_x, translate_y = 0.0;
    float translate_z = -5000;
    bool is_ortho = true;
    bool roi_exorter = false;
    bool laz_wrapper = false;
    bool single_trajectory_viewer = false;
    bool odo_with_gnss_fusion = false;
    float shift_x = 0.0;
    float shift_y = 0.0;
};

struct PointCloudWithPose
{
    bool visible = true;
    double timestamp;
    std::string trajectory_file_name;
    Eigen::Affine3d m_pose;
    std::vector<Point> points_global;
    int point_size = 1;
};

struct Job
{
    uint64_t index_begin_inclusive;
    uint64_t index_end_exclusive;
};

inline std::vector<Job> get_jobs(uint64_t size, int num_threads)
{
    int hc = size / num_threads;
    if (hc < 1)
        hc = 1;

    std::vector<Job> jobs;
    for (uint64_t i = 0; i < size; i += hc)
    {
        uint64_t sequence_length = hc;
        if (i + hc >= size)
        {
            sequence_length = size - i;
        }
        if (sequence_length == 0)
            break;

        Job j;
        j.index_begin_inclusive = i;
        j.index_end_exclusive = i + sequence_length;
        jobs.push_back(j);
    }

    return jobs;
}

struct LAZPoint
{
    double x = 0.0f;
    double y = 0.0f;
    double z = 0.0f;
    // uint16_t r;
    // uint16_t g;
    // uint16_t b;
    float r;
    float g;
    float b;
    uint8_t classsification;
    double timestamp;
    // unsigned short RGB[4];
};

struct LAZSector
{
    std::string file_name;
    std::vector<LAZPoint> point_cloud;
    double min_x;
    double max_x;
    double min_y;
    double max_y;
    bool visible = true;
    int point_size = 1;
};

struct ConstraintToGeoreference
{
    std::string trajectory_file_name;
    double time_stamp;
    Eigen::Affine3d m_pose;
};

struct ROIwithConstraints
{
    std::vector<ConstraintToGeoreference> constraints;
};

struct Node
{
    double timestamp = 0.0;
    Eigen::Affine3d m_pose = Eigen::Affine3d::Identity();
    int index_to_lidar_odometry_odo = -1;
    int index_to_gnss = -1;
};

struct BetweenNode
{
    Node node_outer;
    std::vector<Node> nodes_between;
    float color_x = 0;
    float color_y = 0;
    float color_z = 0;
};

struct GeoPoint
{
    std::string name;
    Eigen::Vector3d coordinates;
    bool choosen = false;
    double w_x = 1000000.0;
    double w_y = 1000000.0;
    double w_z = 1000000.0;
};

struct WorkerData
{
    // std::vector<Point3Di> intermediate_points;
    std::filesystem::path intermediate_points_cache_file_name;
    // std::vector<Point3Di> original_points_to_save;
    std::filesystem::path original_points_cache_file_name;
    std::filesystem::path original_points_to_save_cache_file_name;
    std::vector<Eigen::Affine3d> intermediate_trajectory;
    std::vector<Eigen::Affine3d> intermediate_trajectory_motion_model;
    std::vector<std::pair<double, double>> intermediate_trajectory_timestamps;
    std::vector<Eigen::Vector3d> imu_om_fi_ka;
    bool show = false;
};

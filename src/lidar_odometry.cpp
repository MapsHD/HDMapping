#include <portable-file-dialogs.h>

#include <laszip/laszip_api.h>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <Fusion.h>
#include <map>
#include <execution>

#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <ImGuizmo.h>
#include <imgui_internal.h>

#include <glew.h>
#include <GL/freeglut.h>

#include <structures.h>
#include <ndt.h>

#include <transformations.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian.h>
#include <python-scripts/constraints/relative_pose_tait_bryan_wc_jacobian.h>
#include <chrono>
#include <python-scripts/constraints/smoothness_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian_simplified.h>
#include <python-scripts/constraints/constraint_fixed_parameter_jacobian.h>
#include <common/include/cauchy.h>
#include <python-scripts/point-to-feature-metrics/point_to_line_tait_bryan_wc_jacobian.h>
//

#define SAMPLE_PERIOD (1.0 / 200.0)
// #define NR_ITER 100
namespace fs = std::filesystem;

std::vector<Eigen::Vector3d> all_points;
std::vector<Point3Di> initial_points;
NDT ndt;

NDT::GridParameters in_out_params;

// std::vector<NDT::PointBucketIndexPair> index_pair;
// std::vector<NDT::Bucket> buckets;
using NDTBucketMapType = std::unordered_map<uint64_t, NDT::Bucket>;
NDTBucketMapType buckets;
NDTBucketMapType reference_buckets;
bool show_reference_buckets = true;

std::vector<Point3Di>
    reference_points;
bool show_reference_points = false;
int dec_reference_points = 100;

Eigen::Matrix4d getInterpolatedPose(const std::map<double, Eigen::Matrix4d> &trajectory, double query_time);
std::vector<Point3Di> decimate(const std::vector<Point3Di> &points, double bucket_x, double bucket_y, double bucket_z);
void update_rgd(NDT::GridParameters &rgd_params, NDTBucketMapType &buckets,
                std::vector<Point3Di> &points_global);

bool show_all_points = false;
bool show_initial_points = true;
bool show_trajectory = true;
bool show_trajectory_as_axes = false;
bool show_covs = false;
int dec_covs = 10;
double filter_threshold_xy = 0.5;
int nr_iter = 100;
double sliding_window_trajectory_length_threshold = 50.0;

bool fusionConventionNwu = true;
bool fusionConventionEnu = false;
bool fusionConventionNed = false;
bool use_motion_from_previous_step = true;
bool useMultithread = true;
struct WorkerData
{
    std::vector<Point3Di> intermediate_points;
    std::vector<Point3Di> original_points;
    std::vector<Eigen::Affine3d> intermediate_trajectory;
    std::vector<Eigen::Affine3d> intermediate_trajectory_motion_model;
    std::vector<double> intermediate_trajectory_timestamps;
    std::vector<std::pair<double, double>> imu_roll_pitch;
    bool show = false;
};

std::vector<WorkerData> worker_data;

float rotate_x = 0.0, rotate_y = 0.0;
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
std::string working_directory_preview = "";
double decimation = 0.1;
int threshold_initial_points = 100000;
bool initial_transformation_gizmo = false;

float m_gizmo[] = {1, 0, 0, 0,
                   0, 1, 0, 0,
                   0, 0, 1, 0,
                   0, 0, 0, 1};
Eigen::Affine3d m_g = Eigen::Affine3d::Identity();
double consecutive_distance = 0.0;

unsigned long long int get_index(const int16_t x, const int16_t y, const int16_t z)
{
    return ((static_cast<unsigned long long int>(x) << 32) & (0x0000FFFF00000000ull)) |
           ((static_cast<unsigned long long int>(y) << 16) & (0x00000000FFFF0000ull)) |
           ((static_cast<unsigned long long int>(z) << 0) & (0x000000000000FFFFull));
}

unsigned long long int get_rgd_index(const Eigen::Vector3d p, const Eigen::Vector3d b)
{
    int16_t x = static_cast<int16_t>(p.x() / b.x());
    int16_t y = static_cast<int16_t>(p.y() / b.y());
    int16_t z = static_cast<int16_t>(p.z() / b.z());
    return get_index(x, y, z);
}

std::vector<std::tuple<double, FusionVector, FusionVector>> load_imu(const std::string &imu_file);
std::vector<Point3Di> load_point_cloud(const std::string &lazFile, bool ommit_points_with_timestamp_equals_zero = true);
void optimize(std::vector<Point3Di> &intermediate_points, std::vector<Eigen::Affine3d> &intermediate_trajectory,
              std::vector<Eigen::Affine3d> &intermediate_trajectory_motion_model,
              NDT::GridParameters &rgd_params, NDTBucketMapType &buckets, bool useMultithread,
              bool add_pitch_roll_constraint, const std::vector<std::pair<double, double>> &imu_roll_pitch);
void align_to_reference(NDT::GridParameters &rgd_params, std::vector<Point3Di> &initial_points, Eigen::Affine3d &m_g, NDTBucketMapType &buckets);
void fix_ptch_roll(std::vector<WorkerData> &worker_data);

void draw_ellipse(const Eigen::Matrix3d &covar, const Eigen::Vector3d &mean, Eigen::Vector3f color, float nstd = 3)
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

void lidar_odometry_gui()
{
    if (ImGui::Begin("lidar_odometry_gui v0.19"))
    {
        ImGui::Text(("Working directory: " + working_directory).c_str());
        ImGui::Checkbox("show_all_points", &show_all_points);
        ImGui::Checkbox("show_initial_points", &show_initial_points);
        ImGui::Checkbox("show_trajectory", &show_trajectory);
        ImGui::SameLine();
        ImGui::Checkbox("show_trajectory_as_axes", &show_trajectory_as_axes);
        // ImGui::Checkbox("show_covs", &show_covs);
        ImGui::InputDouble("normal distributions transform bucket size X", &in_out_params.resolution_X);
        if (in_out_params.resolution_X < 0.2)
        {
            in_out_params.resolution_X = 0.2;
        }
        ImGui::InputDouble("normal distributions transform bucket size Y", &in_out_params.resolution_Y);
        if (in_out_params.resolution_Y < 0.2)
        {
            in_out_params.resolution_Y = 0.2;
        }
        ImGui::InputDouble("normal distributions transform bucket size Z", &in_out_params.resolution_Z);
        if (in_out_params.resolution_Z < 0.2)
        {
            in_out_params.resolution_Z = 0.2;
        }

        ImGui::InputDouble("filter_threshold_xy (all local points inside lidar xy_circle radius[m] will be removed)", &filter_threshold_xy);

        ImGui::InputDouble("decimation (larger value of decimation better performance, but worse accuracy)", &decimation);
        ImGui::InputInt("number iterations", &nr_iter);
        ImGui::InputDouble("sliding window trajectory length threshold", &sliding_window_trajectory_length_threshold);
        ImGui::InputInt("threshold initial points", &threshold_initial_points);

        ImGui::Checkbox("fusionConventionNwu", &fusionConventionNwu);
        ImGui::Checkbox("use_multithread", &useMultithread);
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

        ImGui::Checkbox("use_motion_from_previous_step", &use_motion_from_previous_step);

        if (ImGui::Button("load data"))
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

            if (input_file_names.size() > 0 && laz_files.size() == csv_files.size())
            {
                working_directory = fs::path(input_file_names[0]).parent_path().string();
                fs::path wdp = fs::path(input_file_names[0]).parent_path();
                wdp /= "preview";
                if (!fs::exists(wdp))
                {
                    fs::create_directory(wdp);
                }

                working_directory_preview = wdp.string();

                for (size_t i = 0; i < input_file_names.size(); i++)
                {
                    std::cout << input_file_names[i] << std::endl;
                }
                std::cout << "loading imu" << std::endl;
                std::vector<std::tuple<double, FusionVector, FusionVector>> imu_data;

                std::for_each(std::begin(csv_files), std::end(csv_files), [&imu_data](const std::string &fn)
                              {
                    auto imu = load_imu(fn.c_str());
                    std::cout << fn << std::endl;
                    imu_data.insert(std::end(imu_data), std::begin(imu), std::end(imu)); });

                std::cout << "loading points" << std::endl;
                std::vector<std::vector<Point3Di>> pointsPerFile;
                pointsPerFile.resize(laz_files.size());

                std::transform(std::execution::par_unseq, std::begin(laz_files), std::end(laz_files), std::begin(pointsPerFile), [](const std::string &fn)
                               {
                                   return load_point_cloud(fn.c_str());
                                   // std::unique_lock lck(mutex);

                                   // std::cout << fn << std::endl;
                                   //
                               });

                std::vector<Point3Di> points;
                for (const auto &pp : pointsPerFile)
                {
                    points.insert(std::end(points), std::begin(pp), std::end(pp));
                }

                //
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
                for (const auto &[timestamp, gyr, acc] : imu_data)
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

                    trajectory[timestamp] = t.matrix();
                    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
                    printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f [%d of %d]\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw, counter++, imu_data.size());
                }

                std::cout << "number of points: " << points.size() << std::endl;
                std::cout << "start transforming points" << std::endl;

                counter = 1; // ToDo make it faster
                for (auto &p : points)
                {
                    Eigen::Matrix4d t = getInterpolatedPose(trajectory, p.timestamp);
                    if (!t.isZero())
                    {
                        Eigen::Affine3d tt(t);
                        Eigen::Vector3d tp = tt * p.point;
                        all_points.push_back(tp);
                    }
                    if (counter % 1000000 == 0)
                    {
                        printf("tranform point %d of %d \n", counter, points.size());
                    }
                    counter++;
                }

                for (int i = 0; i < threshold_initial_points; i++)
                {
                    auto p = points[i];
                    // p.point = all_points[i];
                    initial_points.push_back(p);
                }

                double timestamp_begin = points[threshold_initial_points - 1].timestamp;
                std::cout << "timestamp_begin: " << timestamp_begin << std::endl;

                std::vector<double> timestamps;
                std::vector<Eigen::Affine3d> poses;
                for (const auto &t : trajectory)
                {
                    if (t.first >= timestamp_begin)
                    {
                        timestamps.push_back(t.first);
                        Eigen::Affine3d m;
                        m.matrix() = t.second;
                        poses.push_back(m);
                    }
                }

                std::cout << "poses.size(): " << poses.size() << std::endl;

                int thershold = 20;
                WorkerData wd;
                // std::vector<double> temp_ts;
                // temp_ts.reserve(1000000);

                // int last_point = 0;
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
                        auto index_lower = std::lower_bound(points.begin(), points.end(), wd.intermediate_trajectory_timestamps[0],
                                                            [](Point3Di lhs, double rhs) -> bool
                                                            { return lhs.timestamp < rhs; });
                        unsigned long long int i_begin = std::distance(points.begin(), index_lower);

                        auto index_upper = std::lower_bound(points.begin(), points.end(), wd.intermediate_trajectory_timestamps[wd.intermediate_trajectory_timestamps.size() - 1],
                                                            [](Point3Di lhs, double rhs) -> bool
                                                            { return lhs.timestamp < rhs; });
                        unsigned long long int i_end = std::distance(points.begin(), index_upper);

                        for (unsigned long long int k = i_begin; k < i_end; k++)
                        {
                            if (points[k].timestamp > wd.intermediate_trajectory_timestamps[0] && points[k].timestamp < wd.intermediate_trajectory_timestamps[wd.intermediate_trajectory_timestamps.size() - 1])
                            {
                                auto p = points[k];
                                auto lower = std::lower_bound(wd.intermediate_trajectory_timestamps.begin(), wd.intermediate_trajectory_timestamps.end(), p.timestamp);
                                p.index_pose = std::distance(wd.intermediate_trajectory_timestamps.begin(), lower);
                                wd.intermediate_points.emplace_back(p);
                                wd.original_points.emplace_back(p);
                            }
                        }

                        if (decimation > 0.0)
                        {
                            wd.intermediate_points = decimate(wd.intermediate_points, decimation, decimation, decimation);
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
                m_g = worker_data[0].intermediate_trajectory[0];
            }
            else
            {
                std::cout << "please select files correctly" << std::endl;
            }
        }
        if (ImGui::Button("compute_all"))
        {
            if (worker_data.size() != 0)
            {

                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();
                double acc_distance = 0.0;
                std::vector<Point3Di> points_global;

                Eigen::Affine3d m_last = m_g;
                auto tmp = worker_data[0].intermediate_trajectory;

                worker_data[0].intermediate_trajectory[0] = m_last;
                for (int k = 1; k < tmp.size(); k++)
                {
                    Eigen::Affine3d m_update = tmp[k - 1].inverse() * tmp[k];
                    m_last = m_last * m_update;
                    worker_data[0].intermediate_trajectory[k] = m_last;
                }
                worker_data[0].intermediate_trajectory_motion_model = worker_data[0].intermediate_trajectory;

                auto pp = initial_points;
                for (int i = 0; i < pp.size(); i++)
                {
                    pp[i].point = m_g * pp[i].point;
                }
                update_rgd(in_out_params, buckets, pp);

                for (int i = 0; i < worker_data.size(); i++)
                {
                    //std::cout << "computing worker_data [" << i + 1 << "] of " << worker_data.size() << " acc_distance: " << acc_distance << std::endl;
                    Eigen::Vector3d mean_shift(0.0, 0.0, 0.0);
                    if (i > 1 && use_motion_from_previous_step)
                    {
                        Eigen::Affine3d m_relative = worker_data[i - 2].intermediate_trajectory[worker_data[i - 2].intermediate_trajectory.size() - 1].inverse() *
                                                     worker_data[i - 1].intermediate_trajectory[0];

                        mean_shift /= (worker_data[i].intermediate_trajectory.size());

                        if (mean_shift.norm() > 0.1)
                        {
                            mean_shift = Eigen::Vector3d(0.1, 0.1, 0.1);
                        }

                        Eigen::Affine3d m_mean_shift = Eigen::Affine3d::Identity();
                        m_mean_shift.translation() = mean_shift;

                        std::vector<Eigen::Affine3d> new_trajectory;
                        Eigen::Affine3d current_node = worker_data[i].intermediate_trajectory[0];
                        new_trajectory.push_back(current_node);

                        for (int tr = 1; tr < worker_data[i].intermediate_trajectory.size(); tr++)
                        {
                            current_node = current_node * (worker_data[i].intermediate_trajectory[tr - 1].inverse() * worker_data[i].intermediate_trajectory[tr]);
                            current_node = current_node * m_mean_shift;
                            new_trajectory.push_back(current_node);
                        }

                        worker_data[i].intermediate_trajectory = new_trajectory;
                        ////////////////////////////////////////////////////////////////////////
                        std::vector<Eigen::Affine3d> new_trajectory_motion_model;
                        Eigen::Affine3d current_node_motion_model = worker_data[i].intermediate_trajectory_motion_model[0];
                        new_trajectory_motion_model.push_back(current_node_motion_model);

                        for (int tr = 1; tr < worker_data[i].intermediate_trajectory_motion_model.size(); tr++)
                        {
                            current_node_motion_model = current_node_motion_model * (worker_data[i].intermediate_trajectory_motion_model[tr - 1].inverse() * worker_data[i].intermediate_trajectory_motion_model[tr]);
                            current_node_motion_model = current_node_motion_model * m_mean_shift;
                            new_trajectory_motion_model.push_back(current_node_motion_model);
                        }

                        worker_data[i].intermediate_trajectory_motion_model = new_trajectory_motion_model;
                    }

                    bool add_pitch_roll_constraint = false;
                    TaitBryanPose pose;
                    pose = pose_tait_bryan_from_affine_matrix(worker_data[i].intermediate_trajectory[0]);

                    double residual1;
                    double residual2;
                    residual_constraint_fixed_optimization_parameter(residual1, normalize_angle(worker_data[i].imu_roll_pitch[0].first), normalize_angle(pose.om));
                    residual_constraint_fixed_optimization_parameter(residual2, normalize_angle(worker_data[i].imu_roll_pitch[0].second), normalize_angle(pose.fi));

                    if (fabs(worker_data[i].imu_roll_pitch[0].first) < 30.0 / 180.0 * M_PI && fabs(worker_data[i].imu_roll_pitch[0].second) < 30.0 / 180.0 * M_PI)
                    {
                        if (consecutive_distance > 10.0)
                        {
                            add_pitch_roll_constraint = true;
                            consecutive_distance = 0.0;
                        }
                    }

                    if (add_pitch_roll_constraint)
                    {
                        std::cout << "residual_imu_roll_deg before: " << residual1 / M_PI * 180.0 << std::endl;
                        std::cout << "residual_imu_pitch_deg before: " << residual2 / M_PI * 180.0 << std::endl;
                    }

                    std::chrono::time_point<std::chrono::system_clock> start1, end1;
                    start1 = std::chrono::system_clock::now();

                    for (int iter = 0; iter < nr_iter; iter++)
                    {
                        optimize(worker_data[i].intermediate_points, worker_data[i].intermediate_trajectory, worker_data[i].intermediate_trajectory_motion_model,
                                 in_out_params, buckets, useMultithread, add_pitch_roll_constraint, worker_data[i].imu_roll_pitch);
                    }
                    end1 = std::chrono::system_clock::now();
                    std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
                    std::cout << "optimizing worker_data [" << i + 1 << "] of " << worker_data.size() << " acc_distance: " << acc_distance << " elapsed time: " << elapsed_seconds1.count() << std::endl;

                    if (add_pitch_roll_constraint)
                    {
                        pose = pose_tait_bryan_from_affine_matrix(worker_data[i].intermediate_trajectory[0]);

                        residual_constraint_fixed_optimization_parameter(residual1, normalize_angle(worker_data[i].imu_roll_pitch[0].first), normalize_angle(pose.om));
                        residual_constraint_fixed_optimization_parameter(residual2, normalize_angle(worker_data[i].imu_roll_pitch[0].second), normalize_angle(pose.fi));

                        std::cout << "residual_imu_roll_deg after: " << residual1 / M_PI * 180.0 << std::endl;
                        std::cout << "residual_imu_pitch_deg after: " << residual2 / M_PI * 180.0 << std::endl;
                    }

                    // align to reference
                    if (reference_points.size() > 0)
                    {
                        std::cout << "align to reference" << std::endl;
                        Eigen::Affine3d m_first = worker_data[i].intermediate_trajectory[0];
                        Eigen::Affine3d m_first_inv = m_first.inverse();

                        // create rigid scan
                        std::vector<Point3Di> local_points;
                        for (int k = 0; k < worker_data[i].intermediate_points.size(); k++)
                        {
                            Point3Di p = worker_data[i].intermediate_points[k];
                            int index_pose = p.index_pose;
                            p.point = worker_data[i].intermediate_trajectory[index_pose] * p.point;
                            p.point = m_first_inv * p.point;
                            local_points.push_back(p);
                        }
                        // std::cout << "before " << m_first.matrix() << std::endl;
                        if (decimation > 0)
                        {
                            local_points = decimate(local_points, decimation, decimation, decimation);
                        }
                        for (int iter = 0; iter < nr_iter; iter++)
                        {
                            align_to_reference(in_out_params, local_points, m_first, reference_buckets);
                        }

                        auto tmp = worker_data[i].intermediate_trajectory;

                        worker_data[i].intermediate_trajectory[0] = m_first;

                        for (int k = 1; k < tmp.size(); k++)
                        {
                            Eigen::Affine3d m_update = tmp[k - 1].inverse() * tmp[k];
                            m_first = m_first * m_update;
                            worker_data[i].intermediate_trajectory[k] = m_first;
                        }
                        worker_data[i].intermediate_trajectory_motion_model = worker_data[i].intermediate_trajectory;
                    }

                    // temp save
                    if (i % 100 == 0)
                    {
                        std::vector<Point3Di> global_points;
                        for (int k = 0; k < worker_data[i].intermediate_points.size(); k++)
                        {
                            Point3Di p = worker_data[i].intermediate_points[k];
                            int index_pose = p.index_pose;
                            p.point = worker_data[i].intermediate_trajectory[index_pose] * p.point;
                            global_points.push_back(p);
                        }
                        std::string fn = working_directory_preview + "/temp_point_cloud_" + std::to_string(i) + ".laz";
                        saveLaz(fn.c_str(), global_points);
                    }
                    acc_distance += ((worker_data[i].intermediate_trajectory[0].inverse()) *
                                     worker_data[i].intermediate_trajectory[worker_data[i].intermediate_trajectory.size() - 1])
                                        .translation()
                                        .norm();

                    // update
                    for (int j = i + 1; j < worker_data.size(); j++)
                    {
                        Eigen::Affine3d m_last = worker_data[j - 1].intermediate_trajectory[worker_data[j - 1].intermediate_trajectory.size() - 1];
                        auto tmp = worker_data[j].intermediate_trajectory;

                        worker_data[j].intermediate_trajectory[0] = m_last;
                        for (int k = 1; k < tmp.size(); k++)
                        {
                            Eigen::Affine3d m_update = tmp[k - 1].inverse() * tmp[k];
                            m_last = m_last * m_update;
                            worker_data[j].intermediate_trajectory[k] = m_last;
                        }
                    }

                    for (int j = 0; j < worker_data[i].intermediate_points.size(); j++)
                    {
                        Point3Di pp = worker_data[i].intermediate_points[j];
                        pp.point = worker_data[i].intermediate_trajectory[worker_data[i].intermediate_points[j].index_pose] * pp.point;
                        points_global.push_back(pp);
                    }

                    // if(reference_points.size() == 0){
                    if (acc_distance > sliding_window_trajectory_length_threshold)
                    {
                        std::chrono::time_point<std::chrono::system_clock> startu, endu;
                        startu = std::chrono::system_clock::now();

                        if (reference_points.size() == 0)
                        {
                            buckets.clear();
                        }

                        std::vector<Point3Di> points_global_new;
                        points_global_new.reserve(points_global.size() / 2 + 1);
                        for (int k = points_global.size() / 2; k < points_global.size(); k++)
                        {
                            points_global_new.emplace_back(points_global[k]);
                        }

                        acc_distance = 0;
                        points_global = points_global_new;

                        // decimate
                        if (decimation > 0)
                        {
                            decimate(points_global, decimation, decimation, decimation);
                        }
                        update_rgd(in_out_params, buckets, points_global);
                        //
                        endu = std::chrono::system_clock::now();

                        std::chrono::duration<double> elapsed_secondsu = endu - startu;
                        std::time_t end_timeu = std::chrono::system_clock::to_time_t(endu);

                        std::cout << "finished computation at " << std::ctime(&end_timeu)
                                  << "elapsed time update: " << elapsed_secondsu.count() << "s\n";
                        // std::cout << "update" << std::endl;
                    }
                    else
                    {
                        std::vector<Point3Di> pg;
                        for (int j = 0; j < worker_data[i].intermediate_points.size(); j++)
                        {
                            Point3Di pp = worker_data[i].intermediate_points[j];
                            pp.point = worker_data[i].intermediate_trajectory[worker_data[i].intermediate_points[j].index_pose] * pp.point;
                            pg.push_back(pp);
                        }
                        update_rgd(in_out_params, buckets, pg);
                    }

                    if (i > 1)
                    {
                        double translation = (worker_data[i - 1].intermediate_trajectory[0].translation() -
                                              worker_data[i - 2].intermediate_trajectory[0].translation())
                                                 .norm();
                        consecutive_distance += translation;
                        // std::cout << "consecutive_distance " << consecutive_distance << std::endl;
                    }
                    //}
                }

                for (int i = 0; i < worker_data.size(); i++)
                {
                    worker_data[i].intermediate_trajectory_motion_model = worker_data[i].intermediate_trajectory;
                }

                end = std::chrono::system_clock::now();

                std::chrono::duration<double> elapsed_seconds = end - start;
                std::time_t end_time = std::chrono::system_clock::to_time_t(end);

                std::cout << "finished computation at " << std::ctime(&end_time)
                          << "elapsed time: " << elapsed_seconds.count() << "s\n";
            }

            std::vector<Eigen::Affine3d> intermediate_trajectory;
            std::vector<Eigen::Affine3d> intermediate_trajectory_motion_model;
        }
        // if (ImGui::Button("fix pitch roll"))
        //{
        //     fix_ptch_roll(worker_data);
        // }
        if (ImGui::Button("save result"))
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
                // outfile <<
                for (int j = 0; j < worker_data_concatenated[i].intermediate_trajectory.size(); j++)
                {
                    auto pose = worker_data_concatenated[i].intermediate_trajectory[0].inverse() * worker_data_concatenated[i].intermediate_trajectory[j];

                    outfile
                        << std::setprecision(20) << worker_data_concatenated[i].intermediate_trajectory_timestamps[j] * 1e9 << " " << std::setprecision(10)
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
                        << pose(2, 3) << std::endl;
                }
                outfile.close();
                //
            }
            fs::path path(working_directory);
            path /= "lio_poses.reg";
            save_poses(path.string(), m_poses, file_names);
        }
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
                reference_buckets.clear();
                reference_points.clear();

                for (size_t i = 0; i < input_file_names.size(); i++)
                {
                    std::cout << "loading reference point cloud from: " << input_file_names[i] << std::endl;
                    auto pp = load_point_cloud(input_file_names[i].c_str(), false);
                    std::cout << "loaded " << pp.size() << " reference points" << std::endl;
                    reference_points.insert(std::end(reference_points), std::begin(pp), std::end(pp));
                }

                update_rgd(in_out_params, reference_buckets, reference_points);
                show_reference_points = true;
            }
        }

        ImGui::SameLine();
        ImGui::Checkbox("show reference points", &show_reference_points);
        ImGui::Checkbox("show reference buckets", &show_reference_buckets);
        ImGui::InputInt("decimation reference points", &dec_reference_points);

        if (initial_points.size() > 0)
        {
            ImGui::Text("-----manipulate initial transformation begin-------");
            ImGui::Checkbox("initial transformation gizmo", &initial_transformation_gizmo);

            if (initial_transformation_gizmo)
            {
                m_gizmo[0] = (float)m_g(0, 0);
                m_gizmo[1] = (float)m_g(1, 0);
                m_gizmo[2] = (float)m_g(2, 0);
                m_gizmo[3] = (float)m_g(3, 0);
                m_gizmo[4] = (float)m_g(0, 1);
                m_gizmo[5] = (float)m_g(1, 1);
                m_gizmo[6] = (float)m_g(2, 1);
                m_gizmo[7] = (float)m_g(3, 1);
                m_gizmo[8] = (float)m_g(0, 2);
                m_gizmo[9] = (float)m_g(1, 2);
                m_gizmo[10] = (float)m_g(2, 2);
                m_gizmo[11] = (float)m_g(3, 2);
                m_gizmo[12] = (float)m_g(0, 3);
                m_gizmo[13] = (float)m_g(1, 3);
                m_gizmo[14] = (float)m_g(2, 3);
                m_gizmo[15] = (float)m_g(3, 3);
            }
            if (!initial_transformation_gizmo)
            {
                if (ImGui::Button("Align to reference"))
                {
                    for (int i = 0; i < 30; i++)
                    {
                        align_to_reference(in_out_params, initial_points, m_g, reference_buckets);
                    }
                }
            }
            ImGui::Text("-----manipulate initial transformation end---------");
        }
        // show_trajectory

        static int current_scan = -1;
        int prev = current_scan;
        ImGui::InputInt("change_current_scan", &current_scan);
        {
            int curr = current_scan;

            if (prev != curr)
            {
                for (int k = 0; k < worker_data.size(); k++)
                {
                    worker_data[k].show = false;
                }
            }
            if (current_scan <= 0)
            {
                current_scan = 0;
            }
            if (current_scan >= worker_data.size())
            {
                current_scan = worker_data.size();
            }
            if (current_scan >= 0 && current_scan < worker_data.size())
            {
                worker_data[current_scan].show = true;
            }
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

        if (ImGui::Button("filter reference buckets"))
        {
            NDTBucketMapType reference_buckets_out;
            for (const auto &b : reference_buckets)
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
            reference_buckets = reference_buckets_out;
        }

        for (int i = 0; i < worker_data.size(); i++)
        {
            std::string text = "show[" + std::to_string(i) + "]";
            ImGui::Checkbox(text.c_str(), &worker_data[i].show);
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
            rotate_x += dy * 0.2f * mouse_sensitivity;
            rotate_y += dx * 0.2f * mouse_sensitivity;
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

    reshape((GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);
    glTranslatef(translate_x, translate_y, translate_z);
    glRotatef(rotate_x, 1.0, 0.0, 0.0);
    glRotatef(rotate_y, 0.0, 0.0, 1.0);

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

    if (show_all_points)
    {
        glColor3d(1.0, 0.0, 0.0);
        glBegin(GL_POINTS);
        for (const auto &p : all_points)
        {
            glVertex3d(p.x(), p.y(), p.z());
        }
        glEnd();
    }
    if (show_initial_points)
    {
        glColor3d(0.0, 1.0, 0.0);
        glBegin(GL_POINTS);
        for (const auto &p : initial_points)
        {
            auto pp = m_g * p.point;
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
        for (const auto &b : buckets)
        {
            draw_ellipse(b.second.cov, b.second.mean, Eigen::Vector3f(0.0f, 0.0f, 1.0f), 3);
        }
    }

    for (int i = 0; i < worker_data.size(); i++)
    {
        if (worker_data[i].show)
        {
            glPointSize(2);
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

            glBegin(GL_LINES);
            const auto &it = worker_data[i].intermediate_trajectory[0];
            glColor3f(1, 0, 0);
            glVertex3f(it(0, 3), it(1, 3), it(2, 3));
            glVertex3f(it(0, 3) + it(0, 0), it(1, 3) + it(1, 0), it(2, 3) + it(2, 0));

            glColor3f(0, 1, 0);
            glVertex3f(it(0, 3), it(1, 3), it(2, 3));
            glVertex3f(it(0, 3) + it(0, 1), it(1, 3) + it(1, 1), it(2, 3) + it(2, 1));

            glColor3f(0, 0, 1);
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
            }
        }
    }

    if (show_reference_points)
    {
        glColor3f(1, 0, 0);
        glBegin(GL_POINTS);
        for (int i = 0; i < reference_points.size(); i += dec_reference_points)
        {
            glVertex3f(reference_points[i].point.x(), reference_points[i].point.y(), reference_points[i].point.z());
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
        for (const auto &b : reference_buckets)
        {
            glVertex3f(b.second.mean.x(), b.second.mean.y(), b.second.mean.z());
        }
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

        m_g(0, 0) = m_gizmo[0];
        m_g(1, 0) = m_gizmo[1];
        m_g(2, 0) = m_gizmo[2];
        m_g(3, 0) = m_gizmo[3];
        m_g(0, 1) = m_gizmo[4];
        m_g(1, 1) = m_gizmo[5];
        m_g(2, 1) = m_gizmo[6];
        m_g(3, 1) = m_gizmo[7];
        m_g(0, 2) = m_gizmo[8];
        m_g(1, 2) = m_gizmo[9];
        m_g(2, 2) = m_gizmo[10];
        m_g(3, 2) = m_gizmo[11];
        m_g(0, 3) = m_gizmo[12];
        m_g(1, 3) = m_gizmo[13];
        m_g(2, 3) = m_gizmo[14];
        m_g(3, 3) = m_gizmo[15];
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

std::vector<Point3Di> load_point_cloud(const std::string &lazFile, bool ommit_points_with_timestamp_equals_zero)
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
            std::abort();
        }
        Point3Di p;

        double cal_x = 11.0 / 1000.0; // ToDo change if lidar differen than livox mid 360
        double cal_y = 23.29 / 1000.0;
        double cal_z = -44.12 / 1000.0;

        Eigen::Vector3d pf(header->x_offset + header->x_scale_factor * static_cast<double>(point->X), header->y_offset + header->y_scale_factor * static_cast<double>(point->Y), header->z_offset + header->z_scale_factor * static_cast<double>(point->Z));
        p.point.x() = pf.x() - cal_x;
        p.point.y() = pf.y() - cal_y;
        p.point.z() = pf.z() - cal_z;
        p.timestamp = point->gps_time;
        p.intensity = point->intensity;

        if (p.timestamp == 0 && ommit_points_with_timestamp_equals_zero)
        {
            counter_ts0++;
        }
        else
        {
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

int main(int argc, char *argv[])
{
    in_out_params.resolution_X = 0.3;
    in_out_params.resolution_Y = 0.3;
    in_out_params.resolution_Z = 0.3;
    in_out_params.bounding_box_extension = 20.0;

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

void optimize(std::vector<Point3Di> &intermediate_points, std::vector<Eigen::Affine3d> &intermediate_trajectory,
              std::vector<Eigen::Affine3d> &intermediate_trajectory_motion_model,
              NDT::GridParameters &rgd_params, NDTBucketMapType &buckets, bool multithread,
              bool add_pitch_roll_constraint, const std::vector<std::pair<double, double>> &imu_roll_pitch)
{
    std::vector<Eigen::Triplet<double>> tripletListA;
    std::vector<Eigen::Triplet<double>> tripletListP;
    std::vector<Eigen::Triplet<double>> tripletListB;

    Eigen::MatrixX<double> AtPAndt(intermediate_trajectory.size() * 6, intermediate_trajectory.size() * 6);
    AtPAndt.setZero();
    Eigen::MatrixX<double> AtPBndt(intermediate_trajectory.size() * 6, 1);
    AtPBndt.setZero();
    Eigen::Vector3d b(rgd_params.resolution_X, rgd_params.resolution_Y, rgd_params.resolution_Z);

    std::vector<std::mutex> mutexes(intermediate_trajectory.size());

    const auto hessian_fun = [&](const Point3Di &intermediate_points_i)
    {
        if (intermediate_points_i.point.norm() < 1.0)
        {
            return;
        }

        Eigen::Vector3d point_global = intermediate_trajectory[intermediate_points_i.index_pose] * intermediate_points_i.point;
        auto index_of_bucket = get_rgd_index(point_global, b);

        auto bucket_it = buckets.find(index_of_bucket);
        // no bucket found
        if (bucket_it == buckets.end())
        {
            return;
        }
        auto &this_bucket = bucket_it->second;

        // if(buckets[index_of_bucket].number_of_points >= 5){
        const Eigen::Matrix3d &infm = this_bucket.cov.inverse();
        const double threshold = 10000.0;

        if ((infm.array() > threshold).any())
        {
            return;
        }
        if ((infm.array() < -threshold).any())
        {
            return;
        }

        const Eigen::Affine3d &m_pose = intermediate_trajectory[intermediate_points_i.index_pose];
        const Eigen::Vector3d &p_s = intermediate_points_i.point;
        const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose);
        //

        Eigen::Matrix<double, 6, 6, Eigen::RowMajor> AtPA;
        point_to_point_source_to_target_tait_bryan_wc_AtPA_simplified(
            AtPA,
            pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
            p_s.x(), p_s.y(), p_s.z(),
            infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2));

        Eigen::Matrix<double, 6, 1> AtPB;
        point_to_point_source_to_target_tait_bryan_wc_AtPB_simplified(
            AtPB,
            pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
            p_s.x(), p_s.y(), p_s.z(),
            infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2),
            this_bucket.mean.x(), this_bucket.mean.y(), this_bucket.mean.z());

        int c = intermediate_points_i.index_pose * 6;

        std::mutex &m = mutexes[intermediate_points_i.index_pose];
        std::unique_lock lck(m);
        AtPAndt.block<6, 6>(c, c) += AtPA;
        AtPBndt.block<6, 1>(c, 0) -= AtPB;
    };

    if (multithread)
    {
        std::for_each(std::execution::par_unseq, std::begin(intermediate_points), std::end(intermediate_points), hessian_fun);
    }
    else
    {
        std::for_each(std::begin(intermediate_points), std::end(intermediate_points), hessian_fun);
    }
    std::vector<std::pair<int, int>> odo_edges;
    for (size_t i = 1; i < intermediate_trajectory.size(); i++)
    {
        odo_edges.emplace_back(i - 1, i);
    }

    std::vector<TaitBryanPose> poses;
    std::vector<TaitBryanPose> poses_desired;

    for (size_t i = 0; i < intermediate_trajectory.size(); i++)
    {
        poses.push_back(pose_tait_bryan_from_affine_matrix(intermediate_trajectory[i]));
    }
    for (size_t i = 0; i < intermediate_trajectory_motion_model.size(); i++)
    {
        poses_desired.push_back(pose_tait_bryan_from_affine_matrix(intermediate_trajectory_motion_model[i]));
    }

    /*for (size_t i = 0; i < odo_edges.size(); i++)
    {
        Eigen::Matrix<double, 6, 1> relative_pose_measurement_odo;
        relative_pose_tait_bryan_wc_case1(relative_pose_measurement_odo,
                                          poses_desired[odo_edges[i].first].px,
                                          poses_desired[odo_edges[i].first].py,
                                          poses_desired[odo_edges[i].first].pz,
                                          poses_desired[odo_edges[i].first].om,
                                          poses_desired[odo_edges[i].first].fi,
                                          poses_desired[odo_edges[i].first].ka,
                                          poses_desired[odo_edges[i].second].px,
                                          poses_desired[odo_edges[i].second].py,
                                          poses_desired[odo_edges[i].second].pz,
                                          poses_desired[odo_edges[i].second].om,
                                          poses_desired[odo_edges[i].second].fi,
                                          poses_desired[odo_edges[i].second].ka);

        Eigen::Matrix<double, 6, 1> delta;
        relative_pose_obs_eq_tait_bryan_wc_case1(
            delta,
            poses[odo_edges[i].first].px,
            poses[odo_edges[i].first].py,
            poses[odo_edges[i].first].pz,
            poses[odo_edges[i].first].om,
            poses[odo_edges[i].first].fi,
            poses[odo_edges[i].first].ka,
            poses[odo_edges[i].second].px,
            poses[odo_edges[i].second].py,
            poses[odo_edges[i].second].pz,
            poses[odo_edges[i].second].om,
            poses[odo_edges[i].second].fi,
            poses[odo_edges[i].second].ka,
            relative_pose_measurement_odo(0, 0),
            relative_pose_measurement_odo(1, 0),
            relative_pose_measurement_odo(2, 0),
            relative_pose_measurement_odo(3, 0),
            relative_pose_measurement_odo(4, 0),
            relative_pose_measurement_odo(5, 0));

        Eigen::Matrix<double, 6, 12, Eigen::RowMajor> jacobian;
        relative_pose_obs_eq_tait_bryan_wc_case1_jacobian(jacobian,
                                                          poses[odo_edges[i].first].px,
                                                          poses[odo_edges[i].first].py,
                                                          poses[odo_edges[i].first].pz,
                                                          poses[odo_edges[i].first].om,
                                                          poses[odo_edges[i].first].fi,
                                                          poses[odo_edges[i].first].ka,
                                                          poses[odo_edges[i].second].px,
                                                          poses[odo_edges[i].second].py,
                                                          poses[odo_edges[i].second].pz,
                                                          poses[odo_edges[i].second].om,
                                                          poses[odo_edges[i].second].fi,
                                                          poses[odo_edges[i].second].ka);

        int ir = tripletListB.size();

        int ic_1 = odo_edges[i].first * 6;
        int ic_2 = odo_edges[i].second * 6;

        for (size_t row = 0; row < 6; row++)
        {
            tripletListA.emplace_back(ir + row, ic_1, -jacobian(row, 0));
            tripletListA.emplace_back(ir + row, ic_1 + 1, -jacobian(row, 1));
            tripletListA.emplace_back(ir + row, ic_1 + 2, -jacobian(row, 2));
            tripletListA.emplace_back(ir + row, ic_1 + 3, -jacobian(row, 3));
            tripletListA.emplace_back(ir + row, ic_1 + 4, -jacobian(row, 4));
            tripletListA.emplace_back(ir + row, ic_1 + 5, -jacobian(row, 5));

            tripletListA.emplace_back(ir + row, ic_2, -jacobian(row, 6));
            tripletListA.emplace_back(ir + row, ic_2 + 1, -jacobian(row, 7));
            tripletListA.emplace_back(ir + row, ic_2 + 2, -jacobian(row, 8));
            tripletListA.emplace_back(ir + row, ic_2 + 3, -jacobian(row, 9));
            tripletListA.emplace_back(ir + row, ic_2 + 4, -jacobian(row, 10));
            tripletListA.emplace_back(ir + row, ic_2 + 5, -jacobian(row, 11));
        }

        tripletListB.emplace_back(ir, 0, delta(0, 0));
        tripletListB.emplace_back(ir + 1, 0, delta(1, 0));
        tripletListB.emplace_back(ir + 2, 0, delta(2, 0));
        tripletListB.emplace_back(ir + 3, 0, delta(3, 0));
        tripletListB.emplace_back(ir + 4, 0, delta(4, 0));
        tripletListB.emplace_back(ir + 5, 0, delta(5, 0));

        tripletListP.emplace_back(ir, ir, 1000000);
        tripletListP.emplace_back(ir + 1, ir + 1, 1000000);
        tripletListP.emplace_back(ir + 2, ir + 2, 1000000);
        tripletListP.emplace_back(ir + 3, ir + 3, 100000000);
        tripletListP.emplace_back(ir + 4, ir + 4, 100000000);
        tripletListP.emplace_back(ir + 5, ir + 5, 1000000);
    }*/
    for (size_t i = 0; i < odo_edges.size(); i++)
    {
        Eigen::Matrix<double, 6, 1> relative_pose_measurement_odo;
        relative_pose_tait_bryan_wc_case1_simplified_1(relative_pose_measurement_odo,
                                                       poses_desired[odo_edges[i].first].px,
                                                       poses_desired[odo_edges[i].first].py,
                                                       poses_desired[odo_edges[i].first].pz,
                                                       poses_desired[odo_edges[i].first].om,
                                                       poses_desired[odo_edges[i].first].fi,
                                                       poses_desired[odo_edges[i].first].ka,
                                                       poses_desired[odo_edges[i].second].px,
                                                       poses_desired[odo_edges[i].second].py,
                                                       poses_desired[odo_edges[i].second].pz,
                                                       poses_desired[odo_edges[i].second].om,
                                                       poses_desired[odo_edges[i].second].fi,
                                                       poses_desired[odo_edges[i].second].ka);

        Eigen::Matrix<double, 12, 12> AtPAodo;
        relative_pose_obs_eq_tait_bryan_wc_case1_AtPA_simplified(AtPAodo,
                                                                 poses[odo_edges[i].first].px,
                                                                 poses[odo_edges[i].first].py,
                                                                 poses[odo_edges[i].first].pz,
                                                                 poses[odo_edges[i].first].om,
                                                                 poses[odo_edges[i].first].fi,
                                                                 poses[odo_edges[i].first].ka,
                                                                 poses[odo_edges[i].second].px,
                                                                 poses[odo_edges[i].second].py,
                                                                 poses[odo_edges[i].second].pz,
                                                                 poses[odo_edges[i].second].om,
                                                                 poses[odo_edges[i].second].fi,
                                                                 poses[odo_edges[i].second].ka,
                                                                 1000000,
                                                                 1000000,
                                                                 1000000,
                                                                 100000000,
                                                                 100000000,
                                                                 1000000);
        Eigen::Matrix<double, 12, 1> AtPBodo;
        relative_pose_obs_eq_tait_bryan_wc_case1_AtPB_simplified(AtPBodo,
                                                                 poses[odo_edges[i].first].px,
                                                                 poses[odo_edges[i].first].py,
                                                                 poses[odo_edges[i].first].pz,
                                                                 poses[odo_edges[i].first].om,
                                                                 poses[odo_edges[i].first].fi,
                                                                 poses[odo_edges[i].first].ka,
                                                                 poses[odo_edges[i].second].px,
                                                                 poses[odo_edges[i].second].py,
                                                                 poses[odo_edges[i].second].pz,
                                                                 poses[odo_edges[i].second].om,
                                                                 poses[odo_edges[i].second].fi,
                                                                 poses[odo_edges[i].second].ka,
                                                                 relative_pose_measurement_odo(0, 0),
                                                                 relative_pose_measurement_odo(1, 0),
                                                                 relative_pose_measurement_odo(2, 0),
                                                                 relative_pose_measurement_odo(3, 0),
                                                                 relative_pose_measurement_odo(4, 0),
                                                                 relative_pose_measurement_odo(5, 0),
                                                                 1000000,
                                                                 1000000,
                                                                 1000000,
                                                                 100000000,
                                                                 100000000,
                                                                 1000000);
        int ic_1 = odo_edges[i].first * 6;
        int ic_2 = odo_edges[i].second * 6;

        for (int row = 0; row < 6; row++)
        {
            for (int col = 0; col < 6; col++)
            {
                AtPAndt(ic_1 + row, ic_1 + col) += AtPAodo(row, col);
                AtPAndt(ic_1 + row, ic_2 + col) += AtPAodo(row, col + 6);
                AtPAndt(ic_2 + row, ic_1 + col) += AtPAodo(row + 6, col);
                AtPAndt(ic_2 + row, ic_2 + col) += AtPAodo(row + 6, col + 6);
            }
        }

        for (int row = 0; row < 6; row++)
        {
            AtPBndt(ic_1 + row, 0) -= AtPBodo(row, 0);
            AtPBndt(ic_2 + row, 0) -= AtPBodo(row + 6, 0);
        }
    }

    // smoothness
    /*for (size_t i = 1; i < poses.size() - 1; i++)
    {
        Eigen::Matrix<double, 6, 1> delta;
        smoothness_obs_eq_tait_bryan_wc(delta,
                                        poses[i - 1].px,
                                        poses[i - 1].py,
                                        poses[i - 1].pz,
                                        poses[i - 1].om,
                                        poses[i - 1].fi,
                                        poses[i - 1].ka,
                                        poses[i].px,
                                        poses[i].py,
                                        poses[i].pz,
                                        poses[i].om,
                                        poses[i].fi,
                                        poses[i].ka,
                                        poses[i + 1].px,
                                        poses[i + 1].py,
                                        poses[i + 1].pz,
                                        poses[i + 1].om,
                                        poses[i + 1].fi,
                                        poses[i + 1].ka);

        Eigen::Matrix<double, 6, 18, Eigen::RowMajor> jacobian;
        smoothness_obs_eq_tait_bryan_wc_jacobian(jacobian,
                                                 poses[i - 1].px,
                                                 poses[i - 1].py,
                                                 poses[i - 1].pz,
                                                 poses[i - 1].om,
                                                 poses[i - 1].fi,
                                                 poses[i - 1].ka,
                                                 poses[i].px,
                                                 poses[i].py,
                                                 poses[i].pz,
                                                 poses[i].om,
                                                 poses[i].fi,
                                                 poses[i].ka,
                                                 poses[i + 1].px,
                                                 poses[i + 1].py,
                                                 poses[i + 1].pz,
                                                 poses[i + 1].om,
                                                 poses[i + 1].fi,
                                                 poses[i + 1].ka);

        int ir = tripletListB.size();

        int ic_1 = (i - 1) * 6;
        int ic_2 = i * 6;
        int ic_3 = (i + 1) * 6;

        for (size_t row = 0; row < 6; row++)
        {
            tripletListA.emplace_back(ir + row, ic_1, -jacobian(row, 0));
            tripletListA.emplace_back(ir + row, ic_1 + 1, -jacobian(row, 1));
            tripletListA.emplace_back(ir + row, ic_1 + 2, -jacobian(row, 2));
            tripletListA.emplace_back(ir + row, ic_1 + 3, -jacobian(row, 3));
            tripletListA.emplace_back(ir + row, ic_1 + 4, -jacobian(row, 4));
            tripletListA.emplace_back(ir + row, ic_1 + 5, -jacobian(row, 5));

            tripletListA.emplace_back(ir + row, ic_2, -jacobian(row, 6));
            tripletListA.emplace_back(ir + row, ic_2 + 1, -jacobian(row, 7));
            tripletListA.emplace_back(ir + row, ic_2 + 2, -jacobian(row, 8));
            tripletListA.emplace_back(ir + row, ic_2 + 3, -jacobian(row, 9));
            tripletListA.emplace_back(ir + row, ic_2 + 4, -jacobian(row, 10));
            tripletListA.emplace_back(ir + row, ic_2 + 5, -jacobian(row, 11));

            tripletListA.emplace_back(ir + row, ic_3, -jacobian(row, 12));
            tripletListA.emplace_back(ir + row, ic_3 + 1, -jacobian(row, 13));
            tripletListA.emplace_back(ir + row, ic_3 + 2, -jacobian(row, 14));
            tripletListA.emplace_back(ir + row, ic_3 + 3, -jacobian(row, 15));
            tripletListA.emplace_back(ir + row, ic_3 + 4, -jacobian(row, 16));
            tripletListA.emplace_back(ir + row, ic_3 + 5, -jacobian(row, 17));
        }
        tripletListB.emplace_back(ir, 0, delta(0, 0));
        tripletListB.emplace_back(ir + 1, 0, delta(1, 0));
        tripletListB.emplace_back(ir + 2, 0, delta(2, 0));
        tripletListB.emplace_back(ir + 3, 0, delta(3, 0));
        tripletListB.emplace_back(ir + 4, 0, delta(4, 0));
        tripletListB.emplace_back(ir + 5, 0, delta(5, 0));

        tripletListP.emplace_back(ir, ir, 10000);
        tripletListP.emplace_back(ir + 1, ir + 1, 10000);
        tripletListP.emplace_back(ir + 2, ir + 2, 10000);
        tripletListP.emplace_back(ir + 3, ir + 3, 10000);
        tripletListP.emplace_back(ir + 4, ir + 4, 10000);
        tripletListP.emplace_back(ir + 5, ir + 5, 10000);
    }*/

    // maintain angles
    if (add_pitch_roll_constraint)
    {
        for (int i = 0; i < imu_roll_pitch.size(); i++)
        {
            TaitBryanPose current_pose = poses[i];
            TaitBryanPose desired_pose = current_pose;
            desired_pose.om = imu_roll_pitch[i].first;
            desired_pose.fi = imu_roll_pitch[i].second;

            Eigen::Affine3d desired_mpose = affine_matrix_from_pose_tait_bryan(desired_pose);
            Eigen::Vector3d vx(desired_mpose(0, 0), desired_mpose(1, 0), desired_mpose(2, 0));
            Eigen::Vector3d vy(desired_mpose(0, 1), desired_mpose(1, 1), desired_mpose(2, 1));
            Eigen::Vector3d point_on_target_line(desired_mpose(0, 3), desired_mpose(1, 3), desired_mpose(2, 3));

            Eigen::Vector3d point_source_local(0, 0, 1);

            Eigen::Matrix<double, 2, 1> delta;
            point_to_line_tait_bryan_wc(delta,
                                        current_pose.px, current_pose.py, current_pose.pz, current_pose.om, current_pose.fi, current_pose.ka,
                                        point_source_local.x(), point_source_local.y(), point_source_local.z(),
                                        point_on_target_line.x(), point_on_target_line.y(), point_on_target_line.z(),
                                        vx.x(), vx.y(), vx.z(), vy.x(), vy.y(), vy.z());

            Eigen::Matrix<double, 2, 6> delta_jacobian;
            point_to_line_tait_bryan_wc_jacobian(delta_jacobian,
                                                 current_pose.px, current_pose.py, current_pose.pz, current_pose.om, current_pose.fi, current_pose.ka,
                                                 point_source_local.x(), point_source_local.y(), point_source_local.z(),
                                                 point_on_target_line.x(), point_on_target_line.y(), point_on_target_line.z(),
                                                 vx.x(), vx.y(), vx.z(), vy.x(), vy.y(), vy.z());

            int ir = tripletListB.size();

            for (int ii = 0; ii < 2; ii++)
            {
                for (int jj = 0; jj < 6; jj++)
                {
                    int ic = i * 6;
                    if (delta_jacobian(ii, jj) != 0.0)
                    {
                        tripletListA.emplace_back(ir + ii, ic + jj, -delta_jacobian(ii, jj));
                    }
                }
            }
            // tripletListP.emplace_back(ir, ir, cauchy(delta(0, 0), 1));
            // tripletListP.emplace_back(ir + 1, ir + 1, cauchy(delta(1, 0), 1));
            tripletListP.emplace_back(ir, ir, 1);
            tripletListP.emplace_back(ir + 1, ir + 1, 1);

            tripletListB.emplace_back(ir, 0, delta(0, 0));
            tripletListB.emplace_back(ir + 1, 0, delta(1, 0));
        }
    }

    Eigen::SparseMatrix<double> matA(tripletListB.size(), intermediate_trajectory.size() * 6);
    Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
    Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

    matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
    matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
    matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

    Eigen::SparseMatrix<double> AtPA(intermediate_trajectory.size() * 6, intermediate_trajectory.size() * 6);
    Eigen::SparseMatrix<double> AtPB(intermediate_trajectory.size() * 6, 1);

    {
        Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
        AtPA = (AtP)*matA;
        AtPB = (AtP)*matB;
    }

    tripletListA.clear();
    tripletListP.clear();
    tripletListB.clear();

    AtPA += AtPAndt.sparseView();
    AtPB += AtPBndt.sparseView();
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);
    Eigen::SparseMatrix<double> x = solver.solve(AtPB);
    std::vector<double> h_x;
    for (int k = 0; k < x.outerSize(); ++k)
    {
        for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
        {
            h_x.push_back(it.value());
        }
    }

    if (h_x.size() == 6 * intermediate_trajectory.size())
    {
        int counter = 0;

        for (size_t i = 0; i < intermediate_trajectory.size(); i++)
        {
            TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(intermediate_trajectory[i]);
            pose.px += h_x[counter++];
            pose.py += h_x[counter++];
            pose.pz += h_x[counter++];
            pose.om += h_x[counter++];
            pose.fi += h_x[counter++];
            pose.ka += h_x[counter++];
            intermediate_trajectory[i] = affine_matrix_from_pose_tait_bryan(pose);
        }
    }
    return;
}

void update_rgd(NDT::GridParameters &rgd_params, NDTBucketMapType &buckets,
                std::vector<Point3Di> &points_global)
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

            this_bucket.cov = this_bucket.cov * (this_bucket.number_of_points - 1) / this_bucket.number_of_points +
                              cov_update * (this_bucket.number_of_points - 1) / (this_bucket.number_of_points * this_bucket.number_of_points);
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

void align_to_reference(NDT::GridParameters &rgd_params, std::vector<Point3Di> &initial_points, Eigen::Affine3d &m_g, NDTBucketMapType &reference_buckets)
{
    Eigen::SparseMatrix<double> AtPAndt(6, 6);
    Eigen::SparseMatrix<double> AtPBndt(6, 1);

    Eigen::Vector3d b(rgd_params.resolution_X, rgd_params.resolution_Y, rgd_params.resolution_Z);

    for (int i = 0; i < initial_points.size(); i += 1)
    {
        // if (initial_points[i].point.norm() < 1.0)
        //{
        //     continue;
        // }

        Eigen::Vector3d point_global = m_g * initial_points[i].point;
        auto index_of_bucket = get_rgd_index(point_global, b);

        if (!reference_buckets.contains(index_of_bucket))
        {
            continue;
        }

        // if(buckets[index_of_bucket].number_of_points >= 5){
        Eigen::Matrix3d infm = reference_buckets[index_of_bucket].cov.inverse();

        constexpr double threshold = 10000.0;

        if ((infm.array() > threshold).any())
        {
            continue;
        }
        if ((infm.array() < -threshold).any())
        {
            continue;
        }

        const Eigen::Affine3d &m_pose = m_g;
        const Eigen::Vector3d &p_s = initial_points[i].point;
        const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose);
        //
        Eigen::Matrix<double, 6, 6, Eigen::RowMajor> AtPA;
        point_to_point_source_to_target_tait_bryan_wc_AtPA_simplified(
            AtPA,
            pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
            p_s.x(), p_s.y(), p_s.z(),
            infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2));

        Eigen::Matrix<double, 6, 1> AtPB;
        point_to_point_source_to_target_tait_bryan_wc_AtPB_simplified(
            AtPB,
            pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
            p_s.x(), p_s.y(), p_s.z(),
            infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2),
            reference_buckets[index_of_bucket].mean.x(), reference_buckets[index_of_bucket].mean.y(), reference_buckets[index_of_bucket].mean.z());

        int c = 0;

        for (int row = 0; row < 6; row++)
        {
            for (int col = 0; col < 6; col++)
            {
                AtPAndt.coeffRef(c + row, c + col) += AtPA(row, col);
            }
        }

        for (int row = 0; row < 6; row++)
        {
            AtPBndt.coeffRef(c + row, 0) -= AtPB(row, 0);
        }
        //}
    }

    AtPAndt.coeffRef(0, 0) += 10000.0;
    AtPAndt.coeffRef(1, 1) += 10000.0;
    AtPAndt.coeffRef(2, 2) += 10000.0;
    AtPAndt.coeffRef(3, 3) += 10000.0;
    AtPAndt.coeffRef(4, 4) += 10000.0;
    AtPAndt.coeffRef(5, 5) += 10000.0;

    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPAndt);
    Eigen::SparseMatrix<double> x = solver.solve(AtPBndt);
    std::vector<double> h_x;
    for (int k = 0; k < x.outerSize(); ++k)
    {
        for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
        {
            h_x.push_back(it.value());
        }
    }

    if (h_x.size() == 6)
    {
        int counter = 0;
        TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(m_g);
        pose.px += h_x[counter++];
        pose.py += h_x[counter++];
        pose.pz += h_x[counter++];
        pose.om += h_x[counter++];
        pose.fi += h_x[counter++];
        pose.ka += h_x[counter++];
        m_g = affine_matrix_from_pose_tait_bryan(pose);
    }
    else
    {
        std::cout << "align_to_reference FAILED" << std::endl;
    }
}

void fix_ptch_roll(std::vector<WorkerData> &worker_data)
{
    std::vector<Eigen::Triplet<double>> tripletListA;
    std::vector<Eigen::Triplet<double>> tripletListP;
    std::vector<Eigen::Triplet<double>> tripletListB;

    std::vector<TaitBryanPose> poses;

    for (size_t i = 0; i < worker_data.size(); i++)
    {
        poses.push_back(pose_tait_bryan_from_affine_matrix(worker_data[i].intermediate_trajectory[0]));
    }

    for (size_t i = 1; i < poses.size(); i++)
    {
        TaitBryanPose pose_prev = pose_tait_bryan_from_affine_matrix(worker_data[i - 1].intermediate_trajectory_motion_model[0]);
        pose_prev.om = worker_data[i - 1].imu_roll_pitch[0].first;
        pose_prev.fi = worker_data[i - 1].imu_roll_pitch[0].second;
        Eigen::Affine3d mrot_prev = affine_matrix_from_pose_tait_bryan(pose_prev);
        mrot_prev(0, 3) = 0;
        mrot_prev(1, 3) = 0;
        mrot_prev(2, 3) = 0;

        TaitBryanPose pose_curr = pose_tait_bryan_from_affine_matrix(worker_data[i].intermediate_trajectory_motion_model[0]);
        pose_curr.om = worker_data[i].imu_roll_pitch[0].first;
        pose_curr.fi = worker_data[i].imu_roll_pitch[0].second;
        Eigen::Affine3d mrot_curr = affine_matrix_from_pose_tait_bryan(pose_curr);
        mrot_curr(0, 3) = 0;
        mrot_curr(1, 3) = 0;
        mrot_curr(2, 3) = 0;

        auto m_rot_rel = mrot_prev.inverse() * mrot_curr;
        auto tb_rot_rel = pose_tait_bryan_from_affine_matrix(m_rot_rel);

        Eigen::Vector3d relative_translation = (worker_data[i - 1].intermediate_trajectory_motion_model[0].inverse() *
                                                worker_data[i].intermediate_trajectory_motion_model[0])
                                                   .translation();

        auto m = worker_data[i - 1].intermediate_trajectory_motion_model[0];
        m(0, 3) = 0;
        m(1, 3) = 0;
        m(2, 3) = 0;

        relative_translation = (mrot_prev.inverse() * m) * relative_translation;

        Eigen::Matrix<double, 6, 1> delta;
        relative_pose_obs_eq_tait_bryan_wc_case1(
            delta,
            poses[i - 1].px,
            poses[i - 1].py,
            poses[i - 1].pz,
            poses[i - 1].om,
            poses[i - 1].fi,
            poses[i - 1].ka,
            poses[i].px,
            poses[i].py,
            poses[i].pz,
            poses[i].om,
            poses[i].fi,
            poses[i].ka,
            relative_translation.x(),
            relative_translation.y(),
            relative_translation.z(),
            tb_rot_rel.om,
            tb_rot_rel.fi,
            tb_rot_rel.ka);

        Eigen::Matrix<double, 6, 12, Eigen::RowMajor> jacobian;
        relative_pose_obs_eq_tait_bryan_wc_case1_jacobian(jacobian,
                                                          poses[i - 1].px,
                                                          poses[i - 1].py,
                                                          poses[i - 1].pz,
                                                          poses[i - 1].om,
                                                          poses[i - 1].fi,
                                                          poses[i - 1].ka,
                                                          poses[i].px,
                                                          poses[i].py,
                                                          poses[i].pz,
                                                          poses[i].om,
                                                          poses[i].fi,
                                                          poses[i].ka);

        int ir = tripletListB.size();

        int ic_1 = (i - 1) * 6;
        int ic_2 = i * 6;

        for (size_t row = 0; row < 6; row++)
        {
            tripletListA.emplace_back(ir + row, ic_1, -jacobian(row, 0));
            tripletListA.emplace_back(ir + row, ic_1 + 1, -jacobian(row, 1));
            tripletListA.emplace_back(ir + row, ic_1 + 2, -jacobian(row, 2));
            tripletListA.emplace_back(ir + row, ic_1 + 3, -jacobian(row, 3));
            tripletListA.emplace_back(ir + row, ic_1 + 4, -jacobian(row, 4));
            tripletListA.emplace_back(ir + row, ic_1 + 5, -jacobian(row, 5));

            tripletListA.emplace_back(ir + row, ic_2, -jacobian(row, 6));
            tripletListA.emplace_back(ir + row, ic_2 + 1, -jacobian(row, 7));
            tripletListA.emplace_back(ir + row, ic_2 + 2, -jacobian(row, 8));
            tripletListA.emplace_back(ir + row, ic_2 + 3, -jacobian(row, 9));
            tripletListA.emplace_back(ir + row, ic_2 + 4, -jacobian(row, 10));
            tripletListA.emplace_back(ir + row, ic_2 + 5, -jacobian(row, 11));
        }

        tripletListB.emplace_back(ir, 0, delta(0, 0));
        tripletListB.emplace_back(ir + 1, 0, delta(1, 0));
        tripletListB.emplace_back(ir + 2, 0, delta(2, 0));
        tripletListB.emplace_back(ir + 3, 0, delta(3, 0));
        tripletListB.emplace_back(ir + 4, 0, delta(4, 0));
        tripletListB.emplace_back(ir + 5, 0, delta(5, 0));

        tripletListP.emplace_back(ir, ir, 1);
        tripletListP.emplace_back(ir + 1, ir + 1, 1);
        tripletListP.emplace_back(ir + 2, ir + 2, 1);
        tripletListP.emplace_back(ir + 3, ir + 3, 1);
        tripletListP.emplace_back(ir + 4, ir + 4, 1);
        tripletListP.emplace_back(ir + 5, ir + 5, 1);
    }

    double rms = 0.0;
    for (size_t i = 0; i < worker_data.size(); i++)
    {
        TaitBryanPose current_pose = pose_tait_bryan_from_affine_matrix(worker_data[i].intermediate_trajectory[0]);
        TaitBryanPose desired_pose = current_pose;
        desired_pose.om = worker_data[i].imu_roll_pitch[0].first;
        desired_pose.fi = worker_data[i].imu_roll_pitch[0].second;

        Eigen::Affine3d desired_mpose = affine_matrix_from_pose_tait_bryan(desired_pose);
        Eigen::Vector3d vx(desired_mpose(0, 0), desired_mpose(1, 0), desired_mpose(2, 0));
        Eigen::Vector3d vy(desired_mpose(0, 1), desired_mpose(1, 1), desired_mpose(2, 1));
        Eigen::Vector3d point_on_target_line(desired_mpose(0, 3), desired_mpose(1, 3), desired_mpose(2, 3));

        Eigen::Vector3d point_source_local(0, 0, 1);

        Eigen::Matrix<double, 2, 1> delta;
        point_to_line_tait_bryan_wc(delta,
                                    current_pose.px, current_pose.py, current_pose.pz, current_pose.om, current_pose.fi, current_pose.ka,
                                    point_source_local.x(), point_source_local.y(), point_source_local.z(),
                                    point_on_target_line.x(), point_on_target_line.y(), point_on_target_line.z(),
                                    vx.x(), vx.y(), vx.z(), vy.x(), vy.y(), vy.z());

        Eigen::Matrix<double, 2, 6> delta_jacobian;
        point_to_line_tait_bryan_wc_jacobian(delta_jacobian,
                                             current_pose.px, current_pose.py, current_pose.pz, current_pose.om, current_pose.fi, current_pose.ka,
                                             point_source_local.x(), point_source_local.y(), point_source_local.z(),
                                             point_on_target_line.x(), point_on_target_line.y(), point_on_target_line.z(),
                                             vx.x(), vx.y(), vx.z(), vy.x(), vy.y(), vy.z());

        int ir = tripletListB.size();

        for (int ii = 0; ii < 2; ii++)
        {
            for (int jj = 0; jj < 6; jj++)
            {
                int ic = i * 6;
                if (delta_jacobian(ii, jj) != 0.0)
                {
                    tripletListA.emplace_back(ir + ii, ic + jj, -delta_jacobian(ii, jj));
                }
            }
        }
        // tripletListP.emplace_back(ir, ir, cauchy(delta(0, 0), 1));
        // tripletListP.emplace_back(ir + 1, ir + 1, cauchy(delta(1, 0), 1));
        tripletListP.emplace_back(ir, ir, 1);
        tripletListP.emplace_back(ir + 1, ir + 1, 1);

        tripletListB.emplace_back(ir, 0, delta(0, 0));
        tripletListB.emplace_back(ir + 1, 0, delta(1, 0));

        rms += sqrt(delta(0, 0) * delta(0, 0) + delta(1, 0) * delta(1, 0));
    }
    std::cout << "rms: " << rms << std::endl;

    int ir = tripletListB.size();
    tripletListA.emplace_back(ir, 0, 1);
    tripletListA.emplace_back(ir + 1, 1, 1);
    tripletListA.emplace_back(ir + 2, 2, 1);
    tripletListA.emplace_back(ir + 3, 3, 1);
    tripletListA.emplace_back(ir + 4, 4, 1);
    tripletListA.emplace_back(ir + 5, 5, 1);

    tripletListP.emplace_back(ir, ir, 10000000000000);
    tripletListP.emplace_back(ir + 1, ir + 1, 10000000000000);
    tripletListP.emplace_back(ir + 2, ir + 2, 10000000000000);
    tripletListP.emplace_back(ir + 3, ir + 3, 10000000000000);
    tripletListP.emplace_back(ir + 4, ir + 4, 10000000000000);
    tripletListP.emplace_back(ir + 5, ir + 5, 10000000000000);

    tripletListB.emplace_back(ir, 0, 0);
    tripletListB.emplace_back(ir + 1, 0, 0);
    tripletListB.emplace_back(ir + 2, 0, 0);
    tripletListB.emplace_back(ir + 3, 0, 0);
    tripletListB.emplace_back(ir + 4, 0, 0);
    tripletListB.emplace_back(ir + 5, 0, 0);

    Eigen::SparseMatrix<double> matA(tripletListB.size(), poses.size() * 6);
    Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
    Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

    matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
    matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
    matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

    Eigen::SparseMatrix<double> AtPA(poses.size() * 6, poses.size() * 6);
    Eigen::SparseMatrix<double> AtPB(poses.size() * 6, 1);

    {
        Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
        AtPA = (AtP)*matA;
        AtPB = (AtP)*matB;
    }

    tripletListA.clear();
    tripletListP.clear();
    tripletListB.clear();

    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);
    Eigen::SparseMatrix<double> x = solver.solve(AtPB);
    std::vector<double> h_x;
    for (int k = 0; k < x.outerSize(); ++k)
    {
        for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
        {
            h_x.push_back(it.value());
        }
    }

    if (h_x.size() == 6 * poses.size())
    {
        std::vector<Eigen::Affine3d> results;

        int counter = 0;

        for (size_t i = 0; i < poses.size(); i++)
        {

            poses[i].px += h_x[counter++];
            poses[i].py += h_x[counter++];
            poses[i].pz += h_x[counter++];
            poses[i].om += h_x[counter++];
            poses[i].fi += h_x[counter++];
            poses[i].ka += h_x[counter++];

            // worker_data[i].intermediate_trajectory[0] = affine_matrix_from_pose_tait_bryan(poses[i]);
            results.push_back(affine_matrix_from_pose_tait_bryan(poses[i]));
        }

        for (int i = 0; i < worker_data.size(); i++)
        {
            Eigen::Affine3d m_last = results[i]; // worker_data[i].intermediate_trajectory[0];

            std::vector<Eigen::Affine3d> local_result;
            local_result.push_back(m_last);
            for (int j = 1; j < worker_data[i].intermediate_trajectory.size(); j++)
            {
                m_last = m_last * (worker_data[i].intermediate_trajectory[j - 1].inverse() * worker_data[i].intermediate_trajectory[j]);
                local_result.push_back(m_last);
            }

            for (int j = 0; j < worker_data[i].intermediate_trajectory.size(); j++)
            {
                worker_data[i].intermediate_trajectory[j] = local_result[j];
            }
        }
    }
}
// clang-format off
#include <GL/glew.h>
#include <GL/freeglut.h>
// clang-format on

#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <imgui_internal.h>
#include <implot/implot.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <utils.hpp>

#include <Eigen/Eigen>

#include <transformations.h>

#include "pfd_wrapper.hpp"

#include "../lidar_odometry_step_1/lidar_odometry.h"
#include "../lidar_odometry_step_1/lidar_odometry_utils.h"
#include <filesystem>

#include <HDMapping/Version.hpp>

#include "tbb/tbb.h"
#include <mutex>

#include <export_laz.h>

#include <opencv2/opencv.hpp>

#include <hash_utils.h>

#ifdef _WIN32
#include "resource.h"
#include <shellapi.h> // <-- Required for ShellExecuteA
#include <windows.h>
#endif

///////////////////////////////////////////////////////////////////////////////////

std::string winTitle = std::string("Raw data viewer ") + HDMAPPING_VERSION_STRING;

std::vector<std::string> infoLines = { "This program is optional step in MANDEYE process",
                                       "",
                                       "It analyzes LiDAR data created by Mission Recorder",
                                       "Next step will be to load data with 'lidar_odometry_step_1' app" };

// App specific shortcuts (using empty dummy until needed)
std::vector<ShortcutEntry> appShortcuts(80, { "", "", "" });

#define SAMPLE_PERIOD (1.0 / 200.0)
namespace fs = std::filesystem;

ImVec4 pc_color = ImVec4(1.0f, 0.0f, 0.0f, 1.00f);
ImVec4 pc_color2 = ImVec4(0.0f, 0.0f, 1.0f, 1.00f);

struct AllData
{
    std::vector<std::pair<double, double>> timestamps;
    std::vector<Eigen::Affine3d> poses;
    std::vector<Point3Di> points_local;
    std::vector<int> lidar_ids;
};
std::vector<AllData> all_data;

struct ImuData
{
    std::vector<double> timestampLidar;
    std::vector<double> angX;
    std::vector<double> angY;
    std::vector<double> angZ;
    std::vector<double> accX;
    std::vector<double> accY;
    std::vector<double> accZ;
    std::vector<double> yaw;
    std::vector<double> pitch;
    std::vector<double> roll;
};
ImuData imu_data_plot;

std::vector<std::string> laz_files;
std::vector<std::string> photos_files;
std::map<uint64_t, std::string> photo_files_ts;

double filter_threshold_xy_inner = 0.0; // no filtering for raw viewing
double filter_threshold_xy_outer = 300.0; // no filtering for raw viewing
bool fusionConventionNwu = true;
bool fusionConventionEnu = false;
bool fusionConventionNed = false;
int number_of_points_threshold = 20000;
bool is_init = false;
int index_rendered_points_local = -1;
// std::vector<std::vector<Point3Di>> all_points_local;
// std::vector<std::vector<Point3Di>> all_points_local;
// std::vector<std::vector<int>> all_lidar_ids;
std::vector<int> indexes_to_filename;
double ahrs_gain = 0.5;
double wx = 1000000.0;
double wy = 1000000.0;
double wz = 1000000.0;
double wom = 1000000.0;
double wfi = 1000000.0;
double wka = 1000000.0;
// bool is_slerp = false;

double distance_bucket = 0.5;
double polar_angle_deg = 5;
double azimutal_angle_deg = 5;
double max_distance_lidar = 70.0;
int robust_and_accurate_lidar_odometry_iterations = 20;
bool useMultithread = true;

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rgd_nn;
std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> mean_cov;
bool show_mean_cov = false;
bool show_rgd_nn = false;
bool show_imu_data = false;
bool show_cameras_data = false;
bool is_settings_gui = false;

namespace photos
{
    cv::Mat imgToShow;
    std::string imgToShowFn;
    GLuint photo_texture_cam0 = 0;
    double nearestTs = 0;
    int photo_width_cam0 = 0;
    int photo_height_cam0 = 0;
} // namespace photos

///////////////////////////////////////////////////////////////////////////////////

static ImVec2 DisplayImageFit(ImTextureID tex, int tex_w, int tex_h, bool allow_upscale = true)
{
    ImVec2 avail = ImGui::GetContentRegionAvail();
    if (tex_w <= 0 || tex_h <= 0 || avail.x <= 0.0f || avail.y <= 0.0f)
        return ImVec2(0, 0);

    float sx = avail.x / float(tex_w);
    float sy = avail.y / float(tex_h);
    float scale = std::min(sx, sy);
    if (!allow_upscale)
        scale = std::min(scale, 1.0f);

    ImVec2 disp(scale * tex_w, scale * tex_h);

    ImVec2 cur = ImGui::GetCursorPos();
    ImGui::SetCursorPosX(cur.x + (avail.x - disp.x) * 0.5f); // center horizontally
    ImGui::Image(tex, disp);
    // cursor Y is already advanced by Image; no restore needed

    return disp;
}

void render_nearest_photo(double ts)
{
    using namespace photos;
    // find closest photo
    if (photo_files_ts.empty())
    {
        return;
    }
    uint64_t ts_uint64 = static_cast<uint64_t>(ts * 1e9);
    auto it = photo_files_ts.lower_bound(ts_uint64);
    if (it == photo_files_ts.end())
    {
        return;
    }
    const std::string& photo_file = it->second;
    if (photo_file == imgToShowFn)
    {
        return;
    }
    std::cout << "render_nearest_photo: " << photo_file << std::endl;
    cv::Mat img = cv::imread(photo_file);
    if (img.empty())
    {
        return;
    }
    cv::Mat img_rgb;
    cv::cvtColor(img, img_rgb, cv::COLOR_BGR2RGB);
    imgToShow = img_rgb;
    imgToShowFn = photo_file;
    photos::nearestTs = double(it->first) / 1e9;
    // upload to OpenGL texture
    photo_width_cam0 = imgToShow.cols;
    photo_height_cam0 = imgToShow.rows;
    if (photo_texture_cam0 == 0)
    {
        glGenTextures(1, &photo_texture_cam0);
    }
    glBindTexture(GL_TEXTURE_2D, photo_texture_cam0);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, photo_width_cam0, photo_height_cam0, 0, GL_RGB, GL_UNSIGNED_BYTE, imgToShow.data);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void optimize()
{
#if 0
    rgd_nn.clear();

    // NDT::GridParameters rgd_params_sc;

    // rgd_params_sc.resolution_X = distance_bucket;
    // rgd_params_sc.resolution_Y = polar_angle_deg;
    // rgd_params_sc.resolution_Z = azimutal_angle_deg;

    if (index_rendered_points_local >= 0 && index_rendered_points_local < all_data.size())
    {
        // std::vector<Point3Di> points_local_sf;
        std::vector<Point3Di> points_local;
        auto tr = all_data[index_rendered_points_local].poses;
        auto trmm = all_data[index_rendered_points_local].poses;

        for (size_t i = 0; i < all_data[index_rendered_points_local].points_local.size(); i++)
        {
            auto lower = std::lower_bound(all_data[index_rendered_points_local].timestamps.begin(), all_data[index_rendered_points_local].timestamps.end(), all_data[index_rendered_points_local].points_local[i].timestamp,
                [](std::pair<double, double> lhs, double rhs) -> bool
                { return lhs.first < rhs; });

            int index_pose = std::distance(all_data[index_rendered_points_local].timestamps.begin(), lower) - 1;

            if (index_pose >= 0 && index_pose < all_data[index_rendered_points_local].poses.size())
            {
                // Eigen::Affine3d m = all_data[index_rendered_points_local].poses[index_pose];
                // Eigen::Vector3d p = m * all_data[index_rendered_points_local].points_local[i].point;
                double r_l = all_data[index_rendered_points_local].points_local[i].point.norm();
                if (r_l > 0.5 && all_data[index_rendered_points_local].points_local[i].index_pose != -1 && r_l < max_distance_lidar)
                {
                    // double polar_angle_deg_l = atan2(all_data[index_rendered_points_local].points_local[i].point.y(), all_data[index_rendered_points_local].points_local[i].point.x()) / M_PI * 180.0;
                    // double azimutal_angle_deg_l = acos(all_data[index_rendered_points_local].points_local[i].point.z() / r_l) / M_PI * 180.0;
                    all_data[index_rendered_points_local].points_local[i].index_pose = index_pose;
                    points_local.push_back(all_data[index_rendered_points_local].points_local[i]);

                    ///////////////////////////////////////////////////////
                    // Point3Di p_sl = all_data[index_rendered_points_local].points_local[i];
                    // p_sl.point.x() = r_l;
                    // p_sl.point.y() = polar_angle_deg_l;
                    // p_sl.point.z() = azimutal_angle_deg_l;

                    // points_local_sf.push_back(p_sl);
                }
            }
        }

        //std::cout << "points_local.size(): " << points_local.size() << std::endl;
        // points_local
        // points_local_sf

        std::vector<Point3Di>
            point_cloud_global;
        std::vector<Eigen::Vector3d> point_cloud_global_sc;

        for (size_t i = 0; i < points_local.size(); i++)
        {
            Point3Di pg = points_local[i];
            pg.point = tr[points_local[i].index_pose] * pg.point;
            point_cloud_global.push_back(pg);
            double r_g = pg.point.norm();
            point_cloud_global_sc.emplace_back(r_g, atan2(pg.point.y(), pg.point.x()) / M_PI * 180.0, acos(pg.point.z() / r_g) / M_PI * 180.0);
        }

        NDT::GridParameters rgd_params_sc;

        rgd_params_sc.resolution_X = distance_bucket;
        rgd_params_sc.resolution_Y = polar_angle_deg;
        rgd_params_sc.resolution_Z = azimutal_angle_deg;
        Eigen::Vector3d b(rgd_params_sc.resolution_X, rgd_params_sc.resolution_Y, rgd_params_sc.resolution_Z);

        NDTBucketMapType buckets;
        update_rgd_spherical_coordinates(rgd_params_sc, buckets, point_cloud_global, point_cloud_global_sc);
        //update_rgd(rgd_params_sc, buckets, point_cloud_global, {0, 0, 0});

        std::vector<Eigen::Triplet<double>> tripletListA;
        std::vector<Eigen::Triplet<double>> tripletListP;
        std::vector<Eigen::Triplet<double>> tripletListB;

        for (size_t i = 0; i < point_cloud_global.size(); i++)
        {

            auto index_of_bucket = get_rgd_index_3d(point_cloud_global_sc[i], b);
            //auto index_of_bucket = get_rgd_index_3d(point_cloud_global[i].point, b);
            auto bucket_it = buckets.find(index_of_bucket);
            // no bucket found
            if (bucket_it == buckets.end())
            {
                continue;
            }
            auto& this_bucket = bucket_it->second;

            rgd_nn.emplace_back(point_cloud_global[i].point, this_bucket.mean);

            const Eigen::Matrix3d& infm = this_bucket.cov.inverse();
            const double threshold = 100000.0;

            if ((infm.array() > threshold).any())
            {
                continue;
            }
            if ((infm.array() < -threshold).any())
            {
                continue;
            }

            // std::cout << infm << std::endl;

            const Eigen::Affine3d& m_pose = tr[points_local[i].index_pose];
            //std::cout << "points_local[i].index_pose " << points_local[i].index_pose << std::endl;
            const Eigen::Vector3d& p_s = points_local[i].point;
            const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose);
            double delta_x;
            double delta_y;
            double delta_z;

            ////
            Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
            // TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix((*mposes)[p.index_pose]);

            point_to_point_source_to_target_tait_bryan_wc(delta_x, delta_y, delta_z,
                pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                p_s.x(), p_s.y(), p_s.z(), this_bucket.mean.x(), this_bucket.mean.y(), this_bucket.mean.z());

            // std::cout << "delta_x: " << delta_x << std::endl;
            // std::cout << "delta_y: " << delta_y << std::endl;
            // std::cout << "delta_z: " << delta_z << std::endl;

            if (sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z) < 0.001)
            {
                continue;
            }

            point_to_point_source_to_target_tait_bryan_wc_jacobian(jacobian, pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka, p_s.x(), p_s.y(), p_s.z());

            int c = points_local[i].index_pose * 6;

            // std::mutex &m = my_mutex[0]; // mutexes[intermediate_points_i.index_pose];
            // std::unique_lock lck(m);
            int ir = tripletListB.size();

            for (int row = 0; row < 3; row++)
            {
                for (int col = 0; col < 6; col++)
                {
                    if (jacobian(row, col) != 0.0)
                    {
                        tripletListA.emplace_back(ir + row, c + col, -jacobian(row, col));
                    }
                }
            }
            tripletListB.emplace_back(ir, 0, delta_x);
            tripletListB.emplace_back(ir + 1, 0, delta_y);
            tripletListB.emplace_back(ir + 2, 0, delta_z);

            tripletListP.emplace_back(ir, ir, infm(0, 0));
            tripletListP.emplace_back(ir, ir + 1, infm(0, 1));
            tripletListP.emplace_back(ir, ir + 2, infm(0, 2));
            tripletListP.emplace_back(ir + 1, ir, infm(1, 0));
            tripletListP.emplace_back(ir + 1, ir + 1, infm(1, 1));
            tripletListP.emplace_back(ir + 1, ir + 2, infm(1, 2));
            tripletListP.emplace_back(ir + 2, ir, infm(2, 0));
            tripletListP.emplace_back(ir + 2, ir + 1, infm(2, 1));
            tripletListP.emplace_back(ir + 2, ir + 2, infm(2, 2));

            ///
        }

        std::vector<std::pair<int, int>> odo_edges;
        for (size_t i = 1; i < tr.size(); i++)
        {
            odo_edges.emplace_back(i - 1, i);
        }

        std::vector<TaitBryanPose> poses;
        std::vector<TaitBryanPose> poses_desired;

        for (size_t i = 0; i < tr.size(); i++)
        {
            poses.push_back(pose_tait_bryan_from_affine_matrix(tr[i]));
        }
        for (size_t i = 0; i < trmm.size(); i++)
        {
            poses_desired.push_back(pose_tait_bryan_from_affine_matrix(trmm[i]));
        }

        for (size_t i = 0; i < odo_edges.size(); i++)
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

            tripletListP.emplace_back(ir, ir, wx);
            tripletListP.emplace_back(ir + 1, ir + 1, wy);
            tripletListP.emplace_back(ir + 2, ir + 2, wz);
            tripletListP.emplace_back(ir + 3, ir + 3, wom);
            tripletListP.emplace_back(ir + 4, ir + 4, wfi);
            tripletListP.emplace_back(ir + 5, ir + 5, wka);
        }

        int ic = 0;
        int ir = tripletListB.size();
        tripletListA.emplace_back(ir, ic * 6 + 0, 1);
        tripletListA.emplace_back(ir + 1, ic * 6 + 1, 1);
        tripletListA.emplace_back(ir + 2, ic * 6 + 2, 1);
        tripletListA.emplace_back(ir + 3, ic * 6 + 3, 1);
        tripletListA.emplace_back(ir + 4, ic * 6 + 4, 1);
        tripletListA.emplace_back(ir + 5, ic * 6 + 5, 1);

        /*tripletListP.emplace_back(ir, ir, 1000000);
        tripletListP.emplace_back(ir + 1, ir + 1, 1000000);
        tripletListP.emplace_back(ir + 2, ir + 2, 1000000);
        tripletListP.emplace_back(ir + 3, ir + 3, 1000000);
        tripletListP.emplace_back(ir + 4, ir + 4, 1000000);
        tripletListP.emplace_back(ir + 5, ir + 5, 1000000);*/
        tripletListP.emplace_back(ir, ir, 1);
        tripletListP.emplace_back(ir + 1, ir + 1, 1);
        tripletListP.emplace_back(ir + 2, ir + 2, 1);
        tripletListP.emplace_back(ir + 3, ir + 3, 1);
        tripletListP.emplace_back(ir + 4, ir + 4, 1);
        tripletListP.emplace_back(ir + 5, ir + 5, 1);

        tripletListB.emplace_back(ir, 0, 0);
        tripletListB.emplace_back(ir + 1, 0, 0);
        tripletListB.emplace_back(ir + 2, 0, 0);
        tripletListB.emplace_back(ir + 3, 0, 0);
        tripletListB.emplace_back(ir + 4, 0, 0);
        tripletListB.emplace_back(ir + 5, 0, 0);

        Eigen::SparseMatrix<double> matA(tripletListB.size(), tr.size() * 6);
        Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
        Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

        matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
        matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
        matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

        Eigen::SparseMatrix<double> AtPA(tr.size() * 6, tr.size() * 6);
        Eigen::SparseMatrix<double> AtPB(tr.size() * 6, 1);

        {
            Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
            AtPA = (AtP)*matA;
            AtPB = (AtP)*matB;
        }

        tripletListA.clear();
        tripletListP.clear();
        tripletListB.clear();

        // AtPA += AtPAndt.sparseView();
        // AtPB += AtPBndt.sparseView();

        // Eigen::SparseMatrix<double> AtPA_I(tr.size() * 6, tr.size() * 6);
        // AtPA_I.setIdentity();
        // AtPA += AtPA_I;

        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>>
            solver(AtPA);
        Eigen::SparseMatrix<double> x = solver.solve(AtPB);
        std::vector<double> h_x;
        for (int k = 0; k < x.outerSize(); ++k)
        {
            for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            {
                h_x.push_back(it.value());

                std::cout << it.value() << " ";
            }
        }

        if (h_x.size() == 6 * tr.size())
        {
            int counter = 0;

            for (size_t i = 0; i < tr.size(); i++)
            {
                TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(tr[i]);
                auto prev_pose = pose;
                pose.px += h_x[counter++];
                pose.py += h_x[counter++];
                pose.pz += h_x[counter++];
                pose.om += h_x[counter++];
                pose.fi += h_x[counter++];
                pose.ka += h_x[counter++];

                // Eigen::Vector3d p1(prev_pose.px, prev_pose.py, prev_pose.pz);
                // Eigen::Vector3d p2(pose.px, pose.py, pose.pz);

                // if ((p1 - p2).norm() < 1.0)
                //{
                tr[i] = affine_matrix_from_pose_tait_bryan(pose);
                //}
            }
            all_data[index_rendered_points_local].poses = tr;
        }
    }
#endif

    NDT::GridParameters rgd_params_sc;

    rgd_params_sc.resolution_X = distance_bucket;
    rgd_params_sc.resolution_Y = polar_angle_deg;
    rgd_params_sc.resolution_Z = azimutal_angle_deg;

    if (index_rendered_points_local >= 0 && index_rendered_points_local < all_data.size())
    {
        std::vector<Eigen::Affine3d> tr = all_data[index_rendered_points_local].poses; // = worker_data[i].intermediate_trajectory;
        std::vector<Eigen::Affine3d> trmm =
            all_data[index_rendered_points_local].poses; // = worker_data[i].intermediate_trajectory_motion_model;

        std::vector<Point3Di> points_local_sf;
        std::vector<Point3Di> points_local;

        for (size_t i = 0; i < all_data[index_rendered_points_local].points_local.size(); i++)
        {
            auto lower = std::lower_bound(
                all_data[index_rendered_points_local].timestamps.begin(),
                all_data[index_rendered_points_local].timestamps.end(),
                all_data[index_rendered_points_local].points_local[i].timestamp,
                [](std::pair<double, double> lhs, double rhs) -> bool
                {
                    return lhs.first < rhs;
                });

            int index_pose = std::distance(all_data[index_rendered_points_local].timestamps.begin(), lower) - 1;

            if (index_pose >= 0 && index_pose < all_data[index_rendered_points_local].poses.size())
            {
                all_data[index_rendered_points_local].points_local[i].index_pose = index_pose;
                // Eigen::Affine3d m = all_data[index_rendered_points_local].poses[index_pose];
                // Eigen::Vector3d p = m * all_data[index_rendered_points_local].points_local[i].point;
                double r_l = all_data[index_rendered_points_local].points_local[i].point.norm();
                if (r_l > 0.5 && all_data[index_rendered_points_local].points_local[i].index_pose != -1 && r_l < max_distance_lidar)
                {
                    double polar_angle_deg_l = atan2(
                                                   all_data[index_rendered_points_local].points_local[i].point.y(),
                                                   all_data[index_rendered_points_local].points_local[i].point.x()) /
                        M_PI * 180.0;
                    double azimutal_angle_deg_l =
                        acos(all_data[index_rendered_points_local].points_local[i].point.z() / r_l) / M_PI * 180.0;

                    points_local.push_back(all_data[index_rendered_points_local].points_local[i]);

                    ///////////////////////////////////////////////////////
                    Point3Di p_sl = all_data[index_rendered_points_local].points_local[i];
                    p_sl.point.x() = r_l;
                    p_sl.point.y() = polar_angle_deg_l;
                    p_sl.point.z() = azimutal_angle_deg_l;

                    points_local_sf.push_back(p_sl);
                }
            }
        }

        std::cout << "optimize_sf2" << std::endl;
        for (int iter = 0; iter < robust_and_accurate_lidar_odometry_iterations; iter++)
        {
            optimize_sf2(points_local, points_local_sf, tr, trmm, rgd_params_sc, useMultithread, wx, wy, wz, wom, wfi, wka);
            trmm = tr;
        }

        all_data[index_rendered_points_local].poses = tr;
    }

    return;
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> get_nn()
{
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> nn;
    if (index_rendered_points_local >= 0 && index_rendered_points_local < all_data.size())
    {
        // for (size_t i = 0; i < all_data[index_rendered_points_local].poses.size(); i++){
        //     all_data[index_rendered_points_local].poses[i](0, 3) += 1.0;
        // }

        auto worker_data = all_data[index_rendered_points_local];

        // index data
        for (size_t i = 0; i < worker_data.points_local.size(); i++)
        {
            auto lower = std::lower_bound(
                worker_data.timestamps.begin(),
                worker_data.timestamps.end(),
                worker_data.points_local[i].timestamp,
                [](std::pair<double, double> lhs, double rhs) -> bool
                {
                    return lhs.first < rhs;
                });

            int index_pose = std::distance(worker_data.timestamps.begin(), lower) - 1;

            if (index_pose >= 0 && index_pose < worker_data.poses.size())
            {
                worker_data.points_local[i].index_pose = index_pose;
            }
            else
            {
                worker_data.points_local[i].index_pose = -1;
            }
        }

        NDT::GridParameters rgd_params;
        // rgd_params.resolution_X = 0.3; // distance bucket
        // rgd_params.resolution_Y = 0.3; // polar angle deg
        // rgd_params.resolution_Z = 0.3; // azimutal angle deg

        rgd_params.resolution_X = distance_bucket; // distance bucket
        rgd_params.resolution_Y = polar_angle_deg; // polar angle deg
        rgd_params.resolution_Z = azimutal_angle_deg; // azimutal angle deg

        std::vector<Point3Di> point_cloud_global;
        std::vector<Point3Di> points_local;

        std::vector<Eigen::Vector3d> point_cloud_global_sc;
        std::vector<Point3Di> points_local_sc;

        for (size_t i = 0; i < worker_data.points_local.size(); i++)
        {
            double r_l = worker_data.points_local[i].point.norm();
            if (r_l > 0.5 && worker_data.points_local[i].index_pose != -1 && r_l < max_distance_lidar)
            {
                double polar_angle_deg_l =
                    atan2(worker_data.points_local[i].point.y(), worker_data.points_local[i].point.x()) / M_PI * 180.0;
                double azimutal_angle_deg_l = acos(worker_data.points_local[i].point.z() / r_l) / M_PI * 180.0;

                Eigen::Vector3d pp = worker_data.points_local[i].point;
                // pps.x() = r;
                // pps.y() = polar_angle_deg;
                // pps.z() = azimutal_angle_deg;
                // point_cloud_spherical_coordinates.push_back(pps);

                Eigen::Affine3d pose = worker_data.poses[worker_data.points_local[i].index_pose];

                pp = pose * pp;

                Point3Di pg = worker_data.points_local[i];
                pg.point = pp;

                point_cloud_global.push_back(pg);
                points_local.push_back(worker_data.points_local[i]);

                ///////////////////////////////////////////////////////
                Point3Di p_sl = worker_data.points_local[i];
                p_sl.point.x() = r_l;
                p_sl.point.y() = polar_angle_deg_l;
                p_sl.point.z() = azimutal_angle_deg_l;

                points_local_sc.push_back(p_sl);
                //
                double r_g = pg.point.norm();
                double polar_angle_deg_g = atan2(pg.point.y(), pg.point.x()) / M_PI * 180.0;
                double azimutal_angle_deg_g = acos(pg.point.z() / r_l) / M_PI * 180.0;

                Eigen::Vector3d p_sg = worker_data.points_local[i].point;
                p_sg.x() = r_g;
                p_sg.y() = polar_angle_deg_g;
                p_sg.z() = azimutal_angle_deg_g;

                point_cloud_global_sc.push_back(p_sg);
            }
        }

        NDTBucketMapType buckets;
        update_rgd_spherical_coordinates(rgd_params, buckets, point_cloud_global, point_cloud_global_sc);

        /////////////
        // std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> nn;
        Eigen::Vector3d b(rgd_params.resolution_X, rgd_params.resolution_Y, rgd_params.resolution_Z);

        for (size_t i = 0; i < point_cloud_global_sc.size(); i++)
        {
            auto index_of_bucket = get_rgd_index_3d(point_cloud_global_sc[i], b);

            auto bucket_it = buckets.find(index_of_bucket);

            if (bucket_it != buckets.end())
            {
                auto& this_bucket = bucket_it->second;
                this_bucket.number_of_points++;
                // const auto &curr_mean = points_global[i].point;
                const auto& mean = this_bucket.mean;

                nn.emplace_back(point_cloud_global[i].point, mean);

                //////////////
            }
        }
    }
    return nn;
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> get_mean_cov()
{
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> mc;
    if (index_rendered_points_local >= 0 && index_rendered_points_local < all_data.size())
    {
        // for (size_t i = 0; i < all_data[index_rendered_points_local].poses.size(); i++){
        //     all_data[index_rendered_points_local].poses[i](0, 3) += 1.0;
        // }

        auto worker_data = all_data[index_rendered_points_local];

        // index data
        for (size_t i = 0; i < worker_data.points_local.size(); i++)
        {
            auto lower = std::lower_bound(
                worker_data.timestamps.begin(),
                worker_data.timestamps.end(),
                worker_data.points_local[i].timestamp,
                [](std::pair<double, double> lhs, double rhs) -> bool
                {
                    return lhs.first < rhs;
                });

            int index_pose = std::distance(worker_data.timestamps.begin(), lower) - 1;

            if (index_pose >= 0 && index_pose < worker_data.poses.size())
                worker_data.points_local[i].index_pose = index_pose;
            else
                worker_data.points_local[i].index_pose = -1;
        }

        NDT::GridParameters rgd_params;
        // rgd_params.resolution_X = 0.3; // distance bucket
        // rgd_params.resolution_Y = 0.3; // polar angle deg
        // rgd_params.resolution_Z = 0.3; // azimutal angle deg

        rgd_params.resolution_X = distance_bucket; // distance bucket
        rgd_params.resolution_Y = polar_angle_deg; // polar angle deg
        rgd_params.resolution_Z = azimutal_angle_deg; // azimutal angle deg

        std::vector<Point3Di> point_cloud_global;
        std::vector<Point3Di> points_local;

        std::vector<Eigen::Vector3d> point_cloud_global_sc;
        std::vector<Point3Di> points_local_sc;

        for (size_t i = 0; i < worker_data.points_local.size(); i++)
        {
            double r_l = worker_data.points_local[i].point.norm();
            if (r_l > 0.5 && worker_data.points_local[i].index_pose != -1 && r_l < max_distance_lidar)
            {
                double polar_angle_deg_l =
                    atan2(worker_data.points_local[i].point.y(), worker_data.points_local[i].point.x()) / M_PI * 180.0;
                double azimutal_angle_deg_l = acos(worker_data.points_local[i].point.z() / r_l) / M_PI * 180.0;

                Eigen::Vector3d pp = worker_data.points_local[i].point;
                // pps.x() = r;
                // pps.y() = polar_angle_deg;
                // pps.z() = azimutal_angle_deg;
                // point_cloud_spherical_coordinates.push_back(pps);

                Eigen::Affine3d pose = worker_data.poses[worker_data.points_local[i].index_pose];

                pp = pose * pp;

                Point3Di pg = worker_data.points_local[i];
                pg.point = pp;

                point_cloud_global.push_back(pg);
                points_local.push_back(worker_data.points_local[i]);

                ///////////////////////////////////////////////////////
                Point3Di p_sl = worker_data.points_local[i];
                p_sl.point.x() = r_l;
                p_sl.point.y() = polar_angle_deg_l;
                p_sl.point.z() = azimutal_angle_deg_l;

                points_local_sc.push_back(p_sl);
                //
                double r_g = pg.point.norm();
                double polar_angle_deg_g = atan2(pg.point.y(), pg.point.x()) / M_PI * 180.0;
                double azimutal_angle_deg_g = acos(pg.point.z() / r_l) / M_PI * 180.0;

                Eigen::Vector3d p_sg = worker_data.points_local[i].point;
                p_sg.x() = r_g;
                p_sg.y() = polar_angle_deg_g;
                p_sg.z() = azimutal_angle_deg_g;

                point_cloud_global_sc.push_back(p_sg);
            }
        }

        NDTBucketMapType buckets;
        update_rgd_spherical_coordinates(rgd_params, buckets, point_cloud_global, point_cloud_global_sc);

        /////////////
        Eigen::Vector3d b(rgd_params.resolution_X, rgd_params.resolution_Y, rgd_params.resolution_Z);

        for (const auto& b : buckets)
        {
            auto& this_bucket = b.second;

            mc.emplace_back(this_bucket.mean, this_bucket.cov);
        }
    }

    return mc;
}

void loadFiles(std::vector<std::string> input_file_names)
{
    LidarOdometryParams params; // dummy for load_data function
    params.save_calibration_validation = false;
    params.filter_threshold_xy_inner = filter_threshold_xy_inner;
    params.filter_threshold_xy_outer = filter_threshold_xy_outer;

    std::vector<std::vector<Point3Di>> pointsPerFile;
    std::vector<std::tuple<std::pair<double, double>, FusionVector, FusionVector>> imu_data;

    // no files selected, quit loading
    if (input_file_names.empty())
        return;

    if (load_data(input_file_names, params, pointsPerFile, imu_data, false))
    {
        // clear possible previous data

        all_data.clear();
        all_data.shrink_to_fit();

        imu_data_plot.timestampLidar.clear();
        imu_data_plot.angX.clear();
        imu_data_plot.angY.clear();
        imu_data_plot.angZ.clear();
        imu_data_plot.accX.clear();
        imu_data_plot.accY.clear();
        imu_data_plot.accZ.clear();
        imu_data_plot.yaw.clear();
        imu_data_plot.pitch.clear();
        imu_data_plot.roll.clear();

        imu_data_plot.timestampLidar.shrink_to_fit();
        imu_data_plot.angX.shrink_to_fit();
        imu_data_plot.angY.shrink_to_fit();
        imu_data_plot.angZ.shrink_to_fit();
        imu_data_plot.accX.shrink_to_fit();
        imu_data_plot.accY.shrink_to_fit();
        imu_data_plot.accZ.shrink_to_fit();
        imu_data_plot.yaw.shrink_to_fit();
        imu_data_plot.pitch.shrink_to_fit();
        imu_data_plot.roll.shrink_to_fit();

        photo_files_ts.clear();

        laz_files.clear();

        // specific processing for RAW data viewer

        std::sort(input_file_names.begin(), input_file_names.end());
        for (const auto& fileName : input_file_names)
        {
            std::string ext = fs::path(fileName).extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

            if (ext == ".laz" || ext == ".las")
                laz_files.push_back(fileName);
            else if (ext == ".jpg" || ext == ".jpeg")
            {
                photos_files.push_back(fileName);
                // decode filename e.g.: ` cam0_1761264773592270949.jpg`
                const std::string filename = fs::path(fileName).stem().string();
                std::string cam_id = filename.substr(0, filename.find("_"));
                std::string timestamp = filename.substr(filename.find("_") + 1, filename.size());

                std::cout << "cam_id: " << cam_id << std::endl;
                std::cout << "timestamp: " << timestamp << std::endl;
                std::cout << "filename: " << filename << std::endl;

                if (cam_id == "cam0" && !timestamp.empty())
                {
                    try
                    {
                        uint64_t ts = std::stoull(timestamp);
                        photo_files_ts[ts] = fileName;
                    } catch (const std::exception& e)
                    {
                        std::cerr << "Error parsing timestamp from filename: " << filename << " - " << e.what() << std::endl;
                    }
                }
            }
        }

        // rest of RAW data viewer processing

        FusionAhrs ahrs;
        FusionAhrsInitialise(&ahrs);

        if (fusionConventionNwu)
            ahrs.settings.convention = FusionConventionNwu;

        if (fusionConventionEnu)
            ahrs.settings.convention = FusionConventionEnu;

        if (fusionConventionNed)
            ahrs.settings.convention = FusionConventionNed;

        ahrs.settings.gain = ahrs_gain;

        std::map<double, std::pair<Eigen::Matrix4d, double>> trajectory;

        int counter = 1;
        static bool first = true;

        static double last_ts;

        for (const auto& [timestamp_pair, gyr, acc] : imu_data)
        {
            const FusionVector gyroscope = { static_cast<float>(gyr.axis.x * 180.0 / M_PI),
                                             static_cast<float>(gyr.axis.y * 180.0 / M_PI),
                                             static_cast<float>(gyr.axis.z * 180.0 / M_PI) };
            // const FusionVector gyroscope = {static_cast<float>(gyr.axis.x), static_cast<float>(gyr.axis.y),
            // static_cast<float>(gyr.axis.z)};
            const FusionVector accelerometer = { acc.axis.x, acc.axis.y, acc.axis.z };

            if (first)
            {
                FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
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
                    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
                }*/
            }

            last_ts = timestamp_pair.first;
            //

            FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);

            Eigen::Quaterniond d{ quat.element.w, quat.element.x, quat.element.y, quat.element.z };
            Eigen::Affine3d t{ Eigen::Matrix4d::Identity() };
            t.rotate(d);

            trajectory[timestamp_pair.first] = std::pair(t.matrix(), timestamp_pair.second);
            const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
            counter++;
            if (counter % 100 == 0)
            {
                std::cout << "Roll " << euler.angle.roll << ", Pitch " << euler.angle.pitch << ", Yaw " << euler.angle.yaw << " ["
                          << counter++ << " of " << imu_data.size() << "]" << std::endl;
            }

            // log it for implot
            imu_data_plot.timestampLidar.push_back(timestamp_pair.first);
            imu_data_plot.angX.push_back(gyr.axis.x);
            imu_data_plot.angY.push_back(gyr.axis.y);
            imu_data_plot.angZ.push_back(gyr.axis.z);
            imu_data_plot.accX.push_back(acc.axis.x);
            imu_data_plot.accY.push_back(acc.axis.y);
            imu_data_plot.accZ.push_back(acc.axis.z);
            imu_data_plot.yaw.push_back(euler.angle.yaw);
            imu_data_plot.pitch.push_back(euler.angle.pitch);
            imu_data_plot.roll.push_back(euler.angle.roll);
        }

        std::vector<std::pair<double, double>> timestamps;
        std::vector<Eigen::Affine3d> poses;
        for (const auto& t : trajectory)
        {
            timestamps.emplace_back(t.first, t.second.second);
            Eigen::Affine3d m;
            m.matrix() = t.second.first;
            poses.push_back(m);
        }

        int number_of_points = 0;
        for (const auto& pp : pointsPerFile)
            number_of_points += pp.size();

        std::cout << "Number of points: " << number_of_points << std::endl;

        std::cout << "Start indexing points" << std::endl;

        // std::vector<Point3Di> points_global;
        std::vector<Point3Di> points_local;
        std::vector<int> lidar_ids;

        Eigen::Affine3d m_prev;
        Eigen::Affine3d m_next;

        Eigen::Quaterniond q_prev;
        Eigen::Quaterniond q_next;
        Eigen::Quaterniond q;

        double time_prev;
        double time_next;
        double curr_time;

        double t;

        for (size_t i = 0; i < pointsPerFile.size(); i++)
        {
            std::cout << "Indexed: " << i + 1 << " of " << pointsPerFile.size() << " files\r";
            for (const auto& pp : pointsPerFile[i])
            {
                auto lower = std::lower_bound(
                    timestamps.begin(),
                    timestamps.end(),
                    pp.timestamp,
                    [](std::pair<double, double> lhs, double rhs) -> bool
                    {
                        return lhs.first < rhs;
                    });

                int index_pose = std::distance(timestamps.begin(), lower) - 1;

                if (index_pose >= 0 && index_pose < poses.size())
                {
                    auto ppp = pp;
                    // Eigen::Affine3d m = poses[index_pose];
                    /*if (is_slerp)
                    {
                        if (index_pose > 0)
                        {
                            m_prev = poses[index_pose - 1];
                            m_next = poses[index_pose];

                            q_prev = Eigen::Quaterniond(m_prev.rotation());
                            q_next = Eigen::Quaterniond(m_next.rotation());

                            time_prev = timestamps[index_pose - 1].first;
                            time_next = timestamps[index_pose].first;
                            curr_time = ppp.timestamp;

                            t = (curr_time - time_prev) / (time_next - time_prev);
                            q = q_prev.slerp(t, q_next);

                            m.linear() = q.toRotationMatrix();
                        }
                    }*/

                    points_local.push_back(ppp);

                    // ppp.point = m * ppp.point;
                    lidar_ids.push_back(pp.lidarid);
                    // points_global.push_back(ppp);
                }

                if (points_local.size() > number_of_points_threshold)
                {
                    // all_points_local.push_back(points_global);
                    indexes_to_filename.push_back(i);
                    // points_global.clear();
                    // all_lidar_ids.push_back(lidar_ids);

                    ///////////////////////////////////////
                    AllData data;
                    data.points_local = points_local;
                    data.lidar_ids = lidar_ids;

                    for (size_t i = 0; i < timestamps.size(); i++)
                    {
                        if (timestamps[i].first >= points_local[0].timestamp &&
                            timestamps[i].first <= points_local[points_local.size() - 1].timestamp)
                        {
                            data.timestamps.push_back(timestamps[i]);
                            data.poses.push_back(poses[i]);
                        }
                    }

                    // correct points timestamps
                    if (data.timestamps.size() > 2)
                    {
                        double ts_begin = data.timestamps[0].first;
                        double ts_step =
                            (data.timestamps[data.timestamps.size() - 1].first - data.timestamps[0].first) / data.points_local.size();

                        // std::cout << "ts_begin " << ts_begin << std::endl;
                        // std::cout << "ts_step " << ts_step << std::endl;
                        // std::cout << "ts_end " << data.timestamps[data.timestamps.size() - 1].first << std::endl;

                        for (size_t pp = 0; pp < data.points_local.size(); pp++)
                            data.points_local[pp].timestamp = ts_begin + pp * ts_step;
                    }

                    all_data.push_back(data);

                    points_local.clear();
                    lidar_ids.clear();
                    //////////////////////////////////////
                }
            }
        }

        std::cout << "\nIndexing points finished\n\n";

        if (all_data.size() > 0)
        {
            is_init = true;
            index_rendered_points_local = 0;
        }
    }
}

void openFolder()
{
    std::string input_folder_name;
    std::vector<std::string> input_file_names;
    input_folder_name = mandeye::fd::SelectFolder("Select Mandeye data folder");

    std::cout << "Selected folder: '" << input_folder_name << std::endl;

    if (fs::exists(input_folder_name))
    {
        for (const auto& entry : fs::directory_iterator(input_folder_name))
            if (entry.is_regular_file())
                input_file_names.push_back(entry.path().string());

        loadFiles(input_file_names);
    }
}

void openFiles()
{
    std::vector<std::string> input_file_names = mandeye::fd::OpenFileDialog("Load all files", mandeye::fd::All_Filter, true);

    loadFiles(input_file_names);
}

void imu_data_gui()
{
    ImGui::Begin("IMU data", &show_imu_data);
    {
        if (imu_data_plot.timestampLidar.size() > 0)
        {
            static double x_min = imu_data_plot.timestampLidar.front();
            static double x_max = x_min + 20.0;
            double annotation = 0;
            if (index_rendered_points_local >= 0 && index_rendered_points_local < all_data.size())
            {
                if (all_data[index_rendered_points_local].timestamps.size() > 0)
                    annotation = all_data[index_rendered_points_local].timestamps.front().first;
            }
            if (ImPlot::BeginPlot("IMU - acceleration 'm/s^2", ImVec2(-1, 0)))
            {
                ImPlot::SetupAxisLimits(ImAxis_X1, x_min, x_max, ImGuiCond_Once);
                ImPlot::SetupAxisLinks(ImAxis_X1, &x_min, &x_max);
                ImPlot::PlotLine(
                    "accX", imu_data_plot.timestampLidar.data(), imu_data_plot.accX.data(), (int)imu_data_plot.timestampLidar.size());
                ImPlot::PlotLine(
                    "accY", imu_data_plot.timestampLidar.data(), imu_data_plot.accY.data(), (int)imu_data_plot.timestampLidar.size());
                ImPlot::PlotLine(
                    "accZ", imu_data_plot.timestampLidar.data(), imu_data_plot.accZ.data(), (int)imu_data_plot.timestampLidar.size());
                ImPlot::TagX(annotation, ImVec4(1, 0, 0, 1));
                ImPlot::EndPlot();
            }

            if (ImPlot::BeginPlot("Imu - gyro", ImVec2(-1, 0)))
            {
                ImPlot::SetupAxisLimits(ImAxis_X1, x_min, x_max, ImGuiCond_Once);
                ImPlot::SetupAxisLinks(ImAxis_X1, &x_min, &x_max);
                ImPlot::PlotLine(
                    "angX", imu_data_plot.timestampLidar.data(), imu_data_plot.angX.data(), (int)imu_data_plot.timestampLidar.size());
                ImPlot::PlotLine(
                    "angY", imu_data_plot.timestampLidar.data(), imu_data_plot.angY.data(), (int)imu_data_plot.timestampLidar.size());
                ImPlot::PlotLine(
                    "angZ", imu_data_plot.timestampLidar.data(), imu_data_plot.angZ.data(), (int)imu_data_plot.timestampLidar.size());
                ImPlot::TagX(annotation, ImVec4(1, 0, 0, 1));
                ImPlot::EndPlot();
            }

            if (ImPlot::BeginPlot("IMU - AHRS angles [deg]", ImVec2(-1, 0)))
            {
                ImPlot::SetupAxisLimits(ImAxis_X1, x_min, x_max, ImGuiCond_Once);
                ImPlot::SetupAxisLinks(ImAxis_X1, &x_min, &x_max);
                ImPlot::PlotLine(
                    "yaw", imu_data_plot.timestampLidar.data(), imu_data_plot.yaw.data(), (int)imu_data_plot.timestampLidar.size());
                ImPlot::PlotLine(
                    "pitch", imu_data_plot.timestampLidar.data(), imu_data_plot.pitch.data(), (int)imu_data_plot.timestampLidar.size());
                ImPlot::PlotLine(
                    "roll", imu_data_plot.timestampLidar.data(), imu_data_plot.roll.data(), (int)imu_data_plot.timestampLidar.size());
                ImPlot::TagX(annotation, ImVec4(1, 0, 0, 1));
                ImPlot::EndPlot();
            }

            if (!photo_files_ts.empty() && ImPlot::BeginPlot("Photos"))
            {
                ImPlot::SetupAxisLimits(ImAxis_X1, x_min, x_max, ImGuiCond_Once);
                ImPlot::SetupAxisLinks(ImAxis_X1, &x_min, &x_max);
                // plot photos timestamps
                std::vector<double> photo_timestamps;
                std::vector<double> dummy;
                for (const auto& [ts, fn] : photo_files_ts)
                {
                    photo_timestamps.push_back(static_cast<double>(ts) / 1e9);
                    dummy.push_back(0.0);
                }
                ImPlot::PlotScatter("photos", photo_timestamps.data(), dummy.data(), (int)photo_timestamps.size());
                ImPlot::TagX(annotation, ImVec4(1, 0, 0, 1));
                ImPlot::EndPlot();
            }
        }

        if (ImGui::Button("Save 'ts gyr_x gyr_y gyr_z acc_x acc_y acc_z yaw_rad pitch_rad roll_rad' to csv"))
        {
            std::string output_file_name = "";
            output_file_name = mandeye::fd::SaveFileDialog("Save IMU data", {}, "");
            std::cout << "file to save: '" << output_file_name << "'" << std::endl;

            ofstream file;
            file.open(output_file_name);
            file << std::setprecision(20);
            for (size_t i = 0; i < imu_data_plot.timestampLidar.size(); i++)
            {
                file << imu_data_plot.timestampLidar[i] << " " << imu_data_plot.angX[i] << " " << imu_data_plot.angY[i] << " "
                     << imu_data_plot.angZ[i] << " " << imu_data_plot.accX[i] << " " << imu_data_plot.accY[i] << " "
                     << imu_data_plot.accZ[i] << " " << imu_data_plot.yaw[i] << " " << imu_data_plot.pitch[i] << " "
                     << imu_data_plot.roll[i] << std::endl;
            }

            file.close();
        }
    }

    ImGui::End();
}

void settings_gui()
{
    if (ImGui::Begin("Settings", &is_settings_gui))
    {
        ImGui::PushItemWidth(ImGuiNumberWidth);
        if (!is_init)
        {
            ImGui::InputInt("number_of_points_threshold", &number_of_points_threshold);
            if (number_of_points_threshold < 0)
                number_of_points_threshold = 0;
        }

        ImGui::InputDouble("AHRS gain", &ahrs_gain);
        if (ImGui::IsItemHovered())
        {
            ImGui::BeginTooltip();
            ImGui::Text("Parameter for AHRS (Attitude and Heading Reference System) filter");
            ImGui::Text("Controls how strongly the filter corrects its orientation estimate");
            ImGui::Text("using accelerometer and magnetometer data, relative to the gyroscope integration");
            ImGui::EndTooltip();
        }

        // ImGui::Checkbox("is_slerp", &is_slerp);

        ImGui::PopItemWidth();

        ImGui::BeginDisabled(!is_init);
        if (ImGui::Button("Save point cloud"))
        {
            std::string output_file_name = "";
            output_file_name = mandeye::fd::SaveFileDialog("Save las or laz file", mandeye::fd::LAS_LAZ_filter);

            if (output_file_name.size() > 0)
            {
                std::cout << "las or laz file to save: '" << output_file_name << "'" << std::endl;

                std::vector<Eigen::Vector3d> pointcloud;
                std::vector<unsigned short> intensity;
                std::vector<double> timestamps;

                /*if (index_rendered_points_local >= 0 && index_rendered_points_local < all_points_local.size())
                {
                    for (size_t i = 0; i < all_points_local[index_rendered_points_local].size(); i++)
                    {
                        pointcloud.emplace_back(all_points_local[index_rendered_points_local][i].point.x(),
                all_points_local[index_rendered_points_local][i].point.y(), all_points_local[index_rendered_points_local][i].point.z());
                        intensity.push_back(all_points_local[index_rendered_points_local][i].intensity);
                        timestamps.push_back(all_points_local[index_rendered_points_local][i].timestamp);
                    }
                }*/

                if (!exportLaz(output_file_name, pointcloud, intensity, timestamps, 0, 0, 0))
                    std::cout << "problem with saving file: " << output_file_name << std::endl;
            }
        }
        ImGui::EndDisabled();

        ImGui::Separator();

        ImGui::PushItemWidth(ImGuiNumberWidth);
        ImGui::InputDouble("Distance bucket [m]", &distance_bucket);
        ImGui::InputDouble("Polar angle [deg]", &polar_angle_deg);
        ImGui::InputDouble("Azimutal angle [deg]", &azimutal_angle_deg);
        ImGui::InputDouble("Max distance LiDAR", &max_distance_lidar);
        ImGui::InputInt("robust_and_accurate_lidar_odometry_iterations", &robust_and_accurate_lidar_odometry_iterations);
        ImGui::Checkbox("Use multithread", &useMultithread);
        ImGui::Text("Pose component weights for optimization:");
        if (ImGui::IsItemHovered())
        {
            ImGui::BeginTooltip();
            ImGui::Text("Value guideline:");
            ImGui::Text("1e6: almost rigid motion, prior dominates (default)");
            ImGui::Text("1e4: small corrections allowed");
            ImGui::Text("1e2: free alignment");
            ImGui::Text("  0: motion model ignored");
            ImGui::EndTooltip();
        }
        ImGui::InputDouble("wx", &wx);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Translation penaltie for %s", xText);
        ImGui::InputDouble("wy", &wy);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Translation penaltie for %s", yText);
        ImGui::InputDouble("wz", &wz);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Translation penaltie for %s", zText);
        ImGui::InputDouble("wom", &wom);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(omText);
        ImGui::InputDouble("wfi", &wfi);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(fiText);
        ImGui::InputDouble("wka", &wka);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(kaText);
        ImGui::PopItemWidth();

        ImGui::NewLine();

        if (ImGui::Button("Optimize"))
            optimize();

        ImGui::Separator();

        if (ImGui::Button("Print to console debug text"))
        {
            double max_diff = 0;
            std::cout << std::setprecision(20);
            if (index_rendered_points_local >= 0 && index_rendered_points_local < all_data.size())
            {
                for (size_t i = 0; i < all_data[index_rendered_points_local].points_local.size(); i++)
                {
                    auto lower = std::lower_bound(
                        all_data[index_rendered_points_local].timestamps.begin(),
                        all_data[index_rendered_points_local].timestamps.end(),
                        all_data[index_rendered_points_local].points_local[i].timestamp,
                        [](std::pair<double, double> lhs, double rhs) -> bool
                        {
                            return lhs.first < rhs;
                        });

                    int index_pose = std::distance(all_data[index_rendered_points_local].timestamps.begin(), lower) - 1;

                    if (index_pose >= 0 && index_pose < all_data[index_rendered_points_local].poses.size())
                    {
                        std::cout << index_pose << " total nr of poses: [" << all_data[index_rendered_points_local].poses.size() << "] "
                                  << all_data[index_rendered_points_local].points_local[i].timestamp -
                                all_data[index_rendered_points_local].timestamps[index_pose].first
                                  << " ts point: " << all_data[index_rendered_points_local].points_local[i].timestamp
                                  << " pose ts: " << all_data[index_rendered_points_local].timestamps[index_pose].first << std::endl;

                        if (fabs(
                                all_data[index_rendered_points_local].points_local[i].timestamp -
                                all_data[index_rendered_points_local].timestamps[index_pose].first) > max_diff)
                        {
                            // std::cout << all_data[index_rendered_points_local].points_local[i].timestamp << std::endl;
                            max_diff = fabs(
                                all_data[index_rendered_points_local].points_local[i].timestamp -
                                all_data[index_rendered_points_local].timestamps[index_pose].first);
                        }
                    }
                }
                std::cout << "max_diff " << max_diff << std::endl;
                std::cout << "----------------" << std::endl;
                for (size_t k = 0; k < all_data[index_rendered_points_local].timestamps.size(); k++)
                    std::cout << all_data[index_rendered_points_local].timestamps[k].first << std::endl;
            }
        }
    }

    ImGui::End();
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

    updateCameraTransition();

    viewLocal = Eigen::Affine3f::Identity();

    if (!is_ortho)
    {
        reshape((GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);

        viewLocal.translate(rotation_center);

        viewLocal.translate(Eigen::Vector3f(translate_x, translate_y, translate_z));
        if (!lock_z)
            viewLocal.rotate(Eigen::AngleAxisf(rotate_x * DEG_TO_RAD, Eigen::Vector3f::UnitX()));
        else
            viewLocal.rotate(Eigen::AngleAxisf(-90.0 * DEG_TO_RAD, Eigen::Vector3f::UnitX()));
        viewLocal.rotate(Eigen::AngleAxisf(rotate_y * DEG_TO_RAD, Eigen::Vector3f::UnitZ()));

        viewLocal.translate(-rotation_center);

        glLoadMatrixf(viewLocal.matrix().data());
    }
    else
        updateOrthoView();

    //
    /* glPointSize(point_size);
     glBegin(GL_POINTS);
     if (index_rendered_points_local >= 0 && index_rendered_points_local < all_points_local.size())
     {
         for (size_t i = 0; i < all_points_local[index_rendered_points_local].size(); i++)
         {
             if (all_lidar_ids[index_rendered_points_local][i] == 0)
             {
                 glColor3f(pc_color.x, pc_color.y, pc_color.z);
             }
             else
             {
                 glColor3f(pc_color2.x, pc_color2.y, pc_color2.z);
             }
             glVertex3f(all_points_local[index_rendered_points_local][i].point.x(),
     all_points_local[index_rendered_points_local][i].point.y(), all_points_local[index_rendered_points_local][i].point.z());
         }
     }
     glEnd();
     glPointSize(1);*/

    glPointSize(point_size);
    glBegin(GL_POINTS);
    if (index_rendered_points_local >= 0 && index_rendered_points_local < all_data.size())
    {
        double max_diff = 0.0;
        for (size_t i = 0; i < all_data[index_rendered_points_local].points_local.size(); i++)
        {
            auto lower = std::lower_bound(
                all_data[index_rendered_points_local].timestamps.begin(),
                all_data[index_rendered_points_local].timestamps.end(),
                all_data[index_rendered_points_local].points_local[i].timestamp,
                [](std::pair<double, double> lhs, double rhs) -> bool
                {
                    return lhs.first < rhs;
                });
            int index_pose = std::distance(all_data[index_rendered_points_local].timestamps.begin(), lower) - 1;
            if (index_pose >= 0 && index_pose < all_data[index_rendered_points_local].poses.size())
            {
                if (fabs(
                        all_data[index_rendered_points_local].points_local[i].timestamp -
                        all_data[index_rendered_points_local].timestamps[index_pose].first) > max_diff)
                {
                    // std::cout << index_pose << " " << all_data[index_rendered_points_local].points_local[i].timestamp -
                    // all_data[index_rendered_points_local].timestamps[index_pose].first << std::endl;
                    max_diff = fabs(
                        all_data[index_rendered_points_local].points_local[i].timestamp -
                        all_data[index_rendered_points_local].timestamps[index_pose].first);
                }
            }
        }

        for (size_t i = 0; i < all_data[index_rendered_points_local].points_local.size(); i++)
        {
            auto lower = std::lower_bound(
                all_data[index_rendered_points_local].timestamps.begin(),
                all_data[index_rendered_points_local].timestamps.end(),
                all_data[index_rendered_points_local].points_local[i].timestamp,
                [](std::pair<double, double> lhs, double rhs) -> bool
                {
                    return lhs.first < rhs;
                });

            int index_pose = std::distance(all_data[index_rendered_points_local].timestamps.begin(), lower) - 1;

            // std::cout << index_pose << std::endl;

            if (max_diff < 0.1)
            {
                if (index_pose >= 0 && index_pose < all_data[index_rendered_points_local].poses.size())
                {
                    // if (fabs(all_data[index_rendered_points_local].timestamps[index_pose].first -
                    // all_data[index_rendered_points_local].points_local[i].timestamp) < 0.001)
                    //{
                    Eigen::Affine3d m = all_data[index_rendered_points_local].poses[index_pose];
                    Eigen::Vector3d p = m * all_data[index_rendered_points_local].points_local[i].point;

                    if (all_data[index_rendered_points_local].lidar_ids[i] == 0)
                        glColor3f(pc_color.x, pc_color.y, pc_color.z);
                    else
                        glColor3f(pc_color2.x, pc_color2.y, pc_color2.z);

                    glVertex3f(p.x(), p.y(), p.z());
                    //}
                }
            }
        }
    }
    glEnd();
    glPointSize(1);

    if (index_rendered_points_local >= 0 && index_rendered_points_local < all_data.size())
    {
        for (size_t i = 0; i < all_data[index_rendered_points_local].points_local.size(); i++)
        {
            auto lower = std::lower_bound(
                all_data[index_rendered_points_local].timestamps.begin(),
                all_data[index_rendered_points_local].timestamps.end(),
                all_data[index_rendered_points_local].points_local[i].timestamp,
                [](std::pair<double, double> lhs, double rhs) -> bool
                {
                    return lhs.first < rhs;
                });

            int index_pose = std::distance(all_data[index_rendered_points_local].timestamps.begin(), lower) - 1;

            if (index_pose >= 0 && index_pose < all_data[index_rendered_points_local].poses.size())
            {
                /*Eigen::Affine3d m = all_data[index_rendered_points_local].poses[index_pose];
                Eigen::Vector3d p = m * all_data[index_rendered_points_local].points_local[i].point;

                if (all_data[index_rendered_points_local].lidar_ids[i] == 0)
                {
                    glColor3f(pc_color.x, pc_color.y, pc_color.z);
                }
                else
                {
                    glColor3f(pc_color2.x, pc_color2.y, pc_color2.z);
                }
                glVertex3f(p.x(), p.y(), p.z());*/

                Eigen::Affine3d m = all_data[index_rendered_points_local].poses[index_pose];
                glLineWidth(2.0);
                glBegin(GL_LINES);
                glColor3f(1.0f, 0.0f, 0.0f);
                glVertex3f(m(0, 3), m(1, 3), m(2, 3));
                glVertex3f(m(0, 3) + m(0, 0), m(1, 3) + m(1, 0), m(2, 3) + m(2, 0));

                glColor3f(0.0f, 1.0f, 0.0f);
                glVertex3f(m(0, 3), m(1, 3), m(2, 3));
                glVertex3f(m(0, 3) + m(0, 1), m(1, 3) + m(1, 1), m(2, 3) + m(2, 1));

                glColor3f(0.0f, 0.0f, 1.0f);
                glVertex3f(m(0, 3), m(1, 3), m(2, 3));
                glVertex3f(m(0, 3) + m(0, 2), m(1, 3) + m(1, 2), m(2, 3) + m(2, 2));
                glEnd();
                glLineWidth(1.0);
                break;
            }
        }
    }

    if (show_rgd_nn)
    {
        glColor3f(0, 0, 0);
        glBegin(GL_LINES);
        for (const auto& nn : rgd_nn)
        {
            glVertex3f(nn.first.x(), nn.first.y(), nn.first.z());
            glVertex3f(nn.second.x(), nn.second.y(), nn.second.z());
        }
        glEnd();
    }

    if (show_mean_cov)
    {
        for (const auto& mc : mean_cov)
            draw_ellipse(mc.second, mc.first, Eigen::Vector3f(1, 0, 0), 1);
    }

    showAxes();

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGLUT_NewFrame();
    ImGui::NewFrame();

    ShowMainDockSpace();

    view_kbd_shortcuts();

    if (all_data.size() > 0)
    {
        if ((!io.KeyCtrl && !io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_RightArrow, true)) || ImGui::IsKeyPressed(ImGuiKey_PageUp, true) ||
            ImGui::IsKeyPressed(ImGuiKey_KeypadAdd, true))
            index_rendered_points_local += 1;
        if ((!io.KeyCtrl && !io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_LeftArrow, true)) ||
            ImGui::IsKeyPressed(ImGuiKey_PageDown, true) || ImGui::IsKeyPressed(ImGuiKey_KeypadSubtract, true))
            index_rendered_points_local -= 1;

        if (index_rendered_points_local < 0)
            index_rendered_points_local = 0;
        if (index_rendered_points_local >= all_data.size())
            index_rendered_points_local = all_data.size() - 1;
    }

    // ask to load the nearest photo
    if (show_cameras_data && index_rendered_points_local >= 0 && index_rendered_points_local < indexes_to_filename.size())
    {
        double ts = all_data[index_rendered_points_local].points_local[0].timestamp; // * 1e9;
        render_nearest_photo(ts);
    }


    if (ImGui::BeginMainMenuBar())
    {
        {
            if (ImGui::Button("Open folder"))
                openFolder();
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Select folder containing files for analyze (Ctrl+O)");

            ImGui::SameLine();

            if (ImGui::ArrowButton("##menuArrow", ImGuiDir_Down))
                ImGui::OpenPopup("OpenMenu");

            if (ImGui::BeginPopup("OpenMenu"))
            {
                if (ImGui::MenuItem("Open files"))
                    openFiles();
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Select files for analyze");

                ImGui::EndPopup();
            }

            ImGui::SameLine();
            ImGui::Dummy(ImVec2(20, 0));
            ImGui::SameLine();
        }

        if (ImGui::BeginMenu("View"))
        {
            ImGui::SetNextItemWidth(ImGuiNumberWidth);
            ImGui::InputInt("Points size", &point_size);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("keyboard 1-9 keys");
            if (point_size < 1)
                point_size = 1;
            else if (point_size > 10)
                point_size = 10;

            ImGui::Separator();

            if (ImGui::MenuItem("Orthographic", "key O", &is_ortho))
            {
                if (is_ortho)
                {
                    new_rotation_center = rotation_center;
                    new_rotate_x = 0.0;
                    new_rotate_y = 0.0;
                    new_translate_x = translate_x;
                    new_translate_y = translate_y;
                    new_translate_z = translate_z;
                    camera_transition_active = true;
                }
            }

            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Switch between perspective view (3D) and orthographic view (2D/flat)");

            ImGui::MenuItem("Show axes", "key X", &show_axes);
            ImGui::MenuItem("Show compass/ruler", "key C", &compass_ruler);

            ImGui::MenuItem("Lock Z", "Shift + Z", &lock_z, !is_ortho);

            ImGui::Separator();

            ImGui::BeginDisabled(!is_init);
            if (ImGui::MenuItem("Show RGD / NN", nullptr, &show_rgd_nn))
            {
                if (show_rgd_nn)
                    rgd_nn = get_nn();
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Show Regular/Range Grid Data / Nearest Neighbors");

            if (ImGui::MenuItem("Show mean covs", nullptr, &show_mean_cov))
            {
                if (show_mean_cov)
                    mean_cov = get_mean_cov();
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Show the mean and covariance ellipsoid representation");

            ImGui::MenuItem("Show IMU data", nullptr, &show_imu_data);
            ImGui::MenuItem("Show cameras data", nullptr, &show_cameras_data);
            ImGui::EndDisabled();

            ImGui::Separator();

            if (ImGui::BeginMenu("Colors"))
            {
                ImGui::ColorEdit3("Background", (float*)&bg_color, ImGuiColorEditFlags_NoInputs);
                ImGui::ColorEdit3("Point cloud 1", (float*)&pc_color, ImGuiColorEditFlags_NoInputs);
                ImGui::ColorEdit3("Point cloud 2", (float*)&pc_color2, ImGuiColorEditFlags_NoInputs);

                ImGui::EndMenu();
            }

            ImGui::Separator();

            ImGui::MenuItem("Settings", nullptr, &is_settings_gui);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Show power user settings window with more parameters");

            ImGui::EndMenu();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Scene view relevant parameters");

        camMenu();

        ImGui::SameLine();
        ImGui::Dummy(ImVec2(20, 0));
        ImGui::SameLine();

        if (all_data.size() > 0)
        {
            int tempIndex = index_rendered_points_local;
            ImGui::Text("Index: ");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Index of rendered points local");
            ImGui::SameLine();
            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::SliderInt("##irpls", &tempIndex, 0, static_cast<int>(all_data.size() - 1));
            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                std::string fn = laz_files[indexes_to_filename[index_rendered_points_local]];
                ImGui::Text(fn.c_str());
                ImGui::EndTooltip();
            }
            ImGui::SameLine();
            ImGui::InputInt("##irpli", &tempIndex, 1, 10);
            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                std::string fn = laz_files[indexes_to_filename[index_rendered_points_local]];
                ImGui::Text(fn.c_str());
                ImGui::EndTooltip();
            }
            ImGui::PopItemWidth();

            if ((tempIndex >= 0) && (tempIndex < all_data.size()))
                index_rendered_points_local = tempIndex;

            ImGui::SameLine();
            double ts = all_data[index_rendered_points_local].points_local[0].timestamp; // * 1e9;
            ImGui::Text("Timestamp: %.6f [ns]", ts);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Click to copy to clipboard");
            if (ImGui::IsItemClicked())
            {
                char tmp[64];
                snprintf(tmp, sizeof(tmp), "%.6f", ts);
                ImGui::SetClipboardText(tmp);
            }

            ImGui::SameLine();
            ImGui::Dummy(ImVec2(20, 0));
            ImGui::SameLine();
        }

        ImGui::SameLine();
        ImGui::Text("(%.1f FPS)", ImGui::GetIO().Framerate);

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

    info_window(infoLines, appShortcuts);

    if (compass_ruler)
        drawMiniCompassWithRuler();

    if (show_cameras_data && photos::photo_texture_cam0)
    {
        if (ImGui::Begin("CAM0"))
        {
            using namespace photos;
            ImGui::Text("fn name: %s ", photos::imgToShowFn.c_str());
            ImGui::Text("timestamp:  %s", std::to_string(photos::nearestTs).c_str());
            // get available size
            DisplayImageFit((ImTextureID)photos::photo_texture_cam0, photo_width_cam0, photo_height_cam0);
        }

        ImGui::End();
    }

    if (show_imu_data)
        imu_data_gui();

    if (is_settings_gui)
        settings_gui();

    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

    glutSwapBuffers();
    glutPostRedisplay();
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
    {
        wheel(glut_button, glut_button == 3 ? 1 : -1, x, y);
    }

    if (!io.WantCaptureMouse)
    {
        if (glut_button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN && io.KeyCtrl)
        {
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

int main(int argc, char* argv[])
{
    try
    {
        if (checkClHelp(argc, argv))
        {
            std::cout << winTitle << "\n\n"
                      << "USAGE:\n"
                      << std::filesystem::path(argv[0]).stem().string() << " <input_folder> /?\n\n"
                      << "where\n"
                      << "   <input_folder>       Path where scan files are located (*.csv, *.laz, *.sn)\n"
                      << "   -h, /h, --help, /?   Show this help and exit\n\n";

            return 0;
        }

        initGL(&argc, argv, winTitle, display, mouse);
        ImPlot::CreateContext();

        if (argc == 2)
        {
            std::vector<std::string> input_file_names;

            if (fs::exists(argv[1]))
            {
                for (const auto& entry : fs::recursive_directory_iterator(argv[1]))
                    if (entry.is_regular_file())
                        input_file_names.push_back(entry.path().string());

                loadFiles(input_file_names);
            }
        }

        glutMainLoop();

        ImGui_ImplOpenGL2_Shutdown();
        ImGui_ImplGLUT_Shutdown();
        ImGui::DestroyContext();
        ImPlot::DestroyContext();
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
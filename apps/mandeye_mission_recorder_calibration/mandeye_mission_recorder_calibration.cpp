#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <ImGuizmo.h>
#include <imgui_internal.h>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <glm.hpp>
#include <gtc/matrix_transform.hpp>

#include <Eigen/Eigen>

#include <transformations.h>

#include <portable-file-dialogs.h>

#include <filesystem>
#include "../lidar_odometry_step_1/lidar_odometry_utils.h"

#include <HDMapping/Version.hpp>

#include <mutex>

#include <pair_wise_iterative_closest_point.h>

#define SAMPLE_PERIOD (1.0 / 200.0)
namespace fs = std::filesystem;

const unsigned int window_width = 800;
const unsigned int window_height = 600;
double camera_ortho_xy_view_zoom = 10;
double camera_ortho_xy_view_shift_x = 0.0;
double camera_ortho_xy_view_shift_y = 0.0;
double camera_mode_ortho_z_center_h = 0.0;
double camera_ortho_xy_view_rotation_angle_deg = 0;
bool is_ortho = false;
bool show_axes = true;
ImVec4 clear_color = ImVec4(0.8f, 0.8f, 0.8f, 1.00f);
ImVec4 pc_color = ImVec4(1.0f, 0.0f, 0.0f, 1.00f);
ImVec4 pc_color2 = ImVec4(0.0f, 0.0f, 1.0f, 1.00f);

int point_size = 1;
Eigen::Vector3f rotation_center = Eigen::Vector3f::Zero();
float translate_x, translate_y = 0.0;
float translate_z = -50.0;
float rotate_x = 0.0, rotate_y = 0.0;
int mouse_old_x, mouse_old_y;
int mouse_buttons = 0;
bool gui_mouse_down{false};
float mouse_sensitivity = 1.0;

float m_ortho_projection[] = {1, 0, 0, 0,
                              0, 1, 0, 0,
                              0, 0, 1, 0,
                              0, 0, 0, 1};

float m_ortho_gizmo_view[] = {1, 0, 0, 0,
                              0, 1, 0, 0,
                              0, 0, 1, 0,
                              0, 0, 0, 1};

std::vector<std::string> laz_files;
std::vector<std::string> csv_files;
std::vector<std::string> sn_files;
std::string working_directory = "";
std::string imuSnToUse;
std::string working_directory_preview;
double filter_threshold_xy = 0.1;
bool fusionConventionNwu = true;
bool fusionConventionEnu = false;
bool fusionConventionNed = false;
int number_of_points_threshold = 20000;
bool is_init = true;
int index_rendered_points_local = -1;
std::vector<std::vector<Eigen::Vector3d>> all_points_local;
std::vector<std::vector<int>> all_lidar_ids;
std::vector<int> indexes_to_filename;
std::vector<std::string> all_file_names;
LidarOdometryParams params;
int threshold_initial_points = 10000;
std::vector<WorkerData> worker_data;
const std::vector<std::string> json_filter = {"Calibration file (*.json)", "*.json", "All files", "*"};
const std::vector<std::string> LAS_LAZ_filter = {"LAS file (*.laz)", "*.laz", "LASzip file (*.las)", "*.las", "All files", "*"};
const std::vector<std::string> sn_filter = {"sn file (*.sn)", "*.sn", "All files", "*"};
std::unordered_map<std::string, Eigen::Affine3d> calibration;
std::vector<Point3Di> point_cloud;

std::unordered_map<int, std::string> idToSn;
std::unordered_map<int, Eigen::Affine3d> calibrations;

struct Checked
{
    bool check;
};
std::vector<Checked> calibrated_lidar;
std::vector<Checked> imu_lidar;

double search_radious = 0.1;

bool show_grid = true;
bool manual_calibration = false;

void reshape(int w, int h)
{
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if (!is_ortho)
    {
        gluPerspective(60.0, (GLfloat)w / (GLfloat)h, 0.01, 10000.0);
    }
    else
    {
        ImGuiIO &io = ImGui::GetIO();
        float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);

        glOrtho(-camera_ortho_xy_view_zoom, camera_ortho_xy_view_zoom,
                -camera_ortho_xy_view_zoom / ratio,
                camera_ortho_xy_view_zoom / ratio, -100000, 100000);
        // glOrtho(-translate_z, translate_z, -translate_z * (float)h / float(w), translate_z * float(h) / float(w), -10000, 10000);
    }
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

        if (is_ortho)
        {
            if (mouse_buttons & 1)
            {
                float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);
                Eigen::Vector3d v(dx * (camera_ortho_xy_view_zoom / (GLsizei)io.DisplaySize.x * 2),
                                  dy * (camera_ortho_xy_view_zoom / (GLsizei)io.DisplaySize.y * 2 / ratio), 0);
                TaitBryanPose pose_tb;
                pose_tb.px = 0.0;
                pose_tb.py = 0.0;
                pose_tb.pz = 0.0;
                pose_tb.om = 0.0;
                pose_tb.fi = 0.0;
                pose_tb.ka = camera_ortho_xy_view_rotation_angle_deg * M_PI / 180.0;
                auto m = affine_matrix_from_pose_tait_bryan(pose_tb);
                Eigen::Vector3d v_t = m * v;
                camera_ortho_xy_view_shift_x += v_t.x();
                camera_ortho_xy_view_shift_y += v_t.y();
            }
        }
        else
        {
            gui_mouse_down = mouse_buttons > 0;
            if (mouse_buttons & 1)
            {
                rotate_x += dy * 0.2f; // * mouse_sensitivity;
                rotate_y += dx * 0.2f; // * mouse_sensitivity;
            }
            if (mouse_buttons & 4)
            {
                translate_x += dx * 0.05f * mouse_sensitivity;
                translate_y -= dy * 0.05f * mouse_sensitivity;
            }
        }

        mouse_old_x = x;
        mouse_old_y = y;
    }
    glutPostRedisplay();
}

std::vector<Point3Di> load_pc(const std::string &lazFile, bool ommit_points_with_timestamp_equals_zero, double filter_threshold_xy)
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

    std::cout << "header->number_of_point_records:  " << header->number_of_point_records << std::endl;

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

        // if (!calibrations.empty())
        //{
        //     if (!calibrations.contains(id))
        //     {
        //         continue;
        //     }
        // }

        // Eigen::Affine3d calibration = calibrations.empty() ? Eigen::Affine3d::Identity() : calibrations.at(id);
        const Eigen::Vector3d pf(header->x_offset + header->x_scale_factor * static_cast<double>(point->X), header->y_offset + header->y_scale_factor * static_cast<double>(point->Y), header->z_offset + header->z_scale_factor * static_cast<double>(point->Z));

        p.point = /*calibration **/ (pf);
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

void project_gui()
{
    if (ImGui::Begin("main gui window"))
    {
        ImGui::ColorEdit3("clear color", (float *)&clear_color);

        if (idToSn.size() == 2)
        {
            ImGui::ColorEdit3(idToSn.at(0).c_str(), (float *)&pc_color);
            ImGui::ColorEdit3(idToSn.at(1).c_str(), (float *)&pc_color2);
        }
        else
        {
            ImGui::ColorEdit3("pc_color_lidar_1", (float *)&pc_color);
            ImGui::ColorEdit3("pc_color_lidar_2", (float *)&pc_color2);
        }

        ImGui::Checkbox("show_grid", &show_grid);
        ImGui::Checkbox("show_axes", &show_axes);

        if (calibrated_lidar.size() == 2)
        {
            ImGui::Text(".......... Check calibrated lidar ..............");

            int chosen_lidar = -1;
            for (int i = 0; i < calibrated_lidar.size(); i++)
            {
                std::string name = idToSn.at(i);
                bool before = calibrated_lidar[i].check;
                ImGui::Checkbox(name.c_str(), &calibrated_lidar[i].check);
                bool after = calibrated_lidar[i].check;

                if (!before && after)
                {
                    chosen_lidar = i;
                }
            }

            bool is_all_false = true;
            for (int i = 0; i < calibrated_lidar.size(); i++)
            {
                if (calibrated_lidar[i].check)
                {
                    is_all_false = false;
                }
            }

            if (is_all_false)
            {
                chosen_lidar = 0;
            }

            if (chosen_lidar != -1)
            {
                for (int i = 0; i < calibrated_lidar.size(); i++)
                {
                    calibrated_lidar[i].check = false;
                }
                calibrated_lidar[chosen_lidar].check = true;
            }

            ImGui::Text("------------------------------------------------");
            ImGui::Checkbox("manual_calibration", &manual_calibration);

            if (manual_calibration)
            {
                int index_calibrated_lidar = -1;
                for (int i = 0; i < calibrated_lidar.size(); i++)
                {
                    if (calibrated_lidar[i].check)
                    {
                        index_calibrated_lidar = i;
                    }
                }

                if (index_calibrated_lidar != -1)
                {
                    TaitBryanPose tb_pose = pose_tait_bryan_from_affine_matrix(calibrations.at(index_calibrated_lidar));
                    tb_pose.om = tb_pose.om * 180.0 / M_PI;
                    tb_pose.fi = tb_pose.fi * 180.0 / M_PI;
                    tb_pose.ka = tb_pose.ka * 180.0 / M_PI;

                    auto tmp = tb_pose;

                    ImGui::InputDouble(std::string(idToSn.at(index_calibrated_lidar) + "_x [m] (offset in X-red axis)").c_str(), &tb_pose.px, 0.01, 0.1);
                    ImGui::InputDouble(std::string(idToSn.at(index_calibrated_lidar) + "_y [m] (offset in Y-green axis)").c_str(), &tb_pose.py, 0.01, 0.1);
                    ImGui::InputDouble(std::string(idToSn.at(index_calibrated_lidar) + "_z [m] (offset in Z-blue axis)").c_str(), &tb_pose.pz, 0.01, 0.1);
                    ImGui::InputDouble(std::string(idToSn.at(index_calibrated_lidar) + "_om [deg] (angle around X-red axis)").c_str(), &tb_pose.om, 0.1, 1.0);
                    ImGui::InputDouble(std::string(idToSn.at(index_calibrated_lidar) + "_fi [deg] (angle around Y-green axis)").c_str(), &tb_pose.fi, 0.1, 1.0);
                    ImGui::InputDouble(std::string(idToSn.at(index_calibrated_lidar) + "_ka [deg] (angle around Z-blue axis)").c_str(), &tb_pose.ka, 0.1, 1.0);

                    if (tmp.px != tb_pose.px || tmp.py != tb_pose.py || tmp.pz != tb_pose.pz ||
                        tmp.om != tb_pose.om || tmp.fi != tb_pose.fi || tmp.ka != tb_pose.ka)
                    {
                        tb_pose.om = tb_pose.om * M_PI / 180.0;
                        tb_pose.fi = tb_pose.fi * M_PI / 180.0;
                        tb_pose.ka = tb_pose.ka * M_PI / 180.0;

                        Eigen::Affine3d m_pose = affine_matrix_from_pose_tait_bryan(tb_pose);
                        calibrations.at(index_calibrated_lidar) = m_pose;
                    }
                }

                // calibrations.at(0) = m0;
            }
            ImGui::Text("................................................");
        }

        if (idToSn.size() == 0)
        {
            if (ImGui::Button("Load 'LiDAR serial number to index' file (lidar****.sn) (step 1)"))
            {
                static std::shared_ptr<pfd::open_file> open_file;
                std::vector<std::string> input_file_names;
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
                const auto t = [&]()
                {
                    auto sel = pfd::open_file("Calibration files", "C:\\", sn_filter, true).result();
                    for (int i = 0; i < sel.size(); i++)
                    {
                        input_file_names.push_back(sel[i]);
                        // std::cout << "las file: '" << input_file_name << "'" << std::endl;
                    }
                };
                std::thread t1(t);
                t1.join();

                idToSn = MLvxCalib::GetIdToSnMapping(input_file_names[0]);
            }
        }

        if (idToSn.size() == 2 && calibrations.size() == 0)
        {
            if (ImGui::Button("Load calibration (*.json) (step 2)"))
            {
                static std::shared_ptr<pfd::open_file> open_file;
                std::vector<std::string> input_file_names;
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
                const auto t = [&]()
                {
                    auto sel = pfd::open_file("Calibration files", "C:\\", json_filter, true).result();
                    for (int i = 0; i < sel.size(); i++)
                    {
                        input_file_names.push_back(sel[i]);
                    }
                };
                std::thread t1(t);
                t1.join();

                if (input_file_names.size() > 0)
                {
                    std::cout << "loading file: " << input_file_names[0] << std::endl;

                    calibration = MLvxCalib::GetCalibrationFromFile(input_file_names[0]);
                    imuSnToUse = MLvxCalib::GetImuSnToUse(input_file_names[0]);

                    calibrations = MLvxCalib::CombineIntoCalibration(idToSn, calibration);

                    if (!calibration.empty())
                    {
                        std::cout << "Loaded calibration for: \n";
                        for (const auto &[sn, _] : calibration)
                        {
                            std::cout << " -> " << sn << std::endl;
                        }
                        std::cout << "imuSnToUse: " << imuSnToUse << std::endl;
                    }
                }
            }

            ImGui::SameLine();
            ImGui::Text(" if You dont have any calibration file --> ");
            ImGui::SameLine();
            if (ImGui::Button("Save default calibration (optional before step 2)"))
            {
                std::shared_ptr<pfd::save_file> save_file;
                std::string output_file_name = "";
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
                const auto t = [&]()
                {
                    auto sel = pfd::save_file("Save *.json file", "", json_filter).result();
                    output_file_name = sel;
                    std::cout << "Calibration file to save: '" << output_file_name << "'" << std::endl;
                };
                std::thread t1(t);
                t1.join();

                if (output_file_name.size() > 0)
                {
                    std::string l1 = idToSn.at(0);
                    std::string l2 = idToSn.at(1);

                    std::cout
                        << "output_file_name: " << output_file_name << std::endl;
                    nlohmann::json j;

                    j["calibration"][l1.c_str()]["identity"] = "true";
                    j["calibration"][l2.c_str()]["order"] = "ROW";
                    j["calibration"][l2.c_str()]["inverted"] = "FALSE";
                    j["calibration"][l2.c_str()]["data"][0] = 1;
                    j["calibration"][l2.c_str()]["data"][1] = 0;
                    j["calibration"][l2.c_str()]["data"][2] = 0;
                    j["calibration"][l2.c_str()]["data"][3] = 0;
                    j["calibration"][l2.c_str()]["data"][4] = 0;
                    j["calibration"][l2.c_str()]["data"][5] = 1;
                    j["calibration"][l2.c_str()]["data"][6] = 0;
                    j["calibration"][l2.c_str()]["data"][7] = 0;
                    j["calibration"][l2.c_str()]["data"][8] = 0;
                    j["calibration"][l2.c_str()]["data"][9] = 0;
                    j["calibration"][l2.c_str()]["data"][10] = 1;
                    j["calibration"][l2.c_str()]["data"][11] = 0;
                    j["calibration"][l2.c_str()]["data"][12] = 0;
                    j["calibration"][l2.c_str()]["data"][13] = 0;
                    j["calibration"][l2.c_str()]["data"][14] = 0;
                    j["calibration"][l2.c_str()]["data"][15] = 1;
                    j["imuToUse"] = l1.c_str();

                    std::ofstream fs(output_file_name);
                    if (!fs.good())
                        return;
                    fs << j.dump(2);
                    fs.close();
                }
            }
        }

        if (calibrations.size() > 0 && point_cloud.size() == 0)
        {
            if (ImGui::Button("Load pointcloud (lidar****.laz) (step 3)"))
            {
                static std::shared_ptr<pfd::open_file> open_file;
                std::vector<std::string> input_file_names;
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
                const auto t = [&]()
                {
                    auto sel = pfd::open_file("Point cloud files", "C:\\", LAS_LAZ_filter, true).result();
                    for (int i = 0; i < sel.size(); i++)
                    {
                        input_file_names.push_back(sel[i]);
                    }
                };
                std::thread t1(t);
                t1.join();

                if (input_file_names.size() > 0)
                {
                    for (int i = 0; i < input_file_names.size(); i++)
                    {
                        auto pc = load_pc(input_file_names[i].c_str(), true, filter_threshold_xy);

                        for (const auto &p : pc)
                        {
                            point_cloud.emplace_back(p);
                        }
                    }
                }

                Checked check;
                check.check = true;
                calibrated_lidar.push_back(check);
                imu_lidar.push_back(check);
                check.check = false;
                calibrated_lidar.push_back(check);
                imu_lidar.push_back(check);
            }
        }

        if (point_cloud.size() > 0)
        {
            std::string calibrated_lidar_name = "calibrate[";

            for (int i = 0; i < calibrated_lidar.size(); i++)
            {
                if (calibrated_lidar[i].check)
                {
                    calibrated_lidar_name += idToSn.at(i);
                }
            }

            calibrated_lidar_name += "] (step 4)";

            if (ImGui::Button(calibrated_lidar_name.c_str()))
            {
                int number_of_iterations = 10;
                PairWiseICP icp;

                Eigen::Affine3d m0 = calibrations.at(0);
                Eigen::Affine3d m1 = calibrations.at(1);

                std::vector<Point3Di> lidar0;
                std::vector<Point3Di> lidar1;

                for (const auto &p : point_cloud)
                {
                    if (p.lidarid == 0)
                    {
                        lidar0.emplace_back(p);
                    }
                    else
                    {
                        lidar1.emplace_back(p);
                    }
                }

                std::cout << "decimation: " << params.decimation << std::endl;
                std::cout << "point cloud size before" << std::endl;
                std::cout << "lidar0.size(): " << lidar0.size() << std::endl;
                std::cout << "lidar1.size(): " << lidar1.size() << std::endl;

                lidar0 = decimate(lidar0, params.decimation, params.decimation, params.decimation);
                lidar1 = decimate(lidar1, params.decimation, params.decimation, params.decimation);

                std::cout << "point cloud size after" << std::endl;
                std::cout << "lidar0.size(): " << lidar0.size() << std::endl;
                std::cout << "lidar1.size(): " << lidar1.size() << std::endl;

                std::vector<Eigen::Vector3d> pc0;
                std::vector<Eigen::Vector3d> pc1;

                if (calibrated_lidar[0].check)
                {
                    for (const auto &s : lidar0)
                    {
                        pc0.emplace_back(s.point.x(), s.point.y(), s.point.z());
                    }
                    for (const auto &t : lidar1)
                    {
                        auto pp = m1 * t.point;
                        pc1.emplace_back(pp.x(), pp.y(), pp.z());
                    }

                    if (icp.compute(pc0, pc1, search_radious, number_of_iterations, m0))
                    {
                        calibrations.at(0) = m0;
                    }
                }
                else
                {
                    for (const auto &s : lidar0)
                    {
                        auto pp = m0 * s.point;
                        pc0.emplace_back(pp.x(), pp.y(), pp.z());
                    }
                    for (const auto &t : lidar1)
                    {
                        pc1.emplace_back(t.point.x(), t.point.y(), t.point.z());
                    }
                    if (icp.compute(pc1, pc0, search_radious, number_of_iterations, m1))
                    {
                        calibrations.at(1) = m1;
                    }
                }
            }

            ImGui::SameLine();

            ImGui::InputDouble("search_radious: ", &search_radious);
            if (search_radious < 0.05)
            {
                search_radious = 0.05;
            }

            /**/

            //
            ImGui::Text("=========================================================================");

            if (imu_lidar.size() == 2)
            {
                ImGui::Text(".......... Check imu for lidar odometry in calibration file ..........");

                int chosen_imu = -1;
                for (int i = 0; i < imu_lidar.size(); i++)
                {
                    std::string name = idToSn.at(i);
                    bool before = imu_lidar[i].check;
                    ImGui::Checkbox(std::string(name + "_imu").c_str(), &imu_lidar[i].check);
                    bool after = imu_lidar[i].check;

                    if (!before && after)
                    {
                        chosen_imu = i;
                    }
                }

                bool is_all_imu_false = true;
                for (int i = 0; i < imu_lidar.size(); i++)
                {
                    if (imu_lidar[i].check)
                    {
                        is_all_imu_false = false;
                    }
                }

                if (is_all_imu_false)
                {
                    chosen_imu = 0;
                }

                if (chosen_imu != -1)
                {
                    for (int i = 0; i < imu_lidar.size(); i++)
                    {
                        imu_lidar[i].check = false;
                    }
                    imu_lidar[chosen_imu].check = true;
                }
            }

            if (ImGui::Button("Save result calibration as 'calibration.json' (step 5)"))
            {
                std::shared_ptr<pfd::save_file> save_file;
                std::string output_file_name = "";
                ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
                const auto t = [&]()
                {
                    auto sel = pfd::save_file("Save las or laz file", "C:\\", json_filter).result();
                    output_file_name = sel;
                    std::cout << "las or laz file to save: '" << output_file_name << "'" << std::endl;
                };
                std::thread t1(t);
                t1.join();

                if (output_file_name.size() > 0)
                {
                    std::cout << "output_file_name: " << output_file_name << std::endl;
                    nlohmann::json j;

                    j["calibration"][idToSn.at(0)]["order"] = "ROW";
                    j["calibration"][idToSn.at(0)]["inverted"] = "FALSE";
                    j["calibration"][idToSn.at(0)]["data"][0] = calibrations.at(0)(0, 0);
                    j["calibration"][idToSn.at(0)]["data"][1] = calibrations.at(0)(0, 1);
                    j["calibration"][idToSn.at(0)]["data"][2] = calibrations.at(0)(0, 2);
                    j["calibration"][idToSn.at(0)]["data"][3] = calibrations.at(0)(0, 3);
                    j["calibration"][idToSn.at(0)]["data"][4] = calibrations.at(0)(1, 0);
                    j["calibration"][idToSn.at(0)]["data"][5] = calibrations.at(0)(1, 1);
                    j["calibration"][idToSn.at(0)]["data"][6] = calibrations.at(0)(1, 2);
                    j["calibration"][idToSn.at(0)]["data"][7] = calibrations.at(0)(1, 3);
                    j["calibration"][idToSn.at(0)]["data"][8] = calibrations.at(0)(2, 0);
                    j["calibration"][idToSn.at(0)]["data"][9] = calibrations.at(0)(2, 1);
                    j["calibration"][idToSn.at(0)]["data"][10] = calibrations.at(0)(2, 2);
                    j["calibration"][idToSn.at(0)]["data"][11] = calibrations.at(0)(2, 3);
                    j["calibration"][idToSn.at(0)]["data"][12] = 0;
                    j["calibration"][idToSn.at(0)]["data"][13] = 0;
                    j["calibration"][idToSn.at(0)]["data"][14] = 0;
                    j["calibration"][idToSn.at(0)]["data"][15] = 1;

                    j["calibration"][idToSn.at(1)]["order"] = "ROW";
                    j["calibration"][idToSn.at(1)]["inverted"] = "FALSE";
                    j["calibration"][idToSn.at(1)]["data"][0] = calibrations.at(1)(0, 0);
                    j["calibration"][idToSn.at(1)]["data"][1] = calibrations.at(1)(0, 1);
                    j["calibration"][idToSn.at(1)]["data"][2] = calibrations.at(1)(0, 2);
                    j["calibration"][idToSn.at(1)]["data"][3] = calibrations.at(1)(0, 3);
                    j["calibration"][idToSn.at(1)]["data"][4] = calibrations.at(1)(1, 0);
                    j["calibration"][idToSn.at(1)]["data"][5] = calibrations.at(1)(1, 1);
                    j["calibration"][idToSn.at(1)]["data"][6] = calibrations.at(1)(1, 2);
                    j["calibration"][idToSn.at(1)]["data"][7] = calibrations.at(1)(1, 3);
                    j["calibration"][idToSn.at(1)]["data"][8] = calibrations.at(1)(2, 0);
                    j["calibration"][idToSn.at(1)]["data"][9] = calibrations.at(1)(2, 1);
                    j["calibration"][idToSn.at(1)]["data"][10] = calibrations.at(1)(2, 2);
                    j["calibration"][idToSn.at(1)]["data"][11] = calibrations.at(1)(2, 3);
                    j["calibration"][idToSn.at(1)]["data"][12] = 0;
                    j["calibration"][idToSn.at(1)]["data"][13] = 0;
                    j["calibration"][idToSn.at(1)]["data"][14] = 0;
                    j["calibration"][idToSn.at(1)]["data"][15] = 1;

                    if (imu_lidar[0].check)
                    {
                        j["imuToUse"] = idToSn.at(0);
                    }
                    else
                    {
                        j["imuToUse"] = idToSn.at(1);
                    }
                    std::ofstream fs(output_file_name);
                    if (!fs.good())
                        return;
                    fs << j.dump(2);
                    fs.close();
                }
            }
            ImGui::SameLine();
            ImGui::Text(" imprtant notice! 'lidar_odometry_step_1.exe' program requires file name 'calibration.json' in folder with data");
            ImGui::Text("=========================================================================");
        }
        ImGui::End();
    }
    return;
}

void display()
{
    ImGuiIO &io = ImGui::GetIO();
    glViewport(0, 0, (GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);

    if (is_ortho)
    {
        glOrtho(-camera_ortho_xy_view_zoom, camera_ortho_xy_view_zoom,
                -camera_ortho_xy_view_zoom / ratio,
                camera_ortho_xy_view_zoom / ratio, -100000, 100000);

        glm::mat4 proj = glm::orthoLH_ZO<float>(-camera_ortho_xy_view_zoom, camera_ortho_xy_view_zoom,
                                                -camera_ortho_xy_view_zoom / ratio,
                                                camera_ortho_xy_view_zoom / ratio, -100, 100);

        std::copy(&proj[0][0], &proj[3][3], m_ortho_projection);

        Eigen::Vector3d v_eye_t(-camera_ortho_xy_view_shift_x, camera_ortho_xy_view_shift_y, camera_mode_ortho_z_center_h + 10);
        Eigen::Vector3d v_center_t(-camera_ortho_xy_view_shift_x, camera_ortho_xy_view_shift_y, camera_mode_ortho_z_center_h);
        Eigen::Vector3d v(0, 1, 0);

        TaitBryanPose pose_tb;
        pose_tb.px = 0.0;
        pose_tb.py = 0.0;
        pose_tb.pz = 0.0;
        pose_tb.om = 0.0;
        pose_tb.fi = 0.0;
        pose_tb.ka = -camera_ortho_xy_view_rotation_angle_deg * M_PI / 180.0;
        auto m = affine_matrix_from_pose_tait_bryan(pose_tb);

        Eigen::Vector3d v_t = m * v;

        gluLookAt(v_eye_t.x(), v_eye_t.y(), v_eye_t.z(),
                  v_center_t.x(), v_center_t.y(), v_center_t.z(),
                  v_t.x(), v_t.y(), v_t.z());
        glm::mat4 lookat = glm::lookAt(glm::vec3(v_eye_t.x(), v_eye_t.y(), v_eye_t.z()),
                                       glm::vec3(v_center_t.x(), v_center_t.y(), v_center_t.z()),
                                       glm::vec3(v_t.x(), v_t.y(), v_t.z()));
        std::copy(&lookat[0][0], &lookat[3][3], m_ortho_gizmo_view);
    }

    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    if (!is_ortho)
    {
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
        /*      glTranslatef(translate_x, translate_y, translate_z);
              glRotatef(rotate_x, 1.0, 0.0, 0.0);
              glRotatef(rotate_y, 0.0, 0.0, 1.0);*/
    }
    else
    {
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
    }

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

    if (show_axes)
    {
        glLineWidth(2);
        glBegin(GL_LINES);
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(1, 0.0f, 0.0f);

        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 1, 0.0f);

        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 1);
        glEnd();
        glLineWidth(1);
    }

    if (calibration.size() > 0)
    {
        for (const auto &c : calibration)
        {
            Eigen::Affine3d m = c.second;
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
        }
    }

    // point_cloud
    //
    if (manual_calibration)
    {
        int index_calibrated_lidar = -1;
        for (int i = 0; i < calibrated_lidar.size(); i++)
        {
            if (calibrated_lidar[i].check)
            {
                index_calibrated_lidar = i;
            }
        }

        glBegin(GL_POINTS);
        for (const auto &p : point_cloud)
        {
            Eigen::Affine3d cal = calibrations.empty() ? Eigen::Affine3d::Identity() : calibrations.at(p.lidarid);

            if (p.lidarid == 0)
            {
                if (p.lidarid != index_calibrated_lidar)
                {
                    glColor3f(0.5, 0.5, 0.5);
                }
                else
                {
                    glColor3f(pc_color.x, pc_color.y, pc_color.z);
                }
            }
            else
            {
                if (p.lidarid != index_calibrated_lidar)
                {
                    glColor3f(0.5, 0.5, 0.5);
                }
                else
                {
                    glColor3f(pc_color2.x, pc_color2.y, pc_color2.z);
                }
            }

            auto pp = cal * p.point;

            glVertex3f(pp.x(), pp.y(), pp.z());
        }
        glEnd();
    }
    else
    {
        glBegin(GL_POINTS);
        for (const auto &p : point_cloud)
        {
            Eigen::Affine3d cal = calibrations.empty() ? Eigen::Affine3d::Identity() : calibrations.at(p.lidarid);

            if (p.lidarid == 0)
            {
                glColor3f(pc_color.x, pc_color.y, pc_color.z);
            }
            else
            {
                glColor3f(pc_color2.x, pc_color2.y, pc_color2.z);
            }

            auto pp = cal * p.point;

            glVertex3f(pp.x(), pp.y(), pp.z());
        }
        glEnd();
    }
    //

    if (show_grid)
    {
        glColor3f(0.4, 0.4, 0.4);
        glBegin(GL_LINES);
        for (float x = -10.0f; x <= 10.0f; x += 1.0f)
        {
            glVertex3f(x, -10.0, 0.0);
            glVertex3f(x, 10.0, 0.0);
        }
        for (float y = -10.0f; y <= 10.0f; y += 1.0f)
        {
            glVertex3f(-10.0, y, 0.0);
            glVertex3f(10.0, y, 0.0);
        }
        glEnd();
    }

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGLUT_NewFrame();

    project_gui();

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
    glutCreateWindow("mandeye_mission_recorder_calibration " HDMAPPING_VERSION_STRING);
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

void wheel(int button, int dir, int x, int y);

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

    static int glutMajorVersion = glutGet(GLUT_VERSION) / 10000;
    if (state == GLUT_DOWN && (glut_button == 3 || glut_button == 4) &&
        glutMajorVersion < 3)
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

void wheel(int button, int dir, int x, int y)
{
    if (dir > 0)
    {
        if (is_ortho)
        {
            camera_ortho_xy_view_zoom -= 0.1f * camera_ortho_xy_view_zoom;

            if (camera_ortho_xy_view_zoom < 0.1)
            {
                camera_ortho_xy_view_zoom = 0.1;
            }
        }
        else
        {
            translate_z -= 0.05f * translate_z;
        }
    }
    else
    {
        if (is_ortho)
        {
            camera_ortho_xy_view_zoom += 0.1 * camera_ortho_xy_view_zoom;
        }
        else
        {
            translate_z += 0.05f * translate_z;
        }
    }

    return;
}

int main(int argc, char *argv[])
{
    params.decimation = 0.03;

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
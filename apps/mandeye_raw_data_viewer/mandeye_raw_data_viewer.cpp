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
std::vector<int> indexes_to_filename;
std::vector<std::string> all_file_names;

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

void project_gui()
{
    if (ImGui::Begin("main gui window"))
    {
        ImGui::ColorEdit3("clear color", (float *)&clear_color);
        ImGui::ColorEdit3("pc_color", (float *)&pc_color);

        ImGui::InputInt("point_size", &point_size);
        if (point_size < 1)
        {
            point_size = 1;
        }

        if (is_init)
        {
            ImGui::InputInt("number_of_points_threshold", &number_of_points_threshold);
            if (number_of_points_threshold < 0)
            {
                number_of_points_threshold = 0;
            }
        }

        ImGui::InputInt("index_rendered_points_local", &index_rendered_points_local);
        if (index_rendered_points_local < 0)
        {
            index_rendered_points_local = 0;
        }
        if (index_rendered_points_local >= all_points_local.size())
        {
            index_rendered_points_local = all_points_local.size() - 1;
        }

        if (is_init)
        {
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
                    }
                };
                std::thread t1(t);
                t1.join();

                std::sort(std::begin(input_file_names), std::end(input_file_names));

                std::for_each(std::begin(input_file_names), std::end(input_file_names), [&](const std::string &fileName)
                              {
                    if (fileName.ends_with(".laz") || fileName.ends_with(".las"))
                    {
                        laz_files.push_back(fileName);
                        all_file_names.push_back(fileName);
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

                    working_directory_preview = wdp.string();

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
                    // std::vector<std::vector<int>> indexesPerFile;

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
                                       auto data = load_point_cloud(fn.c_str(), true, filter_threshold_xy, calibration);

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

                        trajectory[timestamp_pair.first] = std::pair(t.matrix(), timestamp_pair.second);
                        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
                        counter++;
                        if (counter % 100 == 0)
                        {
                            printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f [%d of %d]\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw, counter++, imu_data.size());
                        }
                    }

                    std::vector<std::pair<double, double>> timestamps;
                    std::vector<Eigen::Affine3d> poses;
                    for (const auto &t : trajectory)
                    {
                        timestamps.emplace_back(t.first, t.second.second);
                        Eigen::Affine3d m;
                        m.matrix() = t.second.first;
                        poses.push_back(m);
                    }

                    int number_of_points = 0;
                    for (const auto &pp : pointsPerFile)
                    {
                        number_of_points += pp.size();
                    }
                    std::cout << "number of points: " << number_of_points << std::endl;

                    std::vector<Eigen::Vector3d> points_local;

                    for (int i = 0; i < pointsPerFile.size(); i++)
                    {
                        for (const auto &pp : pointsPerFile[i])
                        {
                            auto lower = std::lower_bound(timestamps.begin(), timestamps.end(), pp.timestamp,
                                                          [](std::pair<double, double> lhs, double rhs) -> bool
                                                          { return lhs.first < rhs; });

                            int index_pose = std::distance(timestamps.begin(), lower);

                            if (index_pose >= 0 && index_pose < poses.size())
                            {
                                auto ppp = pp;
                                ppp.point = poses[index_pose] * ppp.point;

                                points_local.push_back(ppp.point);
                            }

                            
                            if (points_local.size() > number_of_points_threshold)
                            {
                                all_points_local.push_back(points_local);
                                indexes_to_filename.push_back(i);
                                points_local.clear();
                            }
                        }
                    }

                    if (all_points_local.size() > 0)
                    {
                        is_init = false;
                        index_rendered_points_local = 0;
                    }

#if 0
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

                        //if (params.decimation > 0.0)
                        //{
                        //    wd.intermediate_points = decimate(wd.intermediate_points, params.decimation, params.decimation, params.decimation);
                        //}

                        /*worker_data.push_back(wd);
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
                        wd.imu_roll_pitch.reserve(1000);*/

                        // temp_ts.clear();
                    }
                }

                //params.m_g = worker_data[0].intermediate_trajectory[0];
                //step_1_done = true;
                //std::cout << "step_1_done please click 'compute_all (step 2)' to continue calculations" << std::endl;
#endif
                }
                else
                {
                    std::cout << "please select files correctly" << std::endl;
                    std::cout << "input_file_names.size(): " << input_file_names.size() << std::endl;
                    std::cout << "laz_files.size(): " << laz_files.size() << std::endl;
                    std::cout << "csv_files.size(): " << csv_files.size() << std::endl;

                    std::cout << "condition: input_file_names.size() > 0 && laz_files.size() == csv_files.size() NOT SATISFIED!!!" << std::endl;
                }
            }
        }

        if (all_file_names.size() > 0)
        {
            if (index_rendered_points_local >= 0 && index_rendered_points_local < indexes_to_filename.size())
            {
                std::string fn = all_file_names[indexes_to_filename[index_rendered_points_local]];
                ImGui::Text(fn.c_str());
            }
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
    }

    glColor3f(pc_color.x, pc_color.y, pc_color.z);
    glPointSize(point_size);
    glBegin(GL_POINTS);
    if (index_rendered_points_local >= 0 && index_rendered_points_local < all_points_local.size())
    {
        for (const auto &p : all_points_local[index_rendered_points_local])
        {
            glVertex3f(p.x(), p.y(), p.z());
        }
    }
    glEnd();
    glPointSize(1);

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
    glutCreateWindow("mandeye raw data viewer " HDMAPPING_VERSION_STRING);
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
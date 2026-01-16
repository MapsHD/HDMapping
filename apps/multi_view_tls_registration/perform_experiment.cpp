#include <cmath>

// clang-format off
#include <GL/glew.h>
#include <GL/freeglut.h>
// clang-format on

#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <imgui_internal.h>

#include <ImGuizmo.h>

#include <Eigen/Eigen>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <icp.h>
#include <ndt.h>
#include <observation_picking.h>
#include <pose_graph_slam.h>
#include <registration_plane_feature.h>
#include <transformations.h>

#include <portable-file-dialogs.h>

#include <laszip/laszip_api.h>

#include <filesystem>
#include <fstream>
#include <iostream>

#include <manual_pose_graph_loop_closure.h>

#include <gnss.h>
#include <session.h>

namespace fs = std::filesystem;

void export_result_to_folder(std::string output_folder_name, ObservationPicking& observation_picking, Session& session)
{
    fs::path path(output_folder_name);
    std::string file_name_rms = "rms.csv";
    auto path_rms = path;
    path_rms /= file_name_rms;
    std::cout << "exporting to file: '" << path_rms.string() << "'" << std::endl;
    std::ofstream outfile_rms;
    outfile_rms.open(path_rms, std::ios_base::app);
    outfile_rms << "index_roi, rms_initial, rms_result" << std::endl;

    for (int i = 0; i < observation_picking.intersections.size(); i++)
    {
        std::string file_name_initial = "intersection_" + std::to_string(i) + "_initial.csv";
        std::string file_name_result = "intersection_" + std::to_string(i) + "_result.csv";

        auto path_initial = path;
        auto path_result = path;

        path_initial /= file_name_initial;
        path_result /= file_name_result;

        std::cout << "exporting to file: '" << path_initial.string() << "'" << std::endl;
        std::cout << "exporting to file: '" << path_result.string() << "'" << std::endl;

        std::ofstream outfile_initial;
        std::ofstream outfile_result;

        outfile_initial.open(path_initial, std::ios_base::app);
        outfile_result.open(path_result, std::ios_base::app);

        const auto& intersection = observation_picking.intersections[i];
        TaitBryanPose pose;
        pose.px = intersection.translation[0];
        pose.py = intersection.translation[1];
        pose.pz = intersection.translation[2];
        pose.om = intersection.rotation[0];
        pose.fi = intersection.rotation[1];
        pose.ka = intersection.rotation[2];
        Eigen::Affine3d m_pose_inv = affine_matrix_from_pose_tait_bryan(pose).inverse();

        double w = intersection.width_length_height[0] * 0.5;
        double l = intersection.width_length_height[1] * 0.5;
        double h = intersection.width_length_height[2] * 0.5;

        outfile_initial << "x;y;z;pc_index;is_initial;index_intersection;file" << std::endl;
        outfile_result << "x;y;z;pc_index;is_initial;index_intersection;file" << std::endl;

        for (int pc_index = 0; pc_index < session.point_clouds_container.point_clouds.size(); pc_index++)
        {
            const auto& pc = session.point_clouds_container.point_clouds[pc_index];
            for (const auto& p : pc.points_local)
            {
                Eigen::Vector3d vpi = pc.m_initial_pose * p;
                Eigen::Vector3d vpr = pc.m_pose * p;

                Eigen::Vector3d vpit = m_pose_inv * vpi;
                Eigen::Vector3d vprt = m_pose_inv * vpr;

                if (fabs(vpit.x()) < w)
                {
                    if (fabs(vpit.y()) < l)
                    {
                        if (fabs(vpit.z()) < h)
                        {
                            outfile_initial << vpit.x() << ";" << vpit.y() << ";" << vpit.z() << ";" << pc_index << ";1;" << i << ";"
                                            << pc.file_name << std::endl;
                        }
                    }
                }
                if (fabs(vprt.x()) < w)
                {
                    if (fabs(vprt.y()) < l)
                    {
                        if (fabs(vprt.z()) < h)
                        {
                            outfile_result << vprt.x() << ";" << vprt.y() << ";" << vprt.z() << ";" << pc_index << ";0;" << i << ";"
                                           << pc.file_name << std::endl;
                        }
                    }
                }
            }
        }
        outfile_initial.close();
        outfile_result.close();

        const auto& obs = observation_picking.observations[i];
        double rms_initial = 0.0;
        int sum = 0;
        double rms_result = 0.0;

        for (const auto& [key1, value1] : obs)
        {
            for (const auto& [key2, value2] : obs)
            {
                if (key1 != key2)
                {
                    Eigen::Vector3d p1, p2;
                    p1 = session.point_clouds_container.point_clouds[key1].m_initial_pose * value1;
                    p2 = session.point_clouds_container.point_clouds[key2].m_initial_pose * value2;
                    rms_initial += (p2.x() - p1.x()) * (p2.x() - p1.x());
                    rms_initial += (p2.y() - p1.y()) * (p2.y() - p1.y());

                    p1 = session.point_clouds_container.point_clouds[key1].m_pose * value1;
                    p2 = session.point_clouds_container.point_clouds[key2].m_pose * value2;
                    rms_result += (p2.x() - p1.x()) * (p2.x() - p1.x());
                    rms_result += (p2.y() - p1.y()) * (p2.y() - p1.y());

                    sum += 2;
                }
            }
        }
        std::cout << "sum: " << sum << std::endl;
        if (sum > 0)
        {
            rms_initial = sqrt(rms_initial / sum);
            rms_result = sqrt(rms_result / sum);
            outfile_rms << i << ";" << rms_initial << ";" << rms_result << std::endl;
        }
    }
    outfile_rms.close();

    std::string file_name_poses = "poses_RESSO.reg";
    auto path_poses = path;
    path_poses /= file_name_poses;
    std::cout << "saving poses to: " << path_poses << std::endl;
    session.point_clouds_container.save_poses(path_poses.string(), false);
}

void export_result_to_folder(std::string output_folder_name, int method_id, ObservationPicking& observation_picking, Session& session)
{
    fs::path path(output_folder_name);
    path /= std::to_string(method_id);
    create_directory(path);
    export_result_to_folder(path.string(), observation_picking, session);
}

template<typename T>
void append_to_result_file(
    std::string file_name, std::string method, const T& t, float rms, int id_method, std::chrono::milliseconds elapsed)
{
    std::ofstream outfile;
    outfile.open(file_name, std::ios_base::app);
    outfile << method << ";" << id_method << ";" << int(t.is_gauss_newton) << ";" << int(t.is_levenberg_marguardt) << ";" << int(t.is_wc)
            << ";" << int(t.is_cw) << ";" << int(t.is_tait_bryan_angles) << ";" << int(t.is_quaternion) << ";" << int(t.is_rodrigues) << ";"
            << int(t.is_lie_algebra_left_jacobian) << ";" << int(t.is_lie_algebra_right_jacobian) << ";" << rms << ";" << elapsed.count()
            << std::endl;
    outfile.close();
}

void create_header(std::string file_name)
{
    std::ofstream outfile;
    outfile.open(file_name);
    outfile << "method;id_method;gauss_newton;levenberg_marguardt;wc;cw;tait_bryan_angles;quaternion;rodrigues;Lie_algebra_left_jacobian;"
               "Lie_algebra_right_jacobian;rms;elapsed_time_miliseconds"
            << std::endl;
    outfile.close();
}

void add_initial_rms_to_file(std::string file_name, float rms)
{
    std::ofstream outfile;
    outfile.open(file_name, std::ios_base::app);
    // outfile <<
    // "method;is_gauss_newton;is_levenberg_marguardt;is_wc;is_cw;is_tait_bryan_angles;is_quaternion;is_rodrigues;is_Lie_algebra_left;Lie_algebra_right;rms;elapsed_time_miliseconds"
    // << std::endl;
    outfile << "initial_rms;0;0;0;0;0;0;0;0;0;0;" << rms << ";0" << std::endl;
    outfile.close();
}

void reset_poses(Session& session)
{
    for (size_t i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
    {
        session.point_clouds_container.point_clouds[i].m_pose = session.point_clouds_container.point_clouds[i].m_initial_pose;
        session.point_clouds_container.point_clouds[i].pose =
            pose_tait_bryan_from_affine_matrix(session.point_clouds_container.point_clouds[i].m_pose);
        session.point_clouds_container.point_clouds[i].gui_translation[0] = (float)session.point_clouds_container.point_clouds[i].pose.px;
        session.point_clouds_container.point_clouds[i].gui_translation[1] = (float)session.point_clouds_container.point_clouds[i].pose.py;
        session.point_clouds_container.point_clouds[i].gui_translation[2] = (float)session.point_clouds_container.point_clouds[i].pose.pz;
        session.point_clouds_container.point_clouds[i].gui_rotation[0] =
            (float)rad2deg(session.point_clouds_container.point_clouds[i].pose.om);
        session.point_clouds_container.point_clouds[i].gui_rotation[1] =
            (float)rad2deg(session.point_clouds_container.point_clouds[i].pose.fi);
        session.point_clouds_container.point_clouds[i].gui_rotation[2] =
            (float)rad2deg(session.point_clouds_container.point_clouds[i].pose.ka);
    }
}

double compute_rms(bool initial, Session& session, ObservationPicking& observation_picking)
{
    double rms = 0.0;
    int sum = 0;
    for (const auto& obs : observation_picking.observations)
    {
        for (const auto& [key1, value1] : obs)
        {
            for (const auto& [key2, value2] : obs)
            {
                if (key1 != key2)
                {
                    Eigen::Vector3d p1, p2;
                    if (initial)
                    {
                        p1 = session.point_clouds_container.point_clouds[key1].m_initial_pose * value1;
                        p2 = session.point_clouds_container.point_clouds[key2].m_initial_pose * value2;
                    }
                    else
                    {
                        p1 = session.point_clouds_container.point_clouds[key1].m_pose * value1;
                        p2 = session.point_clouds_container.point_clouds[key2].m_pose * value2;
                    }
                    rms += (p2.x() - p1.x()) * (p2.x() - p1.x());
                    sum++;
                    rms += (p2.y() - p1.y()) * (p2.y() - p1.y());
                    sum++;
                }
            }
        }
    }
    if (sum == 0)
    {
        std::cout << "sum == 0" << std::endl;
        return -1;
    }
    else
    {
        rms = sqrt(rms / sum);
        return rms;
    }
}

void perform_experiment_on_windows(
    Session& session,
    ObservationPicking& observation_picking,
    ICP& icp,
    NDT& ndt,
    RegistrationPlaneFeature& registration_plane_feature,
    PoseGraphSLAM& pose_graph_slam)
{
    bool compute_mean_and_cov_for_bucket = false;
    session.point_clouds_container.show_with_initial_pose = false;
    auto temp_data = session.point_clouds_container;
    reset_poses(session);
    double rms = 0.0f;
    std::string result_file = session.working_directory + "/result_win.csv";
    float search_radius = 0.1f;
    int number_of_threads = 16;
    int number_of_iterations = 6;
    int id_method = 0;

    create_header(result_file);
    double initial_rms = compute_rms(false, session, observation_picking);
    std::cout << "initial rms: " << initial_rms << std::endl;
    add_initial_rms_to_file(result_file, initial_rms);

    // void export_result_to_folder(std::string output_folder_name, int method_id) {
    //     fs::path path(output_folder_name);
    //     path /= std::to_string(method_id);
    //     create_directory(path);
    //     export_result_to_folder(path.string());
    // }

    fs::path path_result(session.working_directory);
    path_result /= "results_win";
    create_directory(path_result);

    //--0--
    icp.is_adaptive_robust_kernel = false;
    icp.is_fix_first_node = false;
    icp.search_radius = search_radius;
    icp.number_of_threads = number_of_threads;
    icp.number_of_iterations = number_of_iterations;
    icp.is_adaptive_robust_kernel = false;

    icp.is_gauss_newton = true;
    icp.is_levenberg_marguardt = false;

    icp.is_wc = true;
    icp.is_cw = false;

    icp.is_tait_bryan_angles = true;
    icp.is_quaternion = false;
    icp.is_rodrigues = false;
    icp.is_lie_algebra_left_jacobian = false;
    icp.is_lie_algebra_right_jacobian = false;

    auto start = std::chrono::system_clock::now();
    icp.optimization_point_to_point_source_to_target(session.point_clouds_container);
    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    std::cout << "final RMS: " << rms << std::endl;

    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;
    id_method++;

    //--1--
    icp.is_tait_bryan_angles = false;
    icp.is_quaternion = true;
    icp.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    icp.optimization_point_to_point_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 1;

    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;
    // id_method++;
    //--2--
    icp.is_tait_bryan_angles = false;
    icp.is_quaternion = false;
    icp.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    icp.optimization_point_to_point_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 2;
    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--3--
    icp.is_wc = false;
    icp.is_cw = true;

    icp.is_tait_bryan_angles = true;
    icp.is_quaternion = false;
    icp.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    icp.optimization_point_to_point_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 3;
    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--4--
    icp.is_tait_bryan_angles = false;
    icp.is_quaternion = true;
    icp.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    icp.optimization_point_to_point_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 4;
    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--5--
    icp.is_tait_bryan_angles = false;
    icp.is_quaternion = false;
    icp.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    icp.optimization_point_to_point_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 5;
    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--6--
    icp.is_gauss_newton = false;
    icp.is_levenberg_marguardt = true;

    icp.is_wc = true;
    icp.is_cw = false;

    icp.is_tait_bryan_angles = true;
    icp.is_quaternion = false;
    icp.is_rodrigues = false;
    icp.is_lie_algebra_left_jacobian = false;
    icp.is_lie_algebra_right_jacobian = false;

    start = std::chrono::system_clock::now();
    icp.optimization_point_to_point_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 6;
    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--7--
    icp.is_tait_bryan_angles = false;
    icp.is_quaternion = true;
    icp.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    icp.optimization_point_to_point_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 7;
    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--8--
    icp.is_tait_bryan_angles = false;
    icp.is_quaternion = false;
    icp.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    icp.optimization_point_to_point_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 8;
    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--9--
    icp.is_wc = false;
    icp.is_cw = true;

    icp.is_tait_bryan_angles = true;
    icp.is_quaternion = false;
    icp.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    icp.optimization_point_to_point_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 9;
    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--10--
    icp.is_wc = false;
    icp.is_cw = true;

    icp.is_tait_bryan_angles = false;
    icp.is_quaternion = true;
    icp.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    icp.optimization_point_to_point_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 10;
    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--11--
    icp.is_wc = false;
    icp.is_cw = true;

    icp.is_tait_bryan_angles = false;
    icp.is_quaternion = false;
    icp.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    icp.optimization_point_to_point_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 11;
    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--12--
    icp.is_gauss_newton = true;
    icp.is_levenberg_marguardt = false;

    icp.is_wc = true;
    icp.is_cw = false;

    icp.is_tait_bryan_angles = false;
    icp.is_quaternion = false;
    icp.is_rodrigues = true;

    icp.is_lie_algebra_left_jacobian = true;
    icp.is_lie_algebra_right_jacobian = false;

    start = std::chrono::system_clock::now();
    icp.optimize_source_to_target_lie_algebra_left_jacobian(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 12;
    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--13--
    icp.is_lie_algebra_left_jacobian = false;
    icp.is_lie_algebra_right_jacobian = true;
    start = std::chrono::system_clock::now();
    icp.optimize_source_to_target_lie_algebra_right_jacobian(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 13;
    append_to_result_file(result_file, "point_to_point", icp, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //---NDT---
    //--14--
    //
    ndt.is_fix_first_node = false;
    ndt.bucket_size[0] = 0.5;
    ndt.bucket_size[1] = 0.5;
    ndt.bucket_size[2] = 0.5;
    ndt.number_of_threads = number_of_threads;
    ndt.number_of_iterations = number_of_iterations;

    ndt.is_gauss_newton = true;
    ndt.is_levenberg_marguardt = false;

    ndt.is_wc = true;
    ndt.is_cw = false;

    ndt.is_tait_bryan_angles = true;
    ndt.is_quaternion = false;
    ndt.is_rodrigues = false;
    ndt.is_lie_algebra_left_jacobian = false;
    ndt.is_lie_algebra_right_jacobian = false;

    start = std::chrono::system_clock::now();
    ndt.optimize(session.point_clouds_container.point_clouds, true, compute_mean_and_cov_for_bucket);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    std::cout << "final rms: " << rms << std::endl;

    id_method = 14;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--15--
    ndt.is_tait_bryan_angles = false;
    ndt.is_quaternion = true;
    ndt.is_rodrigues = false;
    start = std::chrono::system_clock::now();
    ndt.optimize(session.point_clouds_container.point_clouds, true, compute_mean_and_cov_for_bucket);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);

    id_method = 15;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    session.point_clouds_container = temp_data;

    //--16--
    ndt.is_tait_bryan_angles = false;
    ndt.is_quaternion = false;
    ndt.is_rodrigues = true;
    start = std::chrono::system_clock::now();
    ndt.optimize(session.point_clouds_container.point_clouds, true, compute_mean_and_cov_for_bucket);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);

    id_method = 16;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--17--
    ndt.is_wc = false;
    ndt.is_cw = true;

    ndt.is_tait_bryan_angles = true;
    ndt.is_quaternion = false;
    ndt.is_rodrigues = false;
    start = std::chrono::system_clock::now();
    ndt.optimize(session.point_clouds_container.point_clouds, true, compute_mean_and_cov_for_bucket);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 17;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--18--
    ndt.is_tait_bryan_angles = false;
    ndt.is_quaternion = true;
    ndt.is_rodrigues = false;
    start = std::chrono::system_clock::now();
    ndt.optimize(session.point_clouds_container.point_clouds, true, compute_mean_and_cov_for_bucket);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);

    id_method = 18;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--19--
    ndt.is_tait_bryan_angles = false;
    ndt.is_quaternion = false;
    ndt.is_rodrigues = true;
    start = std::chrono::system_clock::now();
    ndt.optimize(session.point_clouds_container.point_clouds, true, compute_mean_and_cov_for_bucket);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);

    id_method = 19;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--20--
    ndt.is_gauss_newton = false;
    ndt.is_levenberg_marguardt = true;

    ndt.is_wc = true;
    ndt.is_cw = false;

    ndt.is_tait_bryan_angles = true;
    ndt.is_quaternion = false;
    ndt.is_rodrigues = false;
    start = std::chrono::system_clock::now();
    ndt.optimize(session.point_clouds_container.point_clouds, true, compute_mean_and_cov_for_bucket);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);

    id_method = 20;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--21--
    ndt.is_tait_bryan_angles = false;
    ndt.is_quaternion = true;
    ndt.is_rodrigues = false;
    start = std::chrono::system_clock::now();
    ndt.optimize(session.point_clouds_container.point_clouds, true, compute_mean_and_cov_for_bucket);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);

    id_method = 21;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--22--
    ndt.is_tait_bryan_angles = false;
    ndt.is_quaternion = false;
    ndt.is_rodrigues = true;
    start = std::chrono::system_clock::now();
    ndt.optimize(session.point_clouds_container.point_clouds, true, compute_mean_and_cov_for_bucket);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);

    id_method = 22;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--23--
    ndt.is_wc = false;
    ndt.is_cw = true;

    ndt.is_tait_bryan_angles = true;
    ndt.is_quaternion = false;
    ndt.is_rodrigues = false;
    start = std::chrono::system_clock::now();
    ndt.optimize(session.point_clouds_container.point_clouds, true, compute_mean_and_cov_for_bucket);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);

    id_method = 23;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--24--
    ndt.is_tait_bryan_angles = false;
    ndt.is_quaternion = true;
    ndt.is_rodrigues = false;
    start = std::chrono::system_clock::now();
    ndt.optimize(session.point_clouds_container.point_clouds, true, compute_mean_and_cov_for_bucket);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);

    id_method = 24;

    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--25--
    ndt.is_tait_bryan_angles = false;
    ndt.is_quaternion = false;
    ndt.is_rodrigues = true;
    start = std::chrono::system_clock::now();
    ndt.optimize(session.point_clouds_container.point_clouds, true, compute_mean_and_cov_for_bucket);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);

    id_method = 25;

    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--26--
    ndt.is_gauss_newton = true;
    ndt.is_levenberg_marguardt = false;

    ndt.is_wc = true;
    ndt.is_cw = false;

    ndt.is_tait_bryan_angles = false;
    ndt.is_quaternion = false;
    ndt.is_rodrigues = false;

    ndt.is_lie_algebra_left_jacobian = true;
    ndt.is_lie_algebra_right_jacobian = false;
    start = std::chrono::system_clock::now();
    ndt.optimize(session.point_clouds_container.point_clouds, true, compute_mean_and_cov_for_bucket);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);

    id_method = 26;

    ndt.is_rodrigues = true;

    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--27--
    ndt.is_gauss_newton = false;
    ndt.is_levenberg_marguardt = true;
    ndt.is_rodrigues = false;
    start = std::chrono::system_clock::now();
    ndt.optimize(session.point_clouds_container.point_clouds, true, compute_mean_and_cov_for_bucket);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);

    id_method = 27;
    ndt.is_rodrigues = true;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--28--
    ndt.is_lie_algebra_left_jacobian = false;
    ndt.is_lie_algebra_right_jacobian = true;
    ndt.is_rodrigues = false;

    ndt.is_gauss_newton = true;
    ndt.is_levenberg_marguardt = false;
    start = std::chrono::system_clock::now();
    ndt.optimize(session.point_clouds_container.point_clouds, true, compute_mean_and_cov_for_bucket);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);

    id_method = 28;
    ndt.is_rodrigues = true;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--29--
    ndt.is_gauss_newton = false;
    ndt.is_levenberg_marguardt = true;
    ndt.is_rodrigues = false;
    start = std::chrono::system_clock::now();
    ndt.optimize(session.point_clouds_container.point_clouds, true, compute_mean_and_cov_for_bucket);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);

    id_method = 29;
    ndt.is_rodrigues = true;
    append_to_result_file(result_file, "normal_distributions_transform", ndt, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //----------------------------------------------------------------------------

    registration_plane_feature.search_radius = search_radius;
    registration_plane_feature.number_of_threads = number_of_threads;
    registration_plane_feature.number_of_iterations = number_of_iterations;
    registration_plane_feature.is_adaptive_robust_kernel = false;
    registration_plane_feature.is_fix_first_node = false;

    //--30--
    registration_plane_feature.is_gauss_newton = true;
    registration_plane_feature.is_levenberg_marguardt = false;

    registration_plane_feature.is_wc = true;
    registration_plane_feature.is_cw = false;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 30;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--31--
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 31;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--32--
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 32;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--33--
    registration_plane_feature.is_wc = false;
    registration_plane_feature.is_cw = true;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 33;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--34--
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 34;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--35--
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 35;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //------------------------------------------------------
    //--36--
    registration_plane_feature.is_gauss_newton = false;
    registration_plane_feature.is_levenberg_marguardt = true;

    registration_plane_feature.is_wc = true;
    registration_plane_feature.is_cw = false;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 36;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--37--
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 37;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--38--
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 38;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--39--
    registration_plane_feature.is_wc = false;
    registration_plane_feature.is_cw = true;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 39;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--40--
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 40;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--41--
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 41;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--42-- Lie
    registration_plane_feature.is_gauss_newton = true;
    registration_plane_feature.is_levenberg_marguardt = false;

    registration_plane_feature.is_wc = true;
    registration_plane_feature.is_cw = false;

    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    registration_plane_feature.is_lie_algebra_left_jacobian = true;
    registration_plane_feature.is_lie_algebra_right_jacobian = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_left_jacobian(
        session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 42;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--43--
    registration_plane_feature.is_gauss_newton = false;
    registration_plane_feature.is_levenberg_marguardt = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_left_jacobian(
        session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 43;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--44--
    registration_plane_feature.is_gauss_newton = true;
    registration_plane_feature.is_levenberg_marguardt = false;
    registration_plane_feature.is_lie_algebra_left_jacobian = false;
    registration_plane_feature.is_lie_algebra_right_jacobian = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_right_jacobian(
        session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 44;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--45--
    registration_plane_feature.is_gauss_newton = false;
    registration_plane_feature.is_levenberg_marguardt = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_right_jacobian(
        session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 45;
    append_to_result_file(result_file, "point_to_projection_onto_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--46--using dot product
    registration_plane_feature.is_lie_algebra_left_jacobian = false;
    registration_plane_feature.is_lie_algebra_right_jacobian = false;

    registration_plane_feature.is_gauss_newton = true;
    registration_plane_feature.is_levenberg_marguardt = false;

    registration_plane_feature.is_wc = true;
    registration_plane_feature.is_cw = false;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 46;
    append_to_result_file(result_file, "point_to_plane_using_dot_product", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--47
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 47;
    append_to_result_file(result_file, "point_to_plane_using_dot_product", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--48
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 48;
    append_to_result_file(result_file, "point_to_plane_using_dot_product", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--49
    registration_plane_feature.is_wc = false;
    registration_plane_feature.is_cw = true;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 49;
    append_to_result_file(result_file, "point_to_plane_using_dot_product", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--50
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 50;
    append_to_result_file(result_file, "point_to_plane_using_dot_product", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--51
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 51;
    append_to_result_file(result_file, "point_to_plane_using_dot_product", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--52
    registration_plane_feature.is_gauss_newton = false;
    registration_plane_feature.is_levenberg_marguardt = true;

    registration_plane_feature.is_wc = true;
    registration_plane_feature.is_cw = false;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 52;
    append_to_result_file(result_file, "point_to_plane_using_dot_product", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--53
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 53;
    append_to_result_file(result_file, "point_to_plane_using_dot_product", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--54
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 54;
    append_to_result_file(result_file, "point_to_plane_using_dot_product", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--55
    registration_plane_feature.is_wc = false;
    registration_plane_feature.is_cw = true;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 55;
    append_to_result_file(result_file, "point_to_plane_using_dot_product", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--56
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 56;
    append_to_result_file(result_file, "point_to_plane_using_dot_product", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--57
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_point_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 57;
    append_to_result_file(result_file, "point_to_plane_using_dot_product", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--58 optimize_distance_point_to_plane_source_to_target
    registration_plane_feature.is_gauss_newton = true;
    registration_plane_feature.is_levenberg_marguardt = false;

    registration_plane_feature.is_wc = true;
    registration_plane_feature.is_cw = false;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_distance_point_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 58;
    append_to_result_file(result_file, "distance_point_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--59
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_distance_point_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 59;
    append_to_result_file(result_file, "distance_point_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--60
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_distance_point_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 60;
    append_to_result_file(result_file, "distance_point_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--61
    registration_plane_feature.is_wc = false;
    registration_plane_feature.is_cw = true;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_distance_point_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 61;
    append_to_result_file(result_file, "distance_point_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--62
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_distance_point_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 62;
    append_to_result_file(result_file, "distance_point_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--63
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_distance_point_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 63;
    append_to_result_file(result_file, "distance_point_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--64
    registration_plane_feature.is_gauss_newton = false;
    registration_plane_feature.is_levenberg_marguardt = true;

    registration_plane_feature.is_wc = true;
    registration_plane_feature.is_cw = false;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_distance_point_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 64;
    append_to_result_file(result_file, "distance_point_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--65
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_distance_point_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 65;
    append_to_result_file(result_file, "distance_point_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--66
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_distance_point_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 66;
    append_to_result_file(result_file, "distance_point_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--67
    registration_plane_feature.is_wc = false;
    registration_plane_feature.is_cw = true;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_distance_point_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 67;
    append_to_result_file(result_file, "distance_point_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--68
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_distance_point_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 68;
    append_to_result_file(result_file, "distance_point_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--69
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_distance_point_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 69;
    append_to_result_file(result_file, "distance_point_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--70 optimize_plane_to_plane_source_to_target
    registration_plane_feature.is_adaptive_robust_kernel = true;

    registration_plane_feature.is_gauss_newton = true;
    registration_plane_feature.is_levenberg_marguardt = false;

    registration_plane_feature.is_wc = true;
    registration_plane_feature.is_cw = false;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_plane_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 70;
    append_to_result_file(result_file, "plane_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--71
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_plane_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 71;
    append_to_result_file(result_file, "plane_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--72
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_plane_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 72;
    append_to_result_file(result_file, "plane_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--73
    registration_plane_feature.is_wc = false;
    registration_plane_feature.is_cw = true;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_plane_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 73;
    append_to_result_file(result_file, "plane_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--74
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_plane_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 74;
    append_to_result_file(result_file, "plane_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--75
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_plane_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 75;
    append_to_result_file(result_file, "plane_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--76
    registration_plane_feature.is_gauss_newton = false;
    registration_plane_feature.is_levenberg_marguardt = true;

    registration_plane_feature.is_wc = true;
    registration_plane_feature.is_cw = false;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_plane_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 76;
    append_to_result_file(result_file, "plane_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--77
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_plane_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 77;
    append_to_result_file(result_file, "plane_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--78
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_plane_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 78;
    append_to_result_file(result_file, "plane_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--79
    registration_plane_feature.is_wc = false;
    registration_plane_feature.is_cw = true;

    registration_plane_feature.is_tait_bryan_angles = true;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_plane_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 79;
    append_to_result_file(result_file, "plane_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--80
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = true;
    registration_plane_feature.is_rodrigues = false;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_plane_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 80;
    append_to_result_file(result_file, "plane_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--81
    registration_plane_feature.is_tait_bryan_angles = false;
    registration_plane_feature.is_quaternion = false;
    registration_plane_feature.is_rodrigues = true;

    start = std::chrono::system_clock::now();
    registration_plane_feature.optimize_plane_to_plane_source_to_target(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 81;
    append_to_result_file(result_file, "plane_to_plane", registration_plane_feature, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    // pose graph slam
    //--82--
    pose_graph_slam.overlap_threshold = 0.3;
    pose_graph_slam.iterations = 6;

    pose_graph_slam.search_radius = search_radius;
    pose_graph_slam.number_of_threads = number_of_threads;
    pose_graph_slam.number_of_iterations_pair_wise_matching = number_of_iterations;

    //--
    pose_graph_slam.is_adaptive_robust_kernel = false;
    pose_graph_slam.is_fix_first_node = true;
    pose_graph_slam.is_gauss_newton = true;
    pose_graph_slam.is_levenberg_marguardt = false;
    pose_graph_slam.is_cw = false;
    pose_graph_slam.is_wc = true;
    pose_graph_slam.is_tait_bryan_angles = true;
    pose_graph_slam.is_quaternion = false;
    pose_graph_slam.is_rodrigues = false;

    session.point_clouds_container = temp_data;
    pose_graph_slam.set_all_to_false();
    pose_graph_slam.is_ndt = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    pose_graph_slam.ndt_bucket_size[0] = ndt.bucket_size[0];
    pose_graph_slam.ndt_bucket_size[1] = ndt.bucket_size[1];
    pose_graph_slam.ndt_bucket_size[2] = ndt.bucket_size[2];
    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 82;
    append_to_result_file(result_file, "pose_graph_slam (normal_distributions_transform)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);

    //--83
    session.point_clouds_container = temp_data;
    pose_graph_slam.set_all_to_false();
    pose_graph_slam.is_optimization_point_to_point_source_to_target = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 83;
    append_to_result_file(result_file, "pose_graph_slam (point_to_point)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--84
    session.point_clouds_container = temp_data;
    pose_graph_slam.set_all_to_false();
    pose_graph_slam.is_optimize_point_to_projection_onto_plane_source_to_target = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 84;
    append_to_result_file(result_file, "pose_graph_slam (point_to_projection_onto_plane)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);

    //--85
    session.point_clouds_container = temp_data;
    pose_graph_slam.set_all_to_false();
    pose_graph_slam.is_optimize_point_to_plane_source_to_target = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 85;
    append_to_result_file(result_file, "pose_graph_slam (point_to_plane_using_dot_product)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);

    //--86
    session.point_clouds_container = temp_data;
    pose_graph_slam.set_all_to_false();
    pose_graph_slam.is_optimize_distance_point_to_plane_source_to_target = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 86;
    append_to_result_file(result_file, "pose_graph_slam (distance_point_to_plane)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--87
    pose_graph_slam.is_adaptive_robust_kernel = true;

    pose_graph_slam.set_all_to_false();
    pose_graph_slam.is_optimize_plane_to_plane_source_to_target = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 87;
    append_to_result_file(result_file, "pose_graph_slam (plane_to_plane)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);

    //--88
    pose_graph_slam.set_all_to_false();
    session.point_clouds_container = temp_data;
    pose_graph_slam.is_adaptive_robust_kernel = false;
    pose_graph_slam.is_ndt_lie_algebra_left_jacobian = true;
    pose_graph_slam.is_lie_algebra_left_jacobian = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 88;
    append_to_result_file(result_file, "pose_graph_slam (ndt_lie_algebra_left_jacobian)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);

    //--89
    pose_graph_slam.set_all_to_false();
    session.point_clouds_container = temp_data;
    pose_graph_slam.is_ndt_lie_algebra_right_jacobian = true;
    pose_graph_slam.is_lie_algebra_right_jacobian = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 89;
    append_to_result_file(result_file, "pose_graph_slam (ndt_lie_algebra_right_jacobian)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);

    //--90
    pose_graph_slam.set_all_to_false();
    session.point_clouds_container = temp_data;
    pose_graph_slam.is_optimize_point_to_point_source_to_target_lie_algebra_left_jacobian = true;
    pose_graph_slam.is_lie_algebra_left_jacobian = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 90;
    append_to_result_file(
        result_file, "pose_graph_slam (point_to_point_lie_algebra_left_jacobian)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);

    //--91
    pose_graph_slam.set_all_to_false();
    session.point_clouds_container = temp_data;
    pose_graph_slam.is_optimize_point_to_point_source_to_target_lie_algebra_right_jacobian = true;
    pose_graph_slam.is_lie_algebra_right_jacobian = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 91;
    append_to_result_file(
        result_file, "pose_graph_slam (point_to_point_lie_algebra_right_jacobian)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);

    //--92
    pose_graph_slam.set_all_to_false();
    session.point_clouds_container = temp_data;
    pose_graph_slam.is_optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_left_jacobian = true;
    pose_graph_slam.is_lie_algebra_left_jacobian = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 92;
    append_to_result_file(
        result_file,
        "pose_graph_slam (point_to_projection_onto_plane_lie_algebra_left_jacobian)",
        pose_graph_slam,
        rms,
        id_method,
        elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);

    //--93
    pose_graph_slam.set_all_to_false();
    session.point_clouds_container = temp_data;
    pose_graph_slam.is_optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_right_jacobian = true;
    pose_graph_slam.is_lie_algebra_right_jacobian = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    rms = compute_rms(false, session, observation_picking);
    id_method = 93;
    append_to_result_file(
        result_file,
        "pose_graph_slam (point_to_projection_onto_plane_lie_algebra_right_jacobian)",
        pose_graph_slam,
        rms,
        id_method,
        elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
}

void perform_experiment_on_linux(
    Session& session,
    ObservationPicking& observation_picking,
    ICP& icp,
    NDT& ndt,
    RegistrationPlaneFeature& registration_plane_feature,
    PoseGraphSLAM& pose_graph_slam)
{
    fs::path path_result(session.working_directory);
    path_result /= "results_linux";
    create_directory(path_result);

    session.point_clouds_container.show_with_initial_pose = false;
    auto temp_data = session.point_clouds_container;
    // reset_poses();
    float rms = 0.0f;
    std::string result_file = session.working_directory + "/result_linux.csv";
    create_header(result_file);
    double initial_rms = compute_rms(false, session, observation_picking);
    std::cout << "initial rms: " << initial_rms << std::endl;
    add_initial_rms_to_file(result_file, initial_rms);

    float search_radius = 0.1f;
    int number_of_threads = 16;
    int number_of_iterations = 6;
    int id_method = 0;

    // pose graph slam
    //--94--
    pose_graph_slam.overlap_threshold = 0.3;
    pose_graph_slam.iterations = 6;

    pose_graph_slam.search_radius = search_radius;
    pose_graph_slam.number_of_threads = number_of_threads;
    pose_graph_slam.number_of_iterations_pair_wise_matching = number_of_iterations;

    //--
    pose_graph_slam.is_adaptive_robust_kernel = false;
    pose_graph_slam.is_fix_first_node = true;
    pose_graph_slam.is_gauss_newton = true;
    pose_graph_slam.is_levenberg_marguardt = false;
    pose_graph_slam.is_cw = false;
    pose_graph_slam.is_wc = true;
    pose_graph_slam.is_tait_bryan_angles = false;
    pose_graph_slam.is_quaternion = false;
    pose_graph_slam.is_rodrigues = true;

    session.point_clouds_container = temp_data;

    // pose_graph_slam.is_ndt = true;
    // pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::general;
    // pose_graph_slam.ndt_bucket_size[0] = ndt.bucket_size[0];
    // pose_graph_slam.ndt_bucket_size[1] = ndt.bucket_size[1];
    // pose_graph_slam.ndt_bucket_size[2] = ndt.bucket_size[2];
    // pose_graph_slam.optimize(point_clouds_container);
    // rms = compute_rms();
    // id_method = 94;
    // append_to_result_file(result_file, "pose_graph_slam (normal_distributions_transform)", pose_graph_slam, rms, id_method);

#ifdef WITH_PCL
    pose_graph_slam.set_all_to_false();
    pose_graph_slam.is_optimize_pcl_ndt = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::pcl_ndt;

    auto start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(session.point_clouds_container);
    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    rms = compute_rms(false, session, observation_picking);
    id_method = 94;
    append_to_result_file(result_file, "pose_graph_slam (pcl_ndt)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;

    //--95--
    pose_graph_slam.set_all_to_false();
    pose_graph_slam.is_optimize_pcl_icp = true;
    pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::pcl_icp;

    start = std::chrono::system_clock::now();
    pose_graph_slam.optimize(session.point_clouds_container);
    end = std::chrono::system_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    rms = compute_rms(false, session, observation_picking);
    id_method = 95;
    append_to_result_file(result_file, "pose_graph_slam (pcl_icp)", pose_graph_slam, rms, id_method, elapsed);
    export_result_to_folder(path_result.string(), id_method, observation_picking, session);
    session.point_clouds_container = temp_data;
#endif

#if WITH_GTSAM
    //--96--
    try
    {
        pose_graph_slam.ndt_bucket_size[0] = ndt.bucket_size[0];
        pose_graph_slam.ndt_bucket_size[1] = ndt.bucket_size[1];
        pose_graph_slam.ndt_bucket_size[2] = ndt.bucket_size[2];

        pose_graph_slam.set_all_to_false();
        pose_graph_slam.is_optimize_pcl_ndt = true;
        pose_graph_slam.is_lie_algebra_right_jacobian = true;
        pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::pcl_ndt;

        auto start = std::chrono::system_clock::now();
        pose_graph_slam.optimize_with_GTSAM(session.point_clouds_container);
        auto end = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        rms = compute_rms(false, session, observation_picking);
        id_method = 96;
        append_to_result_file(result_file, "pose_graph_slam (GTSAM pcl_ndt)", pose_graph_slam, rms, id_method, elapsed);
        export_result_to_folder(path_result.string(), id_method, observation_picking, session);
        session.point_clouds_container = temp_data;
    } catch (std::exception& e)
    {
        std::cout << e.what() << std::endl;
        rms = compute_rms(false, session, observation_picking);
        // append_to_result_file(result_file, "pose_graph_slam (GTSAM pcl_ndt)", pose_graph_slam, rms, id_method, elapsed);
        export_result_to_folder(path_result.string(), id_method, observation_picking, session);
        session.point_clouds_container = temp_data;
    }
    //--97--
    try
    {
        pose_graph_slam.set_all_to_false();
        pose_graph_slam.is_optimize_pcl_icp = true;
        pose_graph_slam.is_lie_algebra_right_jacobian = true;
        pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::pcl_icp;

        auto start = std::chrono::system_clock::now();
        pose_graph_slam.optimize_with_GTSAM(session.point_clouds_container);
        auto end = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        rms = compute_rms(false, session, observation_picking);
        id_method = 97;
        append_to_result_file(result_file, "pose_graph_slam (GTSAM pcl_icp)", pose_graph_slam, rms, id_method, elapsed);
        export_result_to_folder(path_result.string(), id_method, observation_picking, session);
        session.point_clouds_container = temp_data;
    } catch (std::exception& e)
    {
        std::cout << e.what() << std::endl;
        rms = compute_rms(false, session, observation_picking);
        // append_to_result_file(result_file, "pose_graph_slam (GTSAM pcl_icp)", pose_graph_slam, rms, id_method, elapsed);
        export_result_to_folder(path_result.string(), id_method, observation_picking, session);
        session.point_clouds_container = temp_data;
    }
#endif
#if WITH_MANIF
    //--98--
    {
        pose_graph_slam.set_all_to_false();
        pose_graph_slam.is_optimize_pcl_ndt = true;
        pose_graph_slam.is_lie_algebra_right_jacobian = true;
        pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::pcl_ndt;

        auto start = std::chrono::system_clock::now();
        pose_graph_slam.optimize_with_manif(session.point_clouds_container);
        auto end = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        rms = compute_rms(false, session, observation_picking);
        id_method = 98;
        append_to_result_file(result_file, "pose_graph_slam (manif pcl_ndt)", pose_graph_slam, rms, id_method, elapsed);
        export_result_to_folder(path_result.string(), id_method, observation_picking, session);
        session.point_clouds_container = temp_data;

        //--99--
        pose_graph_slam.set_all_to_false();
        pose_graph_slam.is_optimize_pcl_icp = true;
        pose_graph_slam.is_lie_algebra_right_jacobian = true;
        pose_graph_slam.pair_wise_matching_type = PoseGraphSLAM::PairWiseMatchingType::pcl_icp;

        start = std::chrono::system_clock::now();
        pose_graph_slam.optimize_with_manif(session.point_clouds_container);
        end = std::chrono::system_clock::now();
        elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        rms = compute_rms(false, session, observation_picking);
        id_method = 99;
        append_to_result_file(result_file, "pose_graph_slam (manif pcl_icp)", pose_graph_slam, rms, id_method, elapsed);
        export_result_to_folder(path_result.string(), id_method, observation_picking, session);
        session.point_clouds_container = temp_data;
    }
#endif
}

#if 0
bool exportLaz(const std::string &filename,
               const std::vector<Eigen::Vector3d> &pointcloud,
               const std::vector<unsigned short> &intensity, double offset_x, double offset_y, double offset_alt)
{

    constexpr float scale = 0.0001f; // one tenth of milimeter
    // find max
    Eigen::Vector3d max(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest());
    Eigen::Vector3d min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    for (auto &p : pointcloud)
    {
        max.x() = std::max(max.x(), p.x());
        max.y() = std::max(max.y(), p.y());
        max.z() = std::max(max.z(), p.z());

        min.x() = std::min(min.x(), p.x());
        min.y() = std::min(min.y(), p.y());
        min.z() = std::min(min.z(), p.z());
    }

    std::cout << "exportLaz to file: '" << filename << "'" << std::endl;
    std::cout << std::setprecision(20) << "min.x " << min.x() << " " << max.x() << std::endl;
    std::cout << "min.y " << min.y() << " " << max.y() << std::endl;
    std::cout << "min.z " << min.z() << " " << max.z() << std::endl;

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

    header->max_x = max.x() + offset_x;
    header->min_x = min.x() + offset_x;
    header->max_y = max.y() + offset_y;
    header->min_y = min.y() + offset_y;
    header->max_z = max.z() + offset_alt;
    header->min_z = min.z() + offset_alt;

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
        coordinates[0] = p.x();// + offset_x;
        coordinates[1] = p.y();// + offset_y;
        coordinates[2] = p.z();// + offset_alt;
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
#endif

#if 0
//void export_result_to_folder(std::string output_folder_name, ObservationPicking &observation_picking, Session &session);
void export_result_to_folder(std::string output_folder_name, ObservationPicking &observation_picking, Session &session)
{
    fs::path path(output_folder_name);
    std::string file_name_rms = "rms.csv";
    auto path_rms = path;
    path_rms /= file_name_rms;
    std::cout << "exporting to file: '" << path_rms.string() << "'" << std::endl;
    std::ofstream outfile_rms;
    outfile_rms.open(path_rms, std::ios_base::app);
    outfile_rms << "index_roi, rms_initial, rms_result" << std::endl;

    for (int i = 0; i < observation_picking.intersections.size(); i++)
    {
        std::string file_name_initial = "intersection_" + std::to_string(i) + "_initial.csv";
        std::string file_name_result = "intersection_" + std::to_string(i) + "_result.csv";

        auto path_initial = path;
        auto path_result = path;

        path_initial /= file_name_initial;
        path_result /= file_name_result;

        std::cout << "exporting to file: '" << path_initial.string() << "'" << std::endl;
        std::cout << "exporting to file: '" << path_result.string() << "'" << std::endl;

        std::ofstream outfile_initial;
        std::ofstream outfile_result;

        outfile_initial.open(path_initial, std::ios_base::app);
        outfile_result.open(path_result, std::ios_base::app);

        const auto &intersection = observation_picking.intersections[i];
        TaitBryanPose pose;
        pose.px = intersection.translation[0];
        pose.py = intersection.translation[1];
        pose.pz = intersection.translation[2];
        pose.om = intersection.rotation[0];
        pose.fi = intersection.rotation[1];
        pose.ka = intersection.rotation[2];
        Eigen::Affine3d m_pose_inv = affine_matrix_from_pose_tait_bryan(pose).inverse();

        double w = intersection.width_length_height[0] * 0.5;
        double l = intersection.width_length_height[1] * 0.5;
        double h = intersection.width_length_height[2] * 0.5;

        outfile_initial << "x;y;z;pc_index;is_initial;index_intersection;file" << std::endl;
        outfile_result << "x;y;z;pc_index;is_initial;index_intersection;file" << std::endl;

        for (int pc_index = 0; pc_index < session.point_clouds_container.point_clouds.size(); pc_index++)
        {
            const auto &pc = session.point_clouds_container.point_clouds[pc_index];
            for (const auto &p : pc.points_local)
            {
                Eigen::Vector3d vpi = pc.m_initial_pose * p;
                Eigen::Vector3d vpr = pc.m_pose * p;

                Eigen::Vector3d vpit = m_pose_inv * vpi;
                Eigen::Vector3d vprt = m_pose_inv * vpr;

                if (fabs(vpit.x()) < w)
                {
                    if (fabs(vpit.y()) < l)
                    {
                        if (fabs(vpit.z()) < h)
                        {
                            outfile_initial << vpit.x() << ";" << vpit.y() << ";" << vpit.z() << ";" << pc_index << ";1;" << i << ";" << pc.file_name << std::endl;
                        }
                    }
                }
                if (fabs(vprt.x()) < w)
                {
                    if (fabs(vprt.y()) < l)
                    {
                        if (fabs(vprt.z()) < h)
                        {
                            outfile_result << vprt.x() << ";" << vprt.y() << ";" << vprt.z() << ";" << pc_index << ";0;" << i << ";" << pc.file_name << std::endl;
                        }
                    }
                }
            }
        }
        outfile_initial.close();
        outfile_result.close();

        const auto &obs = observation_picking.observations[i];
        double rms_initial = 0.0;
        int sum = 0;
        double rms_result = 0.0;

        for (const auto &[key1, value1] : obs)
        {
            for (const auto &[key2, value2] : obs)
            {
                if (key1 != key2)
                {
                    Eigen::Vector3d p1, p2;
                    p1 = session.point_clouds_container.point_clouds[key1].m_initial_pose * value1;
                    p2 = session.point_clouds_container.point_clouds[key2].m_initial_pose * value2;
                    rms_initial += (p2.x() - p1.x()) * (p2.x() - p1.x());
                    rms_initial += (p2.y() - p1.y()) * (p2.y() - p1.y());

                    p1 = session.point_clouds_container.point_clouds[key1].m_pose * value1;
                    p2 = session.point_clouds_container.point_clouds[key2].m_pose * value2;
                    rms_result += (p2.x() - p1.x()) * (p2.x() - p1.x());
                    rms_result += (p2.y() - p1.y()) * (p2.y() - p1.y());

                    sum += 2;
                }
            }
        }
        std::cout << "sum: " << sum << std::endl;
        if (sum > 0)
        {
            rms_initial = sqrt(rms_initial / sum);
            rms_result = sqrt(rms_result / sum);
            outfile_rms << i << ";" << rms_initial << ";" << rms_result << std::endl;
        }
    }
    outfile_rms.close();

    std::string file_name_poses = "poses_RESSO.reg";
    auto path_poses = path;
    path_poses /= file_name_poses;
    std::cout << "saving poses to: " << path_poses << std::endl;
    session.point_clouds_container.save_poses(path_poses.string(), false);
}


//void export_result_to_folder(std::string output_folder_name, ObservationPicking &observation_picking, Session &session)
void export_result_to_folder(std::string output_folder_name, int method_id, ObservationPicking &observation_picking)
{
    fs::path path(output_folder_name);
    path /= std::to_string(method_id);
    create_directory(path);
    export_result_to_folder(path.string(), observation_picking);
}

#endif
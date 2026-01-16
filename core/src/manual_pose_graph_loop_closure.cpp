#include <pch/pch.h>

#if WITH_GUI == 1
#include <icp.h>
#include <manual_pose_graph_loop_closure.h>
#include <pair_wise_iterative_closest_point.h>
#include <transformations.h>

#include <python-scripts/constraints/quaternion_constraint_jacobian.h>
#include <python-scripts/constraints/relative_pose_quaternion_cw_jacobian.h>
#include <python-scripts/constraints/relative_pose_quaternion_wc_jacobian.h>
#include <python-scripts/constraints/relative_pose_rodrigues_wc_jacobian.h>
#include <python-scripts/constraints/relative_pose_tait_bryan_cw_jacobian.h>
#include <python-scripts/constraints/relative_pose_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-feature-metrics/point_to_line_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian.h>

#include <gnss.h>
#include <m_estimators.h>

#include <common/include/cauchy.h>

#include <pfd_wrapper.hpp>

#include "export_laz.h"

#include <utils.hpp>

void ManualPoseGraphLoopClosure::Gui(
    PointClouds& point_clouds_container,
    int& index_loop_closure_source,
    int& index_loop_closure_target,
    float* m_gizmo,
    GNSS& gnss,
    GroundControlPoints& gcps,
    ControlPoints& cps,
    int num_edge_extended_before,
    int num_edge_extended_after)
{
    if (point_clouds_container.point_clouds.size() > 0)
    {
        if (!manipulate_active_edge)
        {
            // suboptimal as distance is computed each frame!
            double distance = (point_clouds_container.point_clouds[index_loop_closure_target].m_pose.translation() -
                               point_clouds_container.point_clouds[index_loop_closure_source].m_pose.translation())
                                  .norm();
            ImGui::Text("Loop closure indexes: (Length of selected edge: %.3f [m])", distance);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Select indexes from going/returning trips to same point that should overlap");

            ImGui::Text("Source: ");
            ImGui::SameLine();
            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::SliderInt("##lcss", &index_loop_closure_source, 0, point_clouds_container.point_clouds.size() - 1);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("max %zu", point_clouds_container.point_clouds.size() - 1);
            ImGui::SameLine();
            ImGui::InputInt("##lcsi", &index_loop_closure_source, 1, 5);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("max %zu", point_clouds_container.point_clouds.size() - 1);
            if (index_loop_closure_source < 0)
                index_loop_closure_source = 0;
            if (index_loop_closure_source >= point_clouds_container.point_clouds.size() - 1)
                index_loop_closure_source = point_clouds_container.point_clouds.size() - 1;

            ImGui::SameLine();
            ImGui::Text("press shift + middle button ");

            ImGui::Text("Target: ");
            ImGui::SameLine();

            ImGui::SliderInt("##lcts", &index_loop_closure_target, 0, point_clouds_container.point_clouds.size() - 1);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("max %zu", point_clouds_container.point_clouds.size() - 1);
            ImGui::SameLine();
            ImGui::InputInt("##lcti", &index_loop_closure_target, 1, 5);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("max %zu", point_clouds_container.point_clouds.size() - 1);
            if (index_loop_closure_target < 0)
                index_loop_closure_target = 0;
            if (index_loop_closure_target >= point_clouds_container.point_clouds.size() - 1)
                index_loop_closure_target = point_clouds_container.point_clouds.size() - 1;
            ImGui::PopItemWidth();

            ImGui::SameLine();
            ImGui::Text("press ctrl + middle button ");

            if (ImGui::Button("Add edge"))
            {
                add_edge(point_clouds_container, index_loop_closure_source, index_loop_closure_target);
                index_active_edge = edges.size() - 1;
            }
            ImGui::SameLine();
            ImGui::Text("(number of active edges: %zu)", edges.size());

            if (!manipulate_active_edge)
            {
                ImGui::Text("Motion model setting:");
                ImGui::SameLine();
                if (ImGui::Button("initial poses"))
                    set_initial_poses_as_motion_model(point_clouds_container);
                ImGui::SameLine();
                if (ImGui::Button("current result"))
                    set_current_poses_as_motion_model(point_clouds_container);

                if (poses_motion_model.size() == point_clouds_container.point_clouds.size())
                {
                    ImGui::SameLine();
                    if (ImGui::Button("Compute Pose Graph SLAM"))
                    {
                        graph_slam(point_clouds_container, gnss, gcps, cps);
                    }
                }
                // ImGui::Text("motion model");

                ImGui::Separator();

                ImGui::Text("Motion model sigmas [m] / [deg]:");
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("position/rotational uncertainties");

                ImGui::PushItemWidth(ImGuiNumberWidth);
                ImGui::InputDouble("px_1", &motion_model_w_px_1_sigma_m, 0.0, 0.0, "%.4f");
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(xText);
                ImGui::SameLine();
                ImGui::InputDouble("om_1", &motion_model_w_om_1_sigma_deg, 0.0, 0.0, "%.3f");
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(omText);

                ImGui::InputDouble("py_1", &motion_model_w_py_1_sigma_m, 0.0, 0.0, "%.4f");
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(yText);
                ImGui::SameLine();
                ImGui::InputDouble("fi_1", &motion_model_w_fi_1_sigma_deg, 0.0, 0.0, "%.3f");
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(fiText);

                ImGui::InputDouble("pz_1", &motion_model_w_pz_1_sigma_m, 0.0, 0.0, "%.4f");
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(zText);
                ImGui::SameLine();
                ImGui::InputDouble("ka_1", &motion_model_w_ka_1_sigma_deg, 0.0, 0.0, "%.3f");
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(kaText);

                ImGui::PopItemWidth();
                ImGui::Separator();

                if (ImGui::Button("Fuse trajectory with GNSS (trajectory is rigid)"))
                {
                    FuseTrajectoryWithGNSS(point_clouds_container, gnss);
                }
            }
        }

        if (edges.size() > 0)
            ImGui::Checkbox("Manipulate active edge", &manipulate_active_edge);

        if (edges.size() > 0 && manipulate_active_edge)
        {
            ImGui::SameLine();
            bool prev_gizmo = gizmo;
            ImGui::Checkbox("Gizmo", &gizmo);

            if (!gizmo)
            {
                if (ImGui::Button("ICP"))
                    run_icp(
                        point_clouds_container, index_active_edge, search_radious, 10, num_edge_extended_before, num_edge_extended_after);
                ImGui::SameLine();
                ImGui::SetNextItemWidth(ImGuiNumberWidth);
                ImGui::InputDouble("Search_radius [m]", &search_radious);
                if (search_radious < 0.01)
                    search_radious = 0.01;

                if (ImGui::Button("ICP (2.0)"))
                    run_icp(point_clouds_container, index_active_edge, 2.0, 30, num_edge_extended_before, num_edge_extended_after);
                ImGui::SameLine();
                if (ImGui::Button("ICP (1.0)"))
                    run_icp(point_clouds_container, index_active_edge, 1.0, 30, num_edge_extended_before, num_edge_extended_after);
                ImGui::SameLine();
                if (ImGui::Button("ICP (0.5)"))
                    run_icp(point_clouds_container, index_active_edge, 0.5, 30, num_edge_extended_before, num_edge_extended_after);
                ImGui::SameLine();
                if (ImGui::Button("ICP (0.25)"))
                    run_icp(point_clouds_container, index_active_edge, 0.25, 30, num_edge_extended_before, num_edge_extended_after);
                ImGui::SameLine();
                if (ImGui::Button("ICP (0.1)"))
                    run_icp(point_clouds_container, index_active_edge, 0.1, 30, num_edge_extended_before, num_edge_extended_after);

                if (ImGui::Button("Save source"))
                {
                    const auto output_file_name = mandeye::fd::SaveFileDialog("Output file name", mandeye::fd::LAS_LAZ_filter, ".laz");
                    std::cout << "laz file to save: '" << output_file_name << "'" << std::endl;

                    if (output_file_name.size() > 0)
                    {
                        std::vector<Eigen::Vector3d> source;
                        auto& e = edges[index_active_edge];
                        for (int i = -num_edge_extended_before; i <= num_edge_extended_after; i++)
                        {
                            int index_src = e.index_to + i;
                            if (index_src >= 0 && index_src < point_clouds_container.point_clouds.size())
                            {
                                Eigen::Affine3d m_src = point_clouds_container.point_clouds.at(index_src).m_pose;
                                for (int k = 0; k < point_clouds_container.point_clouds[index_src].points_local.size(); k++)
                                {
                                    Eigen::Vector3d p_g = m_src * point_clouds_container.point_clouds[index_src].points_local[k];
                                    source.push_back(p_g);
                                }
                                // point_clouds_container.point_clouds.at(index_src).render(m_src, 1);
                            }
                        }

                        Eigen::Affine3d m_src_inv = point_clouds_container.point_clouds[e.index_to].m_pose.inverse();
                        for (auto& p : source)
                        {
                            p = m_src_inv * p;
                        }

                        auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                        for (auto& p : source)
                        {
                            p = m_pose * p;
                        }

                        std::vector<unsigned short> intensity;
                        std::vector<double> timestamps;

                        for (int i = 0; i < source.size(); i++)
                        {
                            intensity.push_back(0);
                            timestamps.push_back(0.0);
                        }
                        exportLaz(output_file_name, source, intensity, timestamps);
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button("Save target (transformed only by rotation)"))
                {
                    const auto output_file_name = mandeye::fd::SaveFileDialog("Output file name", mandeye::fd::LAS_LAZ_filter, ".laz");
                    std::cout << "laz file to save: '" << output_file_name << "'" << std::endl;

                    if (output_file_name.size() > 0)
                    {
                        std::vector<Eigen::Vector3d> target;
                        auto& e = edges[index_active_edge];
                        for (int i = -num_edge_extended_before; i <= num_edge_extended_after; i++)
                        {
                            int index_trg = e.index_from + i;
                            if (index_trg >= 0 && index_trg < point_clouds_container.point_clouds.size())
                            {
                                Eigen::Affine3d m_trg = point_clouds_container.point_clouds.at(index_trg).m_pose;
                                for (int k = 0; k < point_clouds_container.point_clouds[index_trg].points_local.size(); k++)
                                {
                                    Eigen::Vector3d p_g = m_trg * point_clouds_container.point_clouds[index_trg].points_local[k];
                                    target.push_back(p_g);
                                }
                            }
                        }

                        Eigen::Affine3d m_trg_inv = point_clouds_container.point_clouds[e.index_from].m_pose.inverse();

                        for (auto& p : target)
                        {
                            p = m_trg_inv * p;
                        }

                        std::vector<unsigned short> intensity;
                        std::vector<double> timestamps;

                        for (int i = 0; i < target.size(); i++)
                        {
                            intensity.push_back(150);
                            timestamps.push_back(0.0);
                        }

                        exportLaz(output_file_name, target, intensity, timestamps);
                    }
                }
            }

            ImGui::Separator();

            int prev_index_active_edge = index_active_edge;
            ImGui::BeginChild("Edges", ImVec2(0, 0), true);
            {
                for (size_t i = 0; i < edges.size(); i++)
                {
                    if (i > 0)
                    {
                        ImGui::Separator();
                    }
                    ImGui::SetWindowFontScale(1.25f);
                    ImGui::RadioButton(("Active edge " + std::to_string(i)).c_str(), &index_active_edge, i);
                    ImGui::SetWindowFontScale(1.0f);

                    // suboptimal as distance is computed each frame!
                    double distance = (point_clouds_container.point_clouds[edges[i].index_to].m_pose.translation() -
                                       point_clouds_container.point_clouds[edges[i].index_from].m_pose.translation())
                                          .norm();

                    ImGui::SameLine();
                    ImGui::Text("(length [m]: %.3f)", distance);
                    ImGui::SameLine();

                    if (ImGui::Button(("Remove##" + std::to_string(i)).c_str()))
                    {
                        gizmo = false;

                        std::vector<Edge> new_edges;
                        for (int ni = 0; ni < edges.size(); ni++)
                        {
                            if (i != ni)
                                new_edges.push_back(edges[ni]);
                        }
                        edges = new_edges;

                        index_active_edge = std::min(index_active_edge, static_cast<int>(edges.size() - 1));
                        manipulate_active_edge = (edges.size() > 0);
                    }

                    if (index_active_edge == i)
                    {
                        ImGui::SameLine();

                        // if(session.)

                        static double downsampling_voxel_size = 0.1;

                        if (ImGui::Button(("Reload cache##" + std::to_string(i)).c_str()))
                        {
                            // std::cout << edges[index_active_edge].index_from

                            std::cout << point_clouds_container.point_clouds[edges[index_active_edge].index_from].file_name << std::endl;
                            std::cout << point_clouds_container.point_clouds[edges[index_active_edge].index_to].file_name << std::endl;

                            point_clouds_container.point_clouds[edges[index_active_edge].index_from].load_pc(
                                point_clouds_container.point_clouds[edges[index_active_edge].index_from].file_name, true);
                            point_clouds_container.point_clouds[edges[index_active_edge].index_from].decimate(
                                downsampling_voxel_size, downsampling_voxel_size, downsampling_voxel_size);

                            point_clouds_container.point_clouds[edges[index_active_edge].index_to].load_pc(
                                point_clouds_container.point_clouds[edges[index_active_edge].index_to].file_name, true);
                            point_clouds_container.point_clouds[edges[index_active_edge].index_to].decimate(
                                downsampling_voxel_size, downsampling_voxel_size, downsampling_voxel_size);

                            // gizmo = false;

                            // std::vector<Edge> new_edges;
                            // for (int ni = 0; ni < edges.size(); ni++)
                            //{
                            //     if (i != ni)
                            //         new_edges.push_back(edges[ni]);
                            // }
                            // edges = new_edges;

                            // index_active_edge = std::min(index_active_edge, static_cast<int>(edges.size() - 1));
                            // manipulate_active_edge = (edges.size() > 0);

                            // for (int i = 0; i < point_clouds_container.point_clouds.size(); i++)
                            // {
                            // }
                        }

                        ImGui::SameLine();
                        ImGui::InputDouble("downsampling_voxel_size ", &downsampling_voxel_size);
                    }

                    ImGui::BeginDisabled(true);
                    ImGui::PushItemWidth(ImGuiNumberWidth);
                    ImGui::InputInt(("Source##" + std::to_string(i)).c_str(), &edges[i].index_from);
                    ImGui::SameLine();
                    ImGui::InputInt(("Target##" + std::to_string(i)).c_str(), &edges[i].index_to);
                    ImGui::PopItemWidth();
                    ImGui::EndDisabled();
                }
            }
            ImGui::EndChild();

            if ((prev_gizmo != gizmo) || (prev_index_active_edge != index_active_edge))
            {
                auto m_to = point_clouds_container.point_clouds[edges[index_active_edge].index_from].m_pose *
                    affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                m_gizmo[0] = (float)m_to(0, 0);
                m_gizmo[1] = (float)m_to(1, 0);
                m_gizmo[2] = (float)m_to(2, 0);
                m_gizmo[3] = (float)m_to(3, 0);
                m_gizmo[4] = (float)m_to(0, 1);
                m_gizmo[5] = (float)m_to(1, 1);
                m_gizmo[6] = (float)m_to(2, 1);
                m_gizmo[7] = (float)m_to(3, 1);
                m_gizmo[8] = (float)m_to(0, 2);
                m_gizmo[9] = (float)m_to(1, 2);
                m_gizmo[10] = (float)m_to(2, 2);
                m_gizmo[11] = (float)m_to(3, 2);
                m_gizmo[12] = (float)m_to(0, 3);
                m_gizmo[13] = (float)m_to(1, 3);
                m_gizmo[14] = (float)m_to(2, 3);
                m_gizmo[15] = (float)m_to(3, 3);
            }
        }
    }
}

void ManualPoseGraphLoopClosure::Render(
    PointClouds& point_clouds_container,
    int index_loop_closure_source,
    int index_loop_closure_target,
    int num_edge_extended_before,
    int num_edge_extended_after)
{
    point_clouds_container.point_clouds.at(index_loop_closure_source).visible = true;
    point_clouds_container.point_clouds.at(index_loop_closure_target).visible = true;

    if (!manipulate_active_edge)
    {
        for (int i = index_loop_closure_source - num_edge_extended_before; i <= index_loop_closure_source + num_edge_extended_after; i++)
        {
            if (i >= 0 && i < point_clouds_container.point_clouds.size() && point_clouds_container.point_clouds.size() > 0)
            {
                ObservationPicking observation_picking;
                point_clouds_container.point_clouds.at(i).render(false, observation_picking, 1, false, false, false, 10000, false);
            }
        }

        for (int i = index_loop_closure_target - num_edge_extended_before; i <= index_loop_closure_target + num_edge_extended_after; i++)
        {
            if (i >= 0 && i < point_clouds_container.point_clouds.size() && point_clouds_container.point_clouds.size() > 0)
            {
                ObservationPicking observation_picking;
                point_clouds_container.point_clouds.at(i).render(false, observation_picking, 1, false, false, false, 10000, false);
            }
        }
    }
    else
    {
        if (edges.size() > 0)
        {
            for (int i = -num_edge_extended_before; i <= num_edge_extended_after; i++)
            {
                int index_src = edges[index_active_edge].index_from + i;
                if (index_src >= 0 && index_src < point_clouds_container.point_clouds.size())
                {
                    Eigen::Affine3d m_src = point_clouds_container.point_clouds.at(index_src).m_pose;
                    point_clouds_container.point_clouds.at(index_src).render(m_src, 1);
                }
            }

            for (int i = -num_edge_extended_before; i <= num_edge_extended_after; i++)
            {
                int index_trg = edges[index_active_edge].index_to;
                int index_src = edges[index_active_edge].index_from;
                Eigen::Affine3d m_src = point_clouds_container.point_clouds.at(index_src).m_pose;

                if (index_trg + i >= 0 && index_trg + i < point_clouds_container.point_clouds.size())
                {
                    Eigen::Affine3d m_trg = m_src * affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                    Eigen::Affine3d m_rel = point_clouds_container.point_clouds.at(index_trg).m_pose.inverse() *
                        point_clouds_container.point_clouds.at(index_trg + i).m_pose;
                    m_trg = m_trg * m_rel;
                    point_clouds_container.point_clouds.at(index_trg + i).render(m_trg, 1);
                }
            }
        }
    }

    glColor3f(0, 1, 0);
    glBegin(GL_LINE_STRIP);

    for (auto& pc : point_clouds_container.point_clouds)
    {
        glVertex3f(pc.m_pose(0, 3), pc.m_pose(1, 3), pc.m_pose(2, 3));
    }
    glEnd();

    int i = 0;
    for (auto& pc : point_clouds_container.point_clouds)
    {
        glRasterPos3f(pc.m_pose(0, 3), pc.m_pose(1, 3), pc.m_pose(2, 3) + 0.1);
        glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, (const unsigned char*)std::to_string(i).c_str());
        i++;
    }

    for (int i = 0; i < edges.size(); i++)
    {
        if (i == index_active_edge)
        {
            glColor3f(1, 0, 0);
        }
        else
        {
            glColor3f(0, 0, 1);
        }

        int index_src = edges[i].index_from;
        int index_trg = edges[i].index_to;

        Eigen::Affine3d m_src = point_clouds_container.point_clouds.at(index_src).m_pose;
        Eigen::Affine3d m_trg = point_clouds_container.point_clouds.at(index_trg).m_pose;

        glBegin(GL_LINES);
        glVertex3f(m_src(0, 3), m_src(1, 3), m_src(2, 3));
        glVertex3f(m_trg(0, 3), m_trg(1, 3), m_trg(2, 3));
        glEnd();

        Eigen::Vector3d m1((m_src(0, 3) + m_trg(0, 3)) * 0.5, (m_src(1, 3) + m_trg(1, 3)) * 0.5, (m_src(2, 3) + m_trg(2, 3)) * 0.5);
        Eigen::Vector3d m2((m_src(0, 3) + m_trg(0, 3)) * 0.5, (m_src(1, 3) + m_trg(1, 3)) * 0.5, (m_src(2, 3) + m_trg(2, 3)) * 0.5 + 10);

        glBegin(GL_LINES);
        glVertex3f(m1.x(), m1.y(), m1.z());
        glVertex3f(m2.x(), m2.y(), m2.z());
        glEnd();

        glRasterPos3f(m2.x(), m2.y(), m2.z() + 0.1);
        glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, (const unsigned char*)std::to_string(i).c_str());
    }
}

#endif

#if WITH_GUI == 1
#include <manual_pose_graph_loop_closure.h>
#include <pair_wise_iterative_closest_point.h>
#include <icp.h>
#include <transformations.h>
#include <python-scripts/constraints/relative_pose_tait_bryan_wc_jacobian.h>
#include <python-scripts/constraints/relative_pose_tait_bryan_cw_jacobian.h>

#include <python-scripts/constraints/relative_pose_rodrigues_wc_jacobian.h>

#include <python-scripts/constraints/relative_pose_quaternion_wc_jacobian.h>
#include <python-scripts/constraints/relative_pose_quaternion_cw_jacobian.h>
#include <python-scripts/constraints/quaternion_constraint_jacobian.h>

#include <m_estimators.h>
#include <gnss.h>

#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-feature-metrics/point_to_line_tait_bryan_wc_jacobian.h>

#include <common/include/cauchy.h>

#include <pfd_wrapper.hpp>

#include "export_laz.h"

#include <utils.hpp>

void ManualPoseGraphLoopClosure::Gui(PointClouds &point_clouds_container,
                                     int &index_loop_closure_source, int &index_loop_closure_target,
                                     float *m_gizmo, GNSS &gnss, GroundControlPoints &gcps, ControlPoints &cps,
                                     int num_edge_extended_before, int num_edge_extended_after)
{
    if (point_clouds_container.point_clouds.size() > 0)
    {
        if (!manipulate_active_edge)
        {
            ImGui::Text("Index loop closure:");
            ImGui::SameLine();
            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::InputInt("source", &index_loop_closure_source);
            if (index_loop_closure_source < 0)
                index_loop_closure_source = 0;
            if (index_loop_closure_source >= point_clouds_container.point_clouds.size() - 1)
                index_loop_closure_source = point_clouds_container.point_clouds.size() - 1;
            ImGui::SameLine();
            ImGui::InputInt("target", &index_loop_closure_target);
            if (index_loop_closure_target < 0)
                index_loop_closure_target = 0;
            if (index_loop_closure_target >= point_clouds_container.point_clouds.size() - 1)
                index_loop_closure_target = point_clouds_container.point_clouds.size() - 1;
            ImGui::PopItemWidth();

            if (ImGui::Button("Add edge"))
            {
                add_edge(point_clouds_container, index_loop_closure_source, index_loop_closure_target);
                index_active_edge = edges.size() - 1;
            }

            if (!manipulate_active_edge)
            {
                ImGui::Text("Motion model setting:");
                ImGui::SameLine();
                if (ImGui::Button("initial poses"))
                    set_initial_poses_as_motion_model(point_clouds_container);
                ImGui::SameLine();
                if (ImGui::Button("current result"))
                    set_current_poses_as_motion_model(point_clouds_container);

                // ImGui::Text("motion model");

                // session.pose_graph_loop_closure.edges.
                
                // ImGui::Separator();


                if (poses_motion_model.size() == point_clouds_container.point_clouds.size())
                {
                    ImGui::SameLine();
                    if (ImGui::Button("Compute Pose Graph SLAM"))
                    {
                        graph_slam(point_clouds_container, gnss, gcps, cps);
                    }
                }

                ImGui::Separator();
                ImGui::InputDouble("motion_model_px_1_sigma_m", &motion_model_w_px_1_sigma_m);
                ImGui::InputDouble("motion_model_py_1_sigma_m", &motion_model_w_py_1_sigma_m);
                ImGui::InputDouble("motion_model_pz_1_sigma_m", &motion_model_w_pz_1_sigma_m);
                ImGui::InputDouble("motion_model_om_1_sigma_deg", &motion_model_w_om_1_sigma_deg);
                ImGui::InputDouble("motion_model_fi_1_sigma_deg", &motion_model_w_fi_1_sigma_deg);
                ImGui::InputDouble("motion_model_ka_1_sigma_deg", &motion_model_w_ka_1_sigma_deg);
                ImGui::Separator();

                if (ImGui::Button("Fuse trajectory with GNSS (trajectory is rigid)"))
                {
                    FuseTrajectoryWithGNSS(point_clouds_container, gnss);
                }
            }
        }

        std::string number_active_edges = "number_edges: " + std::to_string(edges.size());
        ImGui::Text(number_active_edges.c_str());

        ImGui::Checkbox("manipulate_active_edge", &manipulate_active_edge);

        if (edges.size() > 0 && manipulate_active_edge)
        {
            // ImGui::SameLine();
            int remove_edge_index = -1;
            if (ImGui::Button("remove active edge"))
            {
                gizmo = false;
                remove_edge_index = index_active_edge;
                // program_params.is_edge_gizmo = false;
            }

            int prev_index_active_edge = index_active_edge;
            ImGui::SetNextItemWidth(ImGuiNumberWidth);
            ImGui::InputInt("index_active_edge", &index_active_edge);

            if (index_active_edge < 0)
            {
                index_active_edge = 0;
            }
            if (index_active_edge >= (int)edges.size())
            {
                index_active_edge = (int)edges.size() - 1;
            }

            bool prev_gizmo = gizmo;
            ImGui::Checkbox("gizmo", &gizmo);

            if (prev_gizmo != gizmo)
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

            ////////////////////////////////////////
            if (remove_edge_index != -1)
            {
                std::vector<Edge> new_edges;
                for (int i = 0; i < edges.size(); i++)
                {
                    if (remove_edge_index != i)
                    {
                        new_edges.push_back(edges[i]);
                    }
                }
                edges = new_edges;

                index_active_edge = remove_edge_index - 1;
            }

            if (!gizmo)
            {
                if (ImGui::Button("ICP"))
                    run_icp(point_clouds_container, index_active_edge, search_radious, 10, num_edge_extended_before, num_edge_extended_after);
                ImGui::SameLine();
                ImGui::SetNextItemWidth(ImGuiNumberWidth);
                ImGui::InputDouble("search_radius [m]", &search_radious);
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

                if (ImGui::Button("Save src"))
                {
                    const auto output_file_name = mandeye::fd::SaveFileDialog("Output file name", mandeye::fd::LAS_LAZ_filter, ".laz");
                    std::cout << "laz file to save: '" << output_file_name << "'" << std::endl;

                    if (output_file_name.size() > 0)
                    {
                        std::vector<Eigen::Vector3d> source;
                        auto &e = edges[index_active_edge];
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
                        for (auto &p : source)
                        {
                            p = m_src_inv * p;
                        }

                        auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                        for (auto &p : source)
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
                        exportLaz(
                            output_file_name,
                            source,
                            intensity,
                            timestamps);
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button("Save trg (transfromed only by rotation)"))
                {
                    const auto output_file_name = mandeye::fd::SaveFileDialog("Output file name", mandeye::fd::LAS_LAZ_filter, ".laz");
                    std::cout << "laz file to save: '" << output_file_name << "'" << std::endl;

                    if (output_file_name.size() > 0)
                    {
                        std::vector<Eigen::Vector3d> target;
                        auto &e = edges[index_active_edge];
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

                        for (auto &p : target)
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

                        exportLaz(
                            output_file_name,
                            target,
                            intensity,
                            timestamps);
                    }
                }
            }
        }
    }
}

void ManualPoseGraphLoopClosure::Render(PointClouds &point_clouds_container,
                                        int index_loop_closure_source, int index_loop_closure_target,
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
                point_clouds_container.point_clouds.at(i).render(false, observation_picking, 1, false, false, false, 10000);
            }
        }

        for (int i = index_loop_closure_target - num_edge_extended_before; i <= index_loop_closure_target + num_edge_extended_after; i++)
        {
            if (i >= 0 && i < point_clouds_container.point_clouds.size() && point_clouds_container.point_clouds.size() > 0)
            {
                ObservationPicking observation_picking;
                point_clouds_container.point_clouds.at(i).render(false, observation_picking, 1, false, false, false, 10000);
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

    for (auto &pc : point_clouds_container.point_clouds)
    {
        glVertex3f(pc.m_pose(0, 3), pc.m_pose(1, 3), pc.m_pose(2, 3));
    }
    glEnd();

    int i = 0;
    for (auto &pc : point_clouds_container.point_clouds)
    {
        glRasterPos3f(pc.m_pose(0, 3), pc.m_pose(1, 3), pc.m_pose(2, 3) + 0.1);
        glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, (const unsigned char *)std::to_string(i).c_str());
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
        glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, (const unsigned char *)std::to_string(i).c_str());
    }
}

#endif
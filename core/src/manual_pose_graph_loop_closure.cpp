#include <pch/pch.h>

#if WITH_GUI == 1
#include <GL/freeglut.h>
#include <imgui.h>

#include <Core/export_laz.h>
#include <Core/gnss.h>
#include <Core/icp.h>
#include <Core/m_estimators.h>
#include <Core/manual_pose_graph_loop_closure.h>
#include <Core/pair_wise_iterative_closest_point.h>
#include <Core/pfd_wrapper.hpp>
#include <Core/transformations.h>
#include <Core/utils.hpp>

#include <python-scripts/constraints/quaternion_constraint_jacobian.h>
#include <python-scripts/constraints/relative_pose_quaternion_cw_jacobian.h>
#include <python-scripts/constraints/relative_pose_quaternion_wc_jacobian.h>
#include <python-scripts/constraints/relative_pose_rodrigues_wc_jacobian.h>
#include <python-scripts/constraints/relative_pose_tait_bryan_cw_jacobian.h>
#include <python-scripts/constraints/relative_pose_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-feature-metrics/point_to_line_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian.h>

#include <common/include/cauchy.h>

void ManualPoseGraphLoopClosure::icp_all_edges_step(
    PointClouds& point_clouds_container, int num_edge_extended_before, int num_edge_extended_after)
{
    auto& run = icp_all_edges_run;
    // The first step waits for a frame that shows the progress bar, since each step blocks the window.
    if (!run.progress_drawn)
    {
        return;
    }
    if (run.edge < static_cast<int>(edges.size()))
    {
        const float radius = run.radii[run.step];
        std::cout << "ICP all edges: edge " << run.edge + 1 << " of " << edges.size() << ", search radius " << radius << " m" << std::endl;
        if (run_icp(point_clouds_container, run.edge, radius, 30, num_edge_extended_before, num_edge_extended_after))
        {
            run.step++;
        }
        else
        {
            std::cout << "ICP all edges: edge " << run.edge << " failed at search radius " << radius << " m" << std::endl;
            run.failed.push_back(run.edge);
            run.step = run.radii.size(); // an edge stops at its first failed step
        }
        if (run.step >= run.radii.size())
        {
            run.edge++;
            run.step = 0;
        }
    }
    if (run.edge >= static_cast<int>(edges.size()))
    {
        run.running = false;
        icp_failed_edges = run.failed;
        icp_all_edges_count = edges.size();
        std::cout << "ICP all edges: " << edges.size() - run.failed.size() << " of " << edges.size() << " edges refined" << std::endl;
    }
}

void ManualPoseGraphLoopClosure::icp_all_edges_gui()
{
    auto& run = icp_all_edges_run;
    const auto format_duration = [](double seconds)
    {
        const int s = static_cast<int>(seconds + 0.5);
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%d:%02d", s / 60, s % 60);
        return std::string(buf);
    };

    if (run.running)
    {
        const size_t steps_per_edge = run.radii.size();
        const double done = static_cast<double>(run.edge * steps_per_edge + run.step);
        const double total = static_cast<double>(edges.size() * steps_per_edge);
        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - run.start).count();
        char overlay[96];
        std::snprintf(
            overlay,
            sizeof(overlay),
            "edge %d of %zu, radius %.2f m",
            std::min(run.edge + 1, static_cast<int>(edges.size())),
            edges.size(),
            run.radii[std::min(run.step, steps_per_edge - 1)]);
        ImGui::ProgressBar(total > 0.0 ? static_cast<float>(done / total) : 1.0f, ImVec2(350, 0), overlay);
        ImGui::SameLine();
        if (ImGui::Button("Cancel"))
        {
            run.running = false;
            std::cout << "ICP all edges: cancelled after " << run.edge << " of " << edges.size() << " edges" << std::endl;
        }
        ImGui::SameLine();
        if (done > 0.0)
            ImGui::Text(
                "%s elapsed, about %s left", format_duration(elapsed).c_str(), format_duration(elapsed * (total - done) / done).c_str());
        else
            ImGui::Text("%s elapsed", format_duration(elapsed).c_str());
        run.progress_drawn = true;
        return;
    }

    ImGui::BeginDisabled(edges.empty());
    if (ImGui::Button("ICP all edges"))
    {
        run = IcpAllEdgesRun{};
        run.running = true;
        run.radii = icp_search_radii(static_cast<float>(icp_all_edges_start_radius));
        run.start = std::chrono::steady_clock::now();
        icp_all_edges_count = 0;
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip(
            "Refine every edge with ICP, coarse to fine from the start radius.\n"
            "Run before 'Compute Pose Graph SLAM': an edge added without ICP holds the current misalignment.");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ImGuiNumberWidth);
    ImGui::InputDouble("Start radius [m]", &icp_all_edges_start_radius, 0.0, 0.0, "%.2f");
    if (icp_all_edges_start_radius < 0.01)
        icp_all_edges_start_radius = 0.01;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(
            "First ICP search radius; the run continues through 2.0, 1.0, 0.5, 0.25, 0.1 m below it.\n"
            "Start above how far the edges are off: a larger radius converges from farther but takes longer.");

    // Edge indexes shift when edges are added or removed elsewhere (e.g. the BoW window), so the result is only
    // shown while the edge count still matches.
    if (icp_all_edges_count > 0 && icp_all_edges_count == edges.size())
    {
        ImGui::SameLine();
        if (icp_failed_edges.empty())
        {
            ImGui::Text("all %zu edges refined", edges.size());
        }
        else
        {
            std::string failed;
            for (const int e : icp_failed_edges)
                failed += (failed.empty() ? "" : ", ") + std::to_string(e);
            ImGui::TextColored(orangeBorder, "ICP failed on edges: %s", failed.c_str());
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Check these edges: farther off than the start radius, or too little overlap");
        }
    }
}

void ManualPoseGraphLoopClosure::Gui(
    PointClouds& point_clouds_container,
    int& index_loop_closure_source,
    int& index_loop_closure_target,
    float* m_gizmo,
    GNSS& gnss,
    TUM& tum,
    GroundControlPoints& gcps,
    ControlPoints& cps,
    int num_edge_extended_before,
    int num_edge_extended_after)
{
    if (point_clouds_container.point_clouds.size() > 0)
    {
        if (icp_all_edges_run.running)
        {
            icp_all_edges_step(point_clouds_container, num_edge_extended_before, num_edge_extended_after);
        }

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
                icp_all_edges_count = 0;
                icp_all_edges_run.running = false;
            }
            ImGui::SameLine();
            ImGui::Text("(number of active edges: %zu)", edges.size());

            if (!manipulate_active_edge)
            {
                static bool keep_initial_trajectory_curvature = true;

                icp_all_edges_gui();

                ImGui::BeginDisabled(icp_all_edges_run.running);
                if (ImGui::Button("Compute Pose Graph SLAM"))
                {
                    if (keep_initial_trajectory_curvature)
                    {
                        set_initial_poses_as_motion_model(point_clouds_container);
                    }
                    else
                    {
                        set_current_poses_as_motion_model(point_clouds_container);
                    }
                    graph_slam(point_clouds_container, gnss, gcps, cps);
                }
                ImGui::EndDisabled();

                ImGui::SameLine();
                ImGui::Checkbox("Keep initial trajectory curvature", &keep_initial_trajectory_curvature);

                if (gnss.gnss_poses.size() > 0)
                {
                    ImGui::Checkbox("Use GNSS correspondences in Pose Graph SLAM", &use_gnss_correspondences);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "When off, loaded GNSS poses are still shown but do not contribute any observations "
                            "to 'Compute Pose Graph SLAM'");
                }

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

                if (ImGui::Button("Fuse trajectory with TUM (trajectory is rigid)"))
                {
                    FuseTrajectoryWithTUM(point_clouds_container, tum);
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

                if (ImGui::Button("Save source (local)"))
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
                if (ImGui::Button("Save target (local)"))
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
                ImGui::SameLine();
                if (ImGui::Button("Print relative pose in console"))
                {
                    auto& e = edges[index_active_edge];
                    Eigen::Affine3d m = affine_matrix_from_pose_tait_bryan(e.relative_pose_tb);
                    std::cout << m.matrix() << std::endl;
                }
                ImGui::SameLine();
                if (ImGui::Button("Print all relative pose in console"))
                {
                    for (const auto& e : edges)
                    {
                        std::cout << "source: " << point_clouds_container.point_clouds[e.index_to].file_name << std::endl;
                        std::cout << "target: " << point_clouds_container.point_clouds[e.index_from].file_name << std::endl;
                        std::cout << "realtive pose: " << std::endl;
                        Eigen::Affine3d m = affine_matrix_from_pose_tait_bryan(e.relative_pose_tb);
                        std::cout << m.matrix() << std::endl;

                        std::cout << std::endl;
                    }
                }

                //-------
                if (ImGui::Button("Save source (global)"))
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

                        // Eigen::Affine3d m_src_inv = point_clouds_container.point_clouds[e.index_to].m_pose.inverse();
                        // for (auto& p : source)
                        //{
                        //     p = m_src_inv * p;
                        // }

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
                if (ImGui::Button("Save target (global)"))
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

                        ////Eigen::Affine3d m_trg_inv = point_clouds_container.point_clouds[e.index_from].m_pose.inverse();

                        // for (auto& p : target)
                        //{
                        //     p = m_trg_inv * p;
                        // }

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

            // N / P: next / previous active edge, unless a text field is being edited.
            const ImGuiIO& io = ImGui::GetIO();
            if (edge_hotkeys && !io.WantTextInput && !io.KeyCtrl && !io.KeyAlt && !io.KeyShift)
            {
                if (ImGui::IsKeyPressed(ImGuiKey_N) && index_active_edge + 1 < static_cast<int>(edges.size()))
                    index_active_edge++;
                if (ImGui::IsKeyPressed(ImGuiKey_P) && index_active_edge > 0)
                    index_active_edge--;
            }
            const bool scroll_to_active_edge = (prev_index_active_edge != index_active_edge);
            ImGui::TextDisabled("N / P: next / previous edge");

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
                    if (scroll_to_active_edge && index_active_edge == static_cast<int>(i))
                        ImGui::SetScrollHereY(0.5f);

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
                        icp_all_edges_count = 0;
                        icp_all_edges_run.running = false;

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
                // ObservationPicking observation_picking;
                // point_clouds_container.point_clouds.at(i).render(false, observation_picking, 1, 1, false, false, false, 10000, false);
                Eigen::Affine3d m_src = point_clouds_container.point_clouds.at(i).m_pose;

                if (render_source_as_red_target_as_blue)
                {
                    float color[3];
                    color[0] = 1.0;
                    color[1] = 0.0;
                    color[2] = 0.0;
                    point_clouds_container.point_clouds.at(i).render(m_src, 1, 1, color);
                }
                else
                {
                    point_clouds_container.point_clouds.at(i).render(m_src, 1, 1, point_clouds_container.point_clouds.at(i).render_color);
                }
            }
        }

        for (int i = index_loop_closure_target - num_edge_extended_before; i <= index_loop_closure_target + num_edge_extended_after; i++)
        {
            if (i >= 0 && i < point_clouds_container.point_clouds.size() && point_clouds_container.point_clouds.size() > 0)
            {
                // ObservationPicking observation_picking;
                // point_clouds_container.point_clouds.at(i).render(false, observation_picking, 1, 1, false, false, false, 10000, false);
                Eigen::Affine3d m_src = point_clouds_container.point_clouds.at(i).m_pose;

                if (render_source_as_red_target_as_blue)
                {
                    float color[3];
                    color[0] = 0.0;
                    color[1] = 0.0;
                    color[2] = 1.0;
                    point_clouds_container.point_clouds.at(i).render(m_src, 1, 1, color);
                }
                else
                {
                    point_clouds_container.point_clouds.at(i).render(m_src, 1, 1, point_clouds_container.point_clouds.at(i).render_color);
                }
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

                    if (render_source_as_red_target_as_blue)
                    {
                        float color[3];
                        color[0] = 1.0;
                        color[1] = 0.0;
                        color[2] = 0.0;
                        point_clouds_container.point_clouds.at(index_src).render(m_src, 1, 1, color);
                    }
                    else
                    {
                        point_clouds_container.point_clouds.at(index_src).render(
                            m_src, 1, 1, point_clouds_container.point_clouds.at(index_src).render_color);
                    }
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

                    if (render_source_as_red_target_as_blue)
                    {
                        float color[3];
                        color[0] = 0.0;
                        color[1] = 0.0;
                        color[2] = 1.0;
                        point_clouds_container.point_clouds.at(index_trg + i).render(m_trg, 1, 1, color);
                    }
                    else
                    {
                        point_clouds_container.point_clouds.at(index_trg + i)
                            .render(m_trg, 1, 1, point_clouds_container.point_clouds.at(index_trg + i).render_color);
                    }
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

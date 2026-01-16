#include <pch/pch.h>

#include <ground_control_points.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian.h>
#if WITH_GUI == 1
#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <imgui_internal.h>

static constexpr float ImGuiNumberWidth = 120.0f;
static constexpr const char* xText = "Longitudinal (forward/backward)";
static constexpr const char* yText = "Lateral (left/right)";
static constexpr const char* zText = "Vertical (up/down)";

void GroundControlPoints::imgui(PointClouds& point_clouds_container)
{
    if (ImGui::Begin("Ground Control Point", &is_imgui))
    {
        ImGui::Checkbox("Draw uncertainty", &draw_uncertainty);
        ImGui::SetNextItemWidth(ImGuiNumberWidth);
        ImGui::InputDouble("Default lidar height above ground [m]", &default_lidar_height_above_ground, 0.0, 10.0, "%.3f");

        // ImGui::Text("To show Ground Control Point (GCP) candidate please press 'CTRL'");
        ImGui::Text("To pick trajectory node please press 'CTRL + left/middle mouse'");
        ImGui::Text("At least 3 GCPs needed!");

        if (picking_mode_index_to_node_inner != -1 && picking_mode_index_to_node_outer != -1)
        {
            if (ImGui::Button("Add GCP"))
            {
                GroundControlPoint gcp;
                gcp.index_to_node_inner = picking_mode_index_to_node_inner;
                gcp.index_to_node_outer = picking_mode_index_to_node_outer;
                gcp.lidar_height_above_ground = default_lidar_height_above_ground;
                gcp.sigma_x = 0.1;
                gcp.sigma_y = 0.1;
                gcp.sigma_z = 0.1;
                const auto& p = point_clouds_container.point_clouds[gcp.index_to_node_inner]
                                    .local_trajectory[gcp.index_to_node_outer]
                                    .m_pose.translation();
                Eigen::Vector3d position_global = point_clouds_container.point_clouds[gcp.index_to_node_inner].m_pose * p;

                gcp.x = position_global.x();
                gcp.y = position_global.y();
                gcp.z = position_global.z();
                strcpy(gcp.name, "default_name");

                gpcs.push_back(gcp);
            }
            // int picking_mode_index_to_node_inner = -1;
            // int picking_mode_index_to_node_outer = -1;
        }

        ImGui::NewLine();
        ImGui::Separator();
        ImGui::NewLine();

        int remove_gcp_index = -1;
        for (int i = 0; i < gpcs.size(); i++)
        {
            ImGui::Text(("GCP_" + std::to_string(i) + " (" + gpcs[i].name + ")").c_str());
            ImGui::SameLine();
            if (ImGui::Button(("Remove##" + std::to_string(i)).c_str()))
            {
                remove_gcp_index = i;
            }

            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::InputText(("GCP name##" + std::to_string(i)).c_str(), gpcs[i].name, IM_ARRAYSIZE(gpcs[i].name));

            ImGui::Text("sigma [m]:");
            ImGui::InputDouble(("X##" + std::to_string(i)).c_str(), &gpcs[i].sigma_x, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(xText);
            ImGui::SameLine();
            ImGui::InputDouble(("Y##" + std::to_string(i)).c_str(), &gpcs[i].sigma_y, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(yText);
            ImGui::SameLine();
            ImGui::InputDouble(("Z##" + std::to_string(i)).c_str(), &gpcs[i].sigma_z, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(zText);

            ImGui::Text("coordinates [m]:");
            ImGui::InputDouble(("X##c" + std::to_string(i)).c_str(), &gpcs[i].x);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(xText);
            ImGui::SameLine();
            ImGui::InputDouble(("Y##c" + std::to_string(i)).c_str(), &gpcs[i].y);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(yText);
            ImGui::SameLine();
            ImGui::InputDouble(("Z##c" + std::to_string(i)).c_str(), &gpcs[i].z);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(zText);

            ImGui::InputDouble(("lidar_height_above_ground [m]##" + std::to_string(i)).c_str(), &gpcs[i].lidar_height_above_ground);
            ImGui::PopItemWidth();

            ImGui::NewLine();
        }

        if (remove_gcp_index != -1)
        {
            std::vector<GroundControlPoint> new_gcps;
            for (int i = 0; i < gpcs.size(); i++)
            {
                if (i != remove_gcp_index)
                {
                    new_gcps.push_back(gpcs[i]);
                }
            }
            gpcs = new_gcps;
        }

        if (gpcs.size() >= 3)
        {
            if (ImGui::Button("Register session to GCPs"))
            {
                ////////////////////////////////////
                TaitBryanPose pose_s;
                pose_s.px = 0.0;
                pose_s.py = 0.0;
                pose_s.pz = 0.0;
                pose_s.om = 0.0;
                pose_s.fi = 0.0;
                pose_s.ka = 0.0;

                std::vector<Eigen::Triplet<double>> tripletListA;
                std::vector<Eigen::Triplet<double>> tripletListP;
                std::vector<Eigen::Triplet<double>> tripletListB;

                // GCPs
                for (int i = 0; i < gpcs.size(); i++)
                {
                    Eigen::Vector3d p_s = point_clouds_container.point_clouds[gpcs[i].index_to_node_inner].m_pose *
                        point_clouds_container.point_clouds[gpcs[i].index_to_node_inner]
                            .local_trajectory[gpcs[i].index_to_node_outer]
                            .m_pose.translation();

                    Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;

                    point_to_point_source_to_target_tait_bryan_wc_jacobian(
                        jacobian, pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka, p_s.x(), p_s.y(), p_s.z());

                    double delta_x;
                    double delta_y;
                    double delta_z;
                    Eigen::Vector3d p_t(gpcs[i].x, gpcs[i].y, gpcs[i].z + gpcs[i].lidar_height_above_ground);
                    point_to_point_source_to_target_tait_bryan_wc(
                        delta_x,
                        delta_y,
                        delta_z,
                        pose_s.px,
                        pose_s.py,
                        pose_s.pz,
                        pose_s.om,
                        pose_s.fi,
                        pose_s.ka,
                        p_s.x(),
                        p_s.y(),
                        p_s.z(),
                        p_t.x(),
                        p_t.y(),
                        p_t.z());

                    int ir = tripletListB.size();
                    int ic = 0;

                    for (int row = 0; row < 3; row++)
                    {
                        for (int col = 0; col < 6; col++)
                        {
                            if (jacobian(row, col) != 0.0)
                            {
                                tripletListA.emplace_back(ir + row, ic + col, -jacobian(row, col));
                            }
                        }
                    }
                    tripletListP.emplace_back(ir + 0, ir + 0, 1.0 / (gpcs[i].sigma_x * gpcs[i].sigma_x));
                    tripletListP.emplace_back(ir + 1, ir + 1, 1.0 / (gpcs[i].sigma_y * gpcs[i].sigma_y));
                    tripletListP.emplace_back(ir + 2, ir + 2, 1.0 / (gpcs[i].sigma_z * gpcs[i].sigma_z));

                    tripletListB.emplace_back(ir, 0, delta_x);
                    tripletListB.emplace_back(ir + 1, 0, delta_y);
                    tripletListB.emplace_back(ir + 2, 0, delta_z);

                    std::cout << "GCP: delta_x " << delta_x << " delta_y " << delta_y << " delta_z " << delta_z << std::endl;
                }

                Eigen::SparseMatrix<double> matA(tripletListB.size(), 6);
                Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
                Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

                matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
                matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
                matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

                Eigen::SparseMatrix<double> AtPA(6, 6);
                Eigen::SparseMatrix<double> AtPB(6, 1);

                {
                    Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
                    AtPA = (AtP)*matA;
                    AtPB = (AtP)*matB;
                }

                tripletListA.clear();
                tripletListP.clear();
                tripletListB.clear();

                std::cout << "AtPA.size: " << AtPA.size() << std::endl;
                std::cout << "AtPB.size: " << AtPB.size() << std::endl;

                std::cout << "start solving AtPA=AtPB" << std::endl;
                Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);

                std::cout << "x = solver.solve(AtPB)" << std::endl;
                Eigen::SparseMatrix<double> x = solver.solve(AtPB);

                std::vector<double> h_x;

                for (int k = 0; k < x.outerSize(); ++k)
                {
                    for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
                    {
                        h_x.push_back(it.value());
                    }
                }

                std::cout << "h_x.size(): " << h_x.size() << std::endl;

                if (h_x.size() == 6)
                {
                    std::cout << "AtPA=AtPB SOLVED" << std::endl;
                    pose_s.px = h_x[0];
                    pose_s.py = h_x[1];
                    pose_s.pz = h_x[2];
                    pose_s.om = h_x[3];
                    pose_s.fi = h_x[4];
                    pose_s.ka = h_x[5];

                    auto m_pose = affine_matrix_from_pose_tait_bryan(pose_s);

                    for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++)
                    {
                        point_clouds_container.point_clouds[i].m_pose = m_pose * point_clouds_container.point_clouds[i].m_pose;
                        point_clouds_container.point_clouds[i].pose =
                            pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);
                        point_clouds_container.point_clouds[i].gui_translation[0] = point_clouds_container.point_clouds[i].pose.px;
                        point_clouds_container.point_clouds[i].gui_translation[1] = point_clouds_container.point_clouds[i].pose.py;
                        point_clouds_container.point_clouds[i].gui_translation[2] = point_clouds_container.point_clouds[i].pose.pz;
                        point_clouds_container.point_clouds[i].gui_rotation[0] = rad2deg(point_clouds_container.point_clouds[i].pose.om);
                        point_clouds_container.point_clouds[i].gui_rotation[1] = rad2deg(point_clouds_container.point_clouds[i].pose.fi);
                        point_clouds_container.point_clouds[i].gui_rotation[2] = rad2deg(point_clouds_container.point_clouds[i].pose.ka);
                    }
                }
                else
                {
                    std::cout << "AtPA=AtPB FAILED" << std::endl;
                }
            }
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("trajectory is rigid");
            }
        }

        ImGui::End();
    }

    return;
}

void GroundControlPoints::render(const PointClouds& point_clouds_container)
{
    for (int i = 0; i < gpcs.size(); i++)
    {
        const auto& p = point_clouds_container.point_clouds[gpcs[i].index_to_node_inner]
                            .local_trajectory[gpcs[i].index_to_node_outer]
                            .m_pose.translation();
        Eigen::Vector3d c = point_clouds_container.point_clouds[gpcs[i].index_to_node_inner].m_pose * p;
        Eigen::Vector3d g(gpcs[i].x, gpcs[i].y, gpcs[i].z);

        glColor3f(0.7f, 0.3f, 0.5f);
        glBegin(GL_LINES);
        glVertex3f(g.x() - 0.05, g.y(), g.z());
        glVertex3f(g.x() + 0.05, g.y(), g.z());

        glVertex3f(g.x(), g.y() - 0.05, g.z());
        glVertex3f(g.x(), g.y() + 0.05, g.z());

        glVertex3f(g.x() - 0.01, g.y(), g.z() + gpcs[i].lidar_height_above_ground);
        glVertex3f(g.x() + 0.01, g.y(), g.z() + gpcs[i].lidar_height_above_ground);

        glVertex3f(g.x(), g.y() - 0.01, g.z() + gpcs[i].lidar_height_above_ground);
        glVertex3f(g.x(), g.y() + 0.01, g.z() + gpcs[i].lidar_height_above_ground);

        glVertex3f(g.x(), g.y(), g.z());
        glVertex3f(g.x(), g.y(), g.z() + gpcs[i].lidar_height_above_ground);
        glEnd();

        glColor3f(0.0f, 0.3f, 0.6f);
        glBegin(GL_LINES);
        glVertex3f(c.x(), c.y(), c.z());
        glVertex3f(gpcs[i].x, gpcs[i].y, gpcs[i].z + gpcs[i].lidar_height_above_ground);
        glEnd();

        if (draw_uncertainty)
        {
            Eigen::Matrix3d covar;
            covar(0, 0) = gpcs[i].sigma_x * gpcs[i].sigma_x;
            covar(0, 1) = 0;
            covar(0, 2) = 0;

            covar(1, 0) = 0;
            covar(1, 1) = gpcs[i].sigma_y * gpcs[i].sigma_y;
            covar(1, 2) = 0;

            covar(2, 0) = 0;
            covar(2, 1) = 0;
            covar(2, 2) = gpcs[i].sigma_z * gpcs[i].sigma_z;

            Eigen::Vector3d gcp(gpcs[i].x, gpcs[i].y, gpcs[i].z + gpcs[i].lidar_height_above_ground);
            draw_ellipse(covar, gcp, Eigen::Vector3f(0.5, 0.5, 0.5), 1.0f);
        }

        glColor3f(0.7f, 0.3f, 0.5f);
        glRasterPos3f(gpcs[i].x, gpcs[i].y, gpcs[i].z + gpcs[i].lidar_height_above_ground);
        glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_10, (const unsigned char*)(("GCP_" + std::to_string(i) + ": LiDAR center").c_str()));

        glRasterPos3f(gpcs[i].x, gpcs[i].y, gpcs[i].z);
        glutBitmapString(
            GLUT_BITMAP_TIMES_ROMAN_10, (const unsigned char*)(("GCP_" + std::to_string(i) + ": 'plane on the ground'").c_str()));

        glColor3f(0, 0, 0);
        glRasterPos3f(gpcs[i].x, gpcs[i].y, gpcs[i].z + gpcs[i].lidar_height_above_ground + 0.1);
        glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, (const unsigned char*)gpcs[i].name);

        glRasterPos3f(c.x(), c.y(), c.z());
        glutBitmapString(
            GLUT_BITMAP_TIMES_ROMAN_10, (const unsigned char*)(("GCP_" + std::to_string(i) + ": assigned trajectory node").c_str()));
    }

    return;
}

void GroundControlPoints::draw_ellipse(const Eigen::Matrix3d& covar, Eigen::Vector3d& mean, Eigen::Vector3f color, float nstd)
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
            double u = i * 2 * pi; // 0     to  2pi
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
#endif

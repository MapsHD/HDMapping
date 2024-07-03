#include <ground_control_points.h>

#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <imgui_internal.h>

void GroundControlPoints::imgui(const PointClouds &point_clouds_container)
{
    if (ImGui::Begin("Ground Control Point"))
    {
        ImGui::Checkbox("draw_uncertainty", &draw_uncertainty);
        ImGui::InputDouble("default_lidar_height_above_ground", &default_lidar_height_above_ground, 0.0, 10.0);
        ImGui::Checkbox("picking_trajectory_node_and_adding_GCP_mode", &picking_mode);

        if (picking_mode)
        {

            // ImGui::SameLine();
            ImGui::Text("To pick trajectory node please press 'ctrl' + 'middle mouse'");
            ImGui::Text("To show Ground Control Point candidate please press 'ctrl'");

            if (picking_mode_index_to_node_inner != -1 && picking_mode_index_to_node_outer != -1)
            {
                if (ImGui::Button("Add Ground Control Point"))
                {
                    GroundControlPoint gcp;
                    gcp.index_to_node_inner = picking_mode_index_to_node_inner;
                    gcp.index_to_node_outer = picking_mode_index_to_node_outer;
                    gcp.lidar_height_above_ground = default_lidar_height_above_ground;
                    gcp.sigma_x = 0.1;
                    gcp.sigma_y = 0.1;
                    gcp.sigma_z = 0.1;
                    const auto &p = point_clouds_container.point_clouds[gcp.index_to_node_inner].local_trajectory[gcp.index_to_node_outer].m_pose.translation();
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
        }

        ImGui::Text("-------------------------------------");
        for (int i = 0; i < gpcs.size(); i++)
        {
            ImGui::Text((std::string("gcp_") + std::to_string(i) + "[" + gpcs[i].name + "]").c_str());
            std::string text = "[" + std::to_string(i) + "]_" + gpcs[i].name;

            ImGui::InputText(text.c_str(), gpcs[i].name, IM_ARRAYSIZE(gpcs[i].name));

            text = "[" + std::to_string(i) + "]_sigma_x";
            ImGui::InputDouble(text.c_str(), &gpcs[i].sigma_x);
            text = "[" + std::to_string(i) + "]_sigma_y";
            ImGui::InputDouble(text.c_str(), &gpcs[i].sigma_y);
            text = "[" + std::to_string(i) + "]_sigma_z";
            ImGui::InputDouble(text.c_str(), &gpcs[i].sigma_z);
            text = "[" + std::to_string(i) + "]_x";
            ImGui::InputDouble(text.c_str(), &gpcs[i].x);
            text = "[" + std::to_string(i) + "]_y";
            ImGui::InputDouble(text.c_str(), &gpcs[i].y);
            text = "[" + std::to_string(i) + "]_z";
            ImGui::InputDouble(text.c_str(), &gpcs[i].z);
            text = "[" + std::to_string(i) + "]_lidar_height_above_ground";
            ImGui::InputDouble(text.c_str(), &gpcs[i].lidar_height_above_ground);
        }

        ImGui::End();
    }

    return;
}

void GroundControlPoints::render(const PointClouds &point_clouds_container)
{
    for (int i = 0; i < gpcs.size(); i++)
    {
        const auto &p = point_clouds_container.point_clouds[gpcs[i].index_to_node_inner].local_trajectory[gpcs[i].index_to_node_outer].m_pose.translation();
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

        glColor3f(0,0,0);
        glRasterPos3f(gpcs[i].x, gpcs[i].y, gpcs[i].z + gpcs[i].lidar_height_above_ground + 0.1);
        glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, (const unsigned char *)gpcs[i].name);

        glColor3f(0, 0, 0);
        glRasterPos3f(gpcs[i].x, gpcs[i].y, gpcs[i].z + gpcs[i].lidar_height_above_ground);
        glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_10, (const unsigned char *)("LiDAR center"));

        glColor3f(0, 0, 0);
        glRasterPos3f(gpcs[i].x, gpcs[i].y, gpcs[i].z);
        glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_10, (const unsigned char *)("GCP 'plane on the ground'"));

        glColor3f(0, 0, 0);
        glRasterPos3f(c.x(), c.y(), c.z());
        glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_10, (const unsigned char *)("trajectory node assigned to GCP"));
    }

    return;
}

void GroundControlPoints::draw_ellipse(const Eigen::Matrix3d &covar, Eigen::Vector3d &mean, Eigen::Vector3f color, float nstd)
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
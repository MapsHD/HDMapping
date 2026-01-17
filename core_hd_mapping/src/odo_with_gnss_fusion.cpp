#include <pch/pch.h>

#include <odo_with_gnss_fusion.h>

#include <GL/freeglut.h>
#include <portable-file-dialogs.h>

#include <m_estimators.h>
#include <transformations.h>

#include <pfd_wrapper.hpp>

#include <python-scripts/constraints/relative_pose_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian.h>

void OdoWithGnssFusion::imgui(CommonData& common_data)
{
    ImGui::Begin("odometry with gnss fusion");
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

    ImGui::Checkbox("set_initial_offset_from_trajectory", &set_initial_offset_from_trajectory);

    if (ImGui::Button("load GNSS trajectory"))
    {
        std::string input_file_name = "";
        input_file_name = mandeye::fd::OpenFileDialogOneFile("Load GNSS trajectory", {});

        if (input_file_name.size() > 0)
        {
            gnss_trajectory = load_trajectory(input_file_name);
            for (size_t i = 0; i < gnss_trajectory.size(); i++)
            {
                double tmp = gnss_trajectory[i].m_pose(1, 3);
                gnss_trajectory[i].m_pose(1, 3) = gnss_trajectory[i].m_pose(0, 3);
                gnss_trajectory[i].m_pose(0, 3) = tmp;
            }
            gnss_trajectory_shifted = gnss_trajectory;

            if (set_initial_offset_from_trajectory)
            {
                float shift_x = -gnss_trajectory[gnss_trajectory.size() / 2].m_pose(0, 3);
                float shift_y = -gnss_trajectory[gnss_trajectory.size() / 2].m_pose(1, 3);
                update_shift(shift_x, shift_y, common_data);
                set_initial_offset_from_trajectory = false;
            }
            else
            {
                update_shift(common_data.shift_x, common_data.shift_y, common_data);
            }
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("load LiDAR odometry trajectory"))
    {
        std::string input_file_name = "";
        input_file_name = mandeye::fd::OpenFileDialogOneFile("Load LiDAR odometry trajectory", {});
        std::cout << "trajectory file: '" << input_file_name << "'" << std::endl;

        if (input_file_name.size() > 0)
        {
            fused_trajectory = load_trajectory(input_file_name);
            lidar_odometry_trajectory_initial = fused_trajectory;
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("save fused trajectory"))
    {
        std::string output_file_name = "";
        output_file_name = mandeye::fd::SaveFileDialog("Save fused trajectory", {});
        if (output_file_name.size() > 0)
        {
            if (save_trajectory(output_file_name))
            {
                std::cout << "trajectory saved to file: " << output_file_name << std::endl;
            }
            else
            {
                std::cout << "problem with saving file: " << output_file_name << std::endl;
            }

            auto new_path = std::filesystem::path(output_file_name).parent_path();
            auto file_name = std::filesystem::path(output_file_name).stem();

            new_path /= file_name;
            new_path += "_motion_model.csv";

            if (save_trajectory(new_path.string()))
            {
                std::cout << "motion model saved to file: " << new_path.string() << std::endl;
            }
            else
            {
                std::cout << "problem with saving file: " << new_path.string() << std::endl;
            }
        }
    }

    if (ImGui::Button("find correspondences"))
    {
        correspondences_from_lo_to_shifted_gnss = find_correspondences_from_lo_to_shifted_gnss();
    }

    ImGui::SameLine();
    if (ImGui::Button("rigid registration"))
    {
        correspondences_from_lo_to_shifted_gnss = find_correspondences_from_lo_to_shifted_gnss();
        rigid_registration(false);
    }
    ImGui::SameLine();
    if (ImGui::Button("rigid registration x6"))
    {
        for (int i = 0; i < 6; i++)
        {
            correspondences_from_lo_to_shifted_gnss = find_correspondences_from_lo_to_shifted_gnss();
            rigid_registration(false);
        }
    }
    ImGui::SameLine();
    ImGui::Checkbox("show_correspondences_rigid_registration", &show_correspondences_rigid_registration);

    if (ImGui::Button("find between nodes"))
    {
        between_nodes = find_between_nodes();
    }

    ImGui::SameLine();
    if (ImGui::Button("semi rigid registration"))
    {
        between_nodes = find_between_nodes();
        semi_rigid_registration();
    }
    ImGui::SameLine();
    if (ImGui::Button("semi rigid registration x30"))
    {
        for (int i = 0; i < 30; i++)
        {
            between_nodes = find_between_nodes();
            semi_rigid_registration();
        }
    }
    ImGui::SameLine();
    ImGui::Checkbox("show_correspondences_semi_rigid_registration", &show_correspondences_semi_rigid_registration);

    ImGui::End();
}

void OdoWithGnssFusion::render()
{
    glColor3d(1, 0, 0);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < gnss_trajectory_shifted.size(); i++)
    {
        glVertex3f(
            gnss_trajectory_shifted[i].m_pose(0, 3), gnss_trajectory_shifted[i].m_pose(1, 3), gnss_trajectory_shifted[i].m_pose(2, 3));
    }
    glEnd();

    glColor3d(0, 1, 0);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < fused_trajectory.size(); i++)
    {
        glVertex3f(fused_trajectory[i].m_pose(0, 3), fused_trajectory[i].m_pose(1, 3), fused_trajectory[i].m_pose(2, 3));
    }
    glEnd();

    if (show_correspondences_rigid_registration)
    {
        glColor3f(0.0f, 0.0f, 1.0f);
        glBegin(GL_LINES);
        for (auto& c : correspondences_from_lo_to_shifted_gnss)
        {
            glVertex3f(
                fused_trajectory[c.first].m_pose.translation().x(),
                fused_trajectory[c.first].m_pose.translation().y(),
                fused_trajectory[c.first].m_pose.translation().z());
            glVertex3f(
                gnss_trajectory_shifted[c.second].m_pose.translation().x(),
                gnss_trajectory_shifted[c.second].m_pose.translation().y(),
                gnss_trajectory_shifted[c.second].m_pose.translation().z());
        }
        glEnd();
    }

    if (show_correspondences_semi_rigid_registration)
    {
        for (const auto& n : between_nodes)
        {
            glColor3f(n.color_x, n.color_y, n.color_z);
            glPointSize(10);
            glBegin(GL_POINTS);
            glVertex3f(n.node_outer.m_pose.translation().x(), n.node_outer.m_pose.translation().y(), n.node_outer.m_pose.translation().z());
            glEnd();
            glPointSize(1);

            for (const auto& ni : n.nodes_between)
            {
                Eigen::Vector3d v = n.node_outer.m_pose * ni.m_pose.translation();
                glPointSize(2);
                glBegin(GL_POINTS);
                glVertex3f(v.x(), v.y(), v.z());
                glEnd();
                glPointSize(1);

                if (ni.index_to_gnss != -1)
                {
                    glBegin(GL_LINES);
                    glVertex3f(v.x(), v.y(), v.z());
                    glVertex3f(
                        gnss_trajectory_shifted[ni.index_to_gnss].m_pose.translation().x(),
                        gnss_trajectory_shifted[ni.index_to_gnss].m_pose.translation().y(),
                        gnss_trajectory_shifted[ni.index_to_gnss].m_pose.translation().z());
                    glEnd();
                }
            }
        }
    }
}

class WordDelimitedBySemicolons : public std::string
{
};

inline std::istream& operator>>(std::istream& is, WordDelimitedBySemicolons& output)
{
    std::getline(is, output, ';');
    return is;
}

std::vector<Node> OdoWithGnssFusion::load_trajectory(const std::string& file_name)
{
    std::vector<Node> trajectory;

    std::ifstream f;
    f.open(file_name.c_str());
    if (f.good())
    {
        std::cout << std::setprecision(20);
        std::cout << "Loading trj from file: " << file_name << std::endl;

        std::string s;
        getline(f, s); // read header
        std::cout << s << "\n";

        while (f.good())
        {
            getline(f, s);
            std::istringstream iss(s);
            std::vector<std::string> results(
                (std::istream_iterator<WordDelimitedBySemicolons>(iss)), std::istream_iterator<WordDelimitedBySemicolons>());
            if (results.size() != 13)
            {
                std::cout << "results.size() != 13 it is " << results.size() << std::endl;
                break;
            }
            Node n;
            // std::cout<<"size: "<<results.size()<<'\n';
            n.timestamp = std::stod(results[0]);
            n.m_pose(0, 3) = std::stod(results[1]);
            n.m_pose(1, 3) = std::stod(results[2]);
            n.m_pose(2, 3) = std::stod(results[3]);
            n.m_pose(0, 0) = std::stod(results[4]);
            n.m_pose(0, 1) = std::stod(results[5]);
            n.m_pose(0, 2) = std::stod(results[6]);
            n.m_pose(1, 0) = std::stod(results[7]);
            n.m_pose(1, 1) = std::stod(results[8]);
            n.m_pose(1, 2) = std::stod(results[9]);
            n.m_pose(2, 0) = std::stod(results[10]);
            n.m_pose(2, 1) = std::stod(results[11]);
            n.m_pose(2, 2) = std::stod(results[12]);
            trajectory.push_back(n);
        }
        f.close();
        return trajectory;
    }
    else
    {
        std::cout << "Can't read trajectory from file " << file_name << std::endl;
        fflush(stdout);
        return trajectory;
    }
}

bool OdoWithGnssFusion::save_trajectory(const std::string& file_name)
{
    std::ofstream outfile;
    outfile << std::setprecision(20);
    outfile.open(file_name);
    outfile << "timestamp;t0;t1;t2;r00;r01;r02;r10;r11;r12;r20;r21;r22" << std::endl;

    for (size_t i = 0; i < this->fused_trajectory.size(); i++)
    {
        outfile << this->fused_trajectory[i].timestamp << ";" << this->fused_trajectory[i].m_pose(0, 3) << ";"
                << this->fused_trajectory[i].m_pose(1, 3) << ";" << this->fused_trajectory[i].m_pose(2, 3) << ";"
                << this->fused_trajectory[i].m_pose(0, 0) << ";" << this->fused_trajectory[i].m_pose(0, 1) << ";"
                << this->fused_trajectory[i].m_pose(0, 2) << ";" << this->fused_trajectory[i].m_pose(1, 0) << ";"
                << this->fused_trajectory[i].m_pose(1, 1) << ";" << this->fused_trajectory[i].m_pose(1, 2) << ";"
                << this->fused_trajectory[i].m_pose(2, 0) << ";" << this->fused_trajectory[i].m_pose(2, 1) << ";"
                << this->fused_trajectory[i].m_pose(2, 2) << std::endl;
    }

    outfile.close();
    return true;
    return true;
}

void OdoWithGnssFusion::update_shift(const float shift_x, float const shift_y, CommonData& common_data)
{
    common_data.shift_x = shift_x;
    common_data.shift_y = shift_y;

    for (size_t i = 0; i < gnss_trajectory.size(); i++)
    {
        gnss_trajectory_shifted[i].m_pose(0, 3) = gnss_trajectory[i].m_pose(0, 3) + shift_x;
        gnss_trajectory_shifted[i].m_pose(1, 3) = gnss_trajectory[i].m_pose(1, 3) + shift_y;
    }
}

std::vector<std::pair<int, int>> OdoWithGnssFusion::find_correspondences_from_lo_to_shifted_gnss()
{
    std::vector<std::pair<int, int>> correspondences;

    double dist_along = 0.0f;

    for (size_t i = 1; i < fused_trajectory.size(); i++)
    {
        dist_along += (fused_trajectory[i].m_pose.translation() - fused_trajectory[i - 1].m_pose.translation()).norm();
        if (dist_along > 5)
        {
            auto it = std::lower_bound(
                gnss_trajectory_shifted.begin(),
                gnss_trajectory_shifted.end(),
                fused_trajectory[i],
                [](Node lhs, Node rhs) -> bool
                {
                    return lhs.timestamp < rhs.timestamp;
                });

            if (fabs(it->timestamp - fused_trajectory[i].timestamp) < 0.01)
            {
                std::pair<int, int> correspondence;
                correspondence.first = i;
                correspondence.second = it - gnss_trajectory_shifted.begin();
                correspondences.emplace_back(correspondence);
                dist_along = 0.0;
            }
        }
    }
    return correspondences;
}

bool OdoWithGnssFusion::rigid_registration(bool adaptive_robust_kernel)
{
    // std::cout << "rigid_registration()" << std::endl;

    // barron
    double min_sum_x = 1000000000000; // std::numeric_limits<double>::max();
    double min_sum_y = 1000000000000; // std::numeric_limits<double>::max();
    double min_sum_z = 1000000000000; // std::numeric_limits<double>::max();

    double barron_alpha_x = 2.;
    double barron_alpha_y = 2.;
    double barron_alpha_z = 2.;

    Eigen::Affine3d m_pose_source = Eigen::Affine3d::Identity();
    TaitBryanPose pose_source = pose_tait_bryan_from_affine_matrix(m_pose_source);

    float scale_factor_x = 1;
    float scale_factor_y = 1;
    float scale_factor_z = 1;

    double barron_c = 1.0;
    if (adaptive_robust_kernel)
    {
        for (double alpha = -10; alpha <= 2; alpha += 0.1)
        {
            double Z_tilde = get_approximate_partition_function(-10, 10, alpha, barron_c, 100);

            double sum_x = 0;
            double sum_y = 0;
            double sum_z = 0;

            for (auto& c : correspondences_from_lo_to_shifted_gnss)
            {
                Eigen::Vector3d p_s(
                    fused_trajectory[c.first].m_pose.translation().x(),
                    fused_trajectory[c.first].m_pose.translation().y(),
                    fused_trajectory[c.first].m_pose.translation().z());
                Eigen::Vector3d p_t(
                    gnss_trajectory_shifted[c.second].m_pose.translation().x(),
                    gnss_trajectory_shifted[c.second].m_pose.translation().y(),
                    gnss_trajectory_shifted[c.second].m_pose.translation().z());
                double delta_x = p_t.x() - p_s.x();
                double delta_y = p_t.y() - p_s.y();
                double delta_z = p_t.z() - p_s.z();

                if (!(delta_x == delta_x))
                {
                    continue;
                }
                if (!(delta_y == delta_y))
                {
                    continue;
                }
                if (!(delta_z == delta_z))
                {
                    continue;
                }

                delta_x *= scale_factor_x;
                delta_y *= scale_factor_y;
                delta_z *= scale_factor_z;

                sum_x += get_truncated_robust_kernel(delta_x, alpha, barron_c, Z_tilde);
                sum_y += get_truncated_robust_kernel(delta_y, alpha, barron_c, Z_tilde);
                sum_z += get_truncated_robust_kernel(delta_z, alpha, barron_c, Z_tilde);

                // std::cout << "sum_x: " << sum_x << " sum_y: " << sum_y << " sum_z: " << sum_z << std::endl;

                if (sum_x < min_sum_x)
                {
                    min_sum_x = sum_x;
                    barron_alpha_x = alpha;
                }
                if (sum_y < min_sum_y)
                {
                    min_sum_y = sum_y;
                    barron_alpha_y = alpha;
                }
                if (sum_z < min_sum_z)
                {
                    min_sum_z = sum_z;
                    barron_alpha_z = alpha;
                }
            }
        }

        std::cout << "barron_alpha_x: " << barron_alpha_x << " barron_alpha_y: " << barron_alpha_y << " barron_alpha_z: " << barron_alpha_z
                  << std::endl;
    }
    std::vector<Eigen::Triplet<double>> tripletListA;
    std::vector<Eigen::Triplet<double>> tripletListP;
    std::vector<Eigen::Triplet<double>> tripletListB;

    for (auto& c : correspondences_from_lo_to_shifted_gnss)
    {
        Eigen::Vector3d p_s(
            fused_trajectory[c.first].m_pose.translation().x(),
            fused_trajectory[c.first].m_pose.translation().y(),
            fused_trajectory[c.first].m_pose.translation().z());
        Eigen::Vector3d p_t(
            gnss_trajectory_shifted[c.second].m_pose.translation().x(),
            gnss_trajectory_shifted[c.second].m_pose.translation().y(),
            gnss_trajectory_shifted[c.second].m_pose.translation().z());

        double delta_x;
        double delta_y;
        double delta_z;
        point_to_point_source_to_target_tait_bryan_wc(
            delta_x,
            delta_y,
            delta_z,
            pose_source.px,
            pose_source.py,
            pose_source.pz,
            pose_source.om,
            pose_source.fi,
            pose_source.ka,
            p_s.x(),
            p_s.y(),
            p_s.z(),
            p_t.x(),
            p_t.y(),
            p_t.z());

        Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
        point_to_point_source_to_target_tait_bryan_wc_jacobian(
            jacobian,
            pose_source.px,
            pose_source.py,
            pose_source.pz,
            pose_source.om,
            pose_source.fi,
            pose_source.ka,
            p_s.x(),
            p_s.y(),
            p_s.z());

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
        if (adaptive_robust_kernel)
        {
            tripletListP.emplace_back(ir, ir, get_barron_w(delta_x * scale_factor_x, barron_alpha_x, barron_c));
            tripletListP.emplace_back(ir + 1, ir + 1, get_barron_w(delta_y * scale_factor_y, barron_alpha_y, barron_c));
            tripletListP.emplace_back(ir + 2, ir + 2, get_barron_w(delta_z * scale_factor_z, barron_alpha_z, barron_c));
        }
        else
        {
            tripletListP.emplace_back(ir, ir, 1);
            tripletListP.emplace_back(ir + 1, ir + 1, 1);
            tripletListP.emplace_back(ir + 2, ir + 2, 1);
        }

        tripletListB.emplace_back(ir, 0, delta_x);
        tripletListB.emplace_back(ir + 1, 0, delta_y);
        tripletListB.emplace_back(ir + 2, 0, delta_z);
    }

    Eigen::SparseMatrix<double> matA(tripletListB.size(), 6);
    Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
    Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

    matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
    matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
    matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

    Eigen::SparseMatrix<double> AtPA(6, 6);
    Eigen::SparseMatrix<double> AtPB(6, 1);

    Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
    AtPA = AtP * matA;
    AtPB = AtP * matB;

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

    if (h_x.size() == 6)
    {
        std::cout << "ICP solution" << std::endl;
        std::cout << "x,y,z,om,fi,ka" << std::endl;
        std::cout << h_x[0] << "," << h_x[1] << "," << h_x[2] << "," << h_x[3] << "," << h_x[4] << "," << h_x[5] << std::endl;

        TaitBryanPose pose_result;
        pose_result.px = h_x[0];
        pose_result.py = h_x[1];
        pose_result.pz = h_x[2];
        pose_result.om = h_x[3];
        pose_result.fi = h_x[4];
        pose_result.ka = h_x[5];

        Eigen::Affine3d m_res = affine_matrix_from_pose_tait_bryan(pose_result);

        for (auto& c : fused_trajectory)
        {
            c.m_pose = m_res * c.m_pose;
        }
    }
    else
    {
        std::cout << "AtPA=AtPB FAILED" << std::endl;
        return false;
    }
    return true;
}

std::vector<BetweenNode> OdoWithGnssFusion::find_between_nodes()
{
    std::vector<BetweenNode> between_nodes;
    double dist_along = 0.0f;
    double dist_along_gnss = 0.0f;

    BetweenNode node_outer;
    node_outer.node_outer.index_to_lidar_odometry_odo = 0;
    node_outer.node_outer.m_pose = fused_trajectory[0].m_pose;
    node_outer.node_outer.timestamp = fused_trajectory[0].timestamp;

    Node node_inner;
    // node_inner.index_to_lidar_odometry_odo = 0;
    // node_inner.m_pose = lidar_odometry_trajectory[0].m_pose;
    // node_inner.timestamp = lidar_odometry_trajectory[0].timestamp;
    // node_outer.nodes_between.push_back(node_inner);

    for (size_t i = 1; i < fused_trajectory.size(); i++)
    {
        node_inner.index_to_lidar_odometry_odo = i;
        node_inner.index_to_gnss = -1;
        node_inner.m_pose = fused_trajectory[i].m_pose;
        node_inner.timestamp = fused_trajectory[i].timestamp;

        double dist_increment = (fused_trajectory[i].m_pose.translation() - fused_trajectory[i - 1].m_pose.translation()).norm();
        dist_along += dist_increment;
        dist_along_gnss += dist_increment;

        // std::cout << "dist_increment " << dist_increment << std::endl;

        if (dist_along_gnss > 10)
        {
            auto it = std::lower_bound(
                gnss_trajectory_shifted.begin(),
                gnss_trajectory_shifted.end(),
                fused_trajectory[i],
                [](Node lhs, Node rhs) -> bool
                {
                    return lhs.timestamp < rhs.timestamp;
                });
            if (fabs(it->timestamp - fused_trajectory[i].timestamp) < 0.01)
            {
                int index_to_gnss = it - gnss_trajectory_shifted.begin();
                int res_index = index_to_gnss;
                double dist_min = 1000000.0;
                for (int index = index_to_gnss - 100; index < index_to_gnss + 100; index++)
                {
                    if (index >= 0 && index < gnss_trajectory_shifted.size())
                    {
                        double distance = sqrt(
                            (fused_trajectory[i].m_pose(0, 3) - gnss_trajectory_shifted[index].m_pose(0, 3)) *
                                (fused_trajectory[i].m_pose(0, 3) - gnss_trajectory_shifted[index].m_pose(0, 3)) +
                            (fused_trajectory[i].m_pose(1, 3) - gnss_trajectory_shifted[index].m_pose(1, 3)) *
                                (fused_trajectory[i].m_pose(1, 3) - gnss_trajectory_shifted[index].m_pose(1, 3)));

                        if (distance < dist_min)
                        {
                            dist_min = distance;
                            res_index = index;
                        }
                    }
                }
                node_inner.index_to_gnss = res_index;
                dist_along_gnss = 0.0;
                // node_inner.index_to_gnss = it - gnss_trajectory_shifted.begin();
                // dist_along_gnss = 0.0;
            }
        }

        node_outer.nodes_between.push_back(node_inner);

        if ((dist_along > 100 || i == fused_trajectory.size() - 1))
        {
            for (auto& n : node_outer.nodes_between)
            {
                n.m_pose = node_outer.node_outer.m_pose.inverse() * n.m_pose;
            }
            node_outer.color_x = float(rand() % 255) / 256.0;
            node_outer.color_y = float(rand() % 255) / 256.0;
            node_outer.color_z = float(rand() % 255) / 256.0;

            between_nodes.push_back(node_outer);

            node_outer.nodes_between.clear();
            node_outer.node_outer.index_to_lidar_odometry_odo = i;
            node_outer.node_outer.m_pose = fused_trajectory[i].m_pose;
            node_outer.node_outer.timestamp = fused_trajectory[i].timestamp;

            dist_along = 0.0;
        }
    }

    /*for (size_t i = 0; i < between_nodes.size(); i++) {
            std::cout << "-------------------------" << std::endl;
            std::cout << between_nodes[i].node_outer.index_to_lidar_odometry_odo << std::endl;

            for (size_t j = 0; j < between_nodes[i].nodes_between.size(); j++) {
                    std::cout << between_nodes[i].nodes_between[j].index_to_lidar_odometry_odo << " ";
            }
            std::cout << std::endl;
    }*/

    return between_nodes;
}

bool OdoWithGnssFusion::semi_rigid_registration()
{
    std::vector<std::pair<int, int>> odo_edges;
    for (size_t i = 1; i < between_nodes.size(); i++)
    {
        odo_edges.emplace_back(i - 1, i);
    }

    std::vector<Eigen::Triplet<double>> tripletListA;
    std::vector<Eigen::Triplet<double>> tripletListP;
    std::vector<Eigen::Triplet<double>> tripletListB;

    std::vector<TaitBryanPose> poses;
    std::vector<TaitBryanPose> poses_desired;

    for (size_t i = 0; i < between_nodes.size(); i++)
    {
        poses.push_back(pose_tait_bryan_from_affine_matrix(between_nodes[i].node_outer.m_pose));
    }
    // poses_desired = poses;

    for (size_t i = 0; i < between_nodes.size(); i++)
    {
        // poses.push_back(pose_tait_bryan_from_affine_matrix(between_nodes[i].node_outer.m_pose));
        poses_desired.push_back(pose_tait_bryan_from_affine_matrix(
            lidar_odometry_trajectory_initial[between_nodes[i].node_outer.index_to_lidar_odometry_odo].m_pose));
    }

    // poses_desired = poses;

    /*for (auto& p : poses) {
            p.px += (((rand() % 1000000) / 1000000.0f) - 0.5) * 2.0 * 0.001;
            p.py += (((rand() % 1000000) / 1000000.0f) - 0.5) * 2.0 * 0.001;
            p.pz += (((rand() % 1000000) / 1000000.0f) - 0.5) * 2.0 * 0.001;

            p.om += (((rand() % 1000000) / 1000000.0f) - 0.5) * 2.0 * 0.000001;
            p.fi += (((rand() % 1000000) / 1000000.0f) - 0.5) * 2.0 * 0.000001;
            p.ka += (((rand() % 1000000) / 1000000.0f) - 0.5) * 2.0 * 0.000001;
    }*/

    for (size_t i = 0; i < odo_edges.size(); i++)
    {
        Eigen::Matrix<double, 6, 1> relative_pose_measurement_odo;
        relative_pose_tait_bryan_wc_case1(
            relative_pose_measurement_odo,
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
            normalize_angle(relative_pose_measurement_odo(3, 0)),
            normalize_angle(relative_pose_measurement_odo(4, 0)),
            normalize_angle(relative_pose_measurement_odo(5, 0)));

        Eigen::Matrix<double, 6, 12, Eigen::RowMajor> jacobian;
        relative_pose_obs_eq_tait_bryan_wc_case1_jacobian(
            jacobian,
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
        tripletListB.emplace_back(ir + 3, 0, normalize_angle(delta(3, 0)));
        tripletListB.emplace_back(ir + 4, 0, normalize_angle(delta(4, 0)));
        tripletListB.emplace_back(ir + 5, 0, normalize_angle(delta(5, 0)));

        tripletListP.emplace_back(ir, ir, 10000);
        tripletListP.emplace_back(ir + 1, ir + 1, 10000);
        tripletListP.emplace_back(ir + 2, ir + 2, 10000);
        tripletListP.emplace_back(ir + 3, ir + 3, 1000000);
        tripletListP.emplace_back(ir + 4, ir + 4, 1000000);
        tripletListP.emplace_back(ir + 5, ir + 5, 1000000);
    }

    // gnss correspondences
    for (size_t i = 0; i < between_nodes.size(); i++)
    {
        TaitBryanPose pose_source = pose_tait_bryan_from_affine_matrix(between_nodes[i].node_outer.m_pose);

        for (size_t j = 0; j < between_nodes[i].nodes_between.size(); j++)
        {
            if (between_nodes[i].nodes_between[j].index_to_gnss != -1)
            {
                Eigen::Vector3d p_s(
                    between_nodes[i].nodes_between[j].m_pose.translation().x(),
                    between_nodes[i].nodes_between[j].m_pose.translation().y(),
                    between_nodes[i].nodes_between[j].m_pose.translation().z());
                Eigen::Vector3d p_t(
                    gnss_trajectory_shifted[between_nodes[i].nodes_between[j].index_to_gnss].m_pose.translation().x(),
                    gnss_trajectory_shifted[between_nodes[i].nodes_between[j].index_to_gnss].m_pose.translation().y(),
                    gnss_trajectory_shifted[between_nodes[i].nodes_between[j].index_to_gnss].m_pose.translation().z());

                double delta_x;
                double delta_y;
                double delta_z;
                point_to_point_source_to_target_tait_bryan_wc(
                    delta_x,
                    delta_y,
                    delta_z,
                    pose_source.px,
                    pose_source.py,
                    pose_source.pz,
                    pose_source.om,
                    pose_source.fi,
                    pose_source.ka,
                    p_s.x(),
                    p_s.y(),
                    p_s.z(),
                    p_t.x(),
                    p_t.y(),
                    p_t.z());

                if (sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z) > 100.0)
                    continue;

                Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
                point_to_point_source_to_target_tait_bryan_wc_jacobian(
                    jacobian,
                    pose_source.px,
                    pose_source.py,
                    pose_source.pz,
                    pose_source.om,
                    pose_source.fi,
                    pose_source.ka,
                    p_s.x(),
                    p_s.y(),
                    p_s.z());

                int ir = tripletListB.size();
                int ic = i * 6;
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

                tripletListP.emplace_back(ir, ir, get_cauchy_w(delta_x, 10));
                tripletListP.emplace_back(ir + 1, ir + 1, get_cauchy_w(delta_y, 10));
                tripletListP.emplace_back(ir + 2, ir + 2, get_cauchy_w(delta_z, 10));

                // tripletListP.emplace_back(ir    ,     ir, 1);
                // tripletListP.emplace_back(ir + 1, ir + 1, 1);
                // tripletListP.emplace_back(ir + 2, ir + 2, 1);

                tripletListB.emplace_back(ir, 0, delta_x);
                tripletListB.emplace_back(ir + 1, 0, delta_y);
                tripletListB.emplace_back(ir + 2, 0, delta_z);
            }
        }
    }

    int ir = tripletListB.size();
    tripletListA.emplace_back(ir, 0, 1);
    tripletListA.emplace_back(ir + 1, 1, 1);
    tripletListA.emplace_back(ir + 2, 2, 1);
    tripletListA.emplace_back(ir + 3, 3, 1);
    tripletListA.emplace_back(ir + 4, 4, 1);
    tripletListA.emplace_back(ir + 5, 5, 1);

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

    Eigen::SparseMatrix<double> matA(tripletListB.size(), between_nodes.size() * 6);
    Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
    Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

    matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
    matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
    matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

    Eigen::SparseMatrix<double> AtPA(between_nodes.size() * 6, between_nodes.size() * 6);
    Eigen::SparseMatrix<double> AtPB(between_nodes.size() * 6, 1);

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
    std::cout << "result: row, col, value" << std::endl;
    for (int k = 0; k < x.outerSize(); ++k)
    {
        for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
        {
            // std::cout << it.row() << " " << it.col() << " " << it.value() << std::endl;
            h_x.push_back(it.value());
        }
    }

    if (h_x.size() == 6 * between_nodes.size())
    {
        int counter = 0;

        for (size_t i = 0; i < between_nodes.size(); i++)
        {
            TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(between_nodes[i].node_outer.m_pose);
            pose.px += h_x[counter++];
            pose.py += h_x[counter++];
            pose.pz += h_x[counter++];
            pose.om += h_x[counter++];
            pose.fi += h_x[counter++];
            pose.ka += h_x[counter++];

            between_nodes[i].node_outer.m_pose = affine_matrix_from_pose_tait_bryan(pose);
        }
        std::cout << "optimizing with tait bryan finished" << std::endl;

        for (size_t i = 0; i < between_nodes.size(); i++)
        {
            fused_trajectory[between_nodes[i].node_outer.index_to_lidar_odometry_odo].m_pose = between_nodes[i].node_outer.m_pose;
        }
        for (size_t i = 0; i < between_nodes.size(); i++)
        {
            for (size_t j = 0; j < between_nodes[i].nodes_between.size() - 1; j++)
            {
                fused_trajectory[between_nodes[i].nodes_between[j].index_to_lidar_odometry_odo].m_pose =
                    between_nodes[i].node_outer.m_pose * between_nodes[i].nodes_between[j].m_pose;
            }
        }

        fused_trajectory[fused_trajectory.size() - 1].m_pose = between_nodes[between_nodes.size() - 1].node_outer.m_pose *
            between_nodes[between_nodes.size() - 1].nodes_between[between_nodes[between_nodes.size() - 1].nodes_between.size() - 1].m_pose;
    }
    else
    {
        std::cout << "optimizing with tait bryan FAILED" << std::endl;
    }

    return true;
}

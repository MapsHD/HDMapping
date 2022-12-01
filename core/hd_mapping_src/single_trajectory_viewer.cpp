#include <single_trajectory_viewer.h>
#include <portable-file-dialogs.h>
#include <GL/freeglut.h>
#include <transformations.h>

#include <filesystem>

namespace fs = std::filesystem;

bool SingleTrajectoryViewer::load_fused_trajectory(const std::string &file_name)
{
    trajectory_filename = file_name;
    trajectory_container.fused_trajectory = trajectory_container.load_trajectory(trajectory_filename);
    if (trajectory_container.fused_trajectory.size() > 0) {
        current_time_stamp = (trajectory_container.fused_trajectory[0].timestamp +
            trajectory_container.fused_trajectory[trajectory_container.fused_trajectory.size() - 1].timestamp) * 0.5;
    }

    working_directory = fs::path(trajectory_filename).parent_path().string();
    return load_vector_data(working_directory.c_str() + std::string("/chunks.bin"), chunk_files);
}

bool SingleTrajectoryViewer::load_fused_trajectory()
{
    static std::shared_ptr<pfd::open_file> open_file;
    std::string input_file_name = "";
    ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
    const auto t = [&]() {
        auto sel = pfd::open_file("Choose folder", "C:\\").result();
        for (int i = 0; i < sel.size(); i++)
        {
            input_file_name = sel[i];
            std::cout << "trajectory file: '" << input_file_name << "'" << std::endl;
        }
    };
    std::thread t1(t);
    t1.join();

    if (input_file_name.size() > 0) {
        return load_fused_trajectory(input_file_name);
    }
    else {
        return false;
    }
}

std::vector<Point> SingleTrajectoryViewer::load_points_and_transform_to_global(double ts, Eigen::Vector3d roi, float roi_size, int ext)
{
    std::vector<Point> pp;
    int index_pc = -1;

    for (size_t i = 0; i < chunk_files.size(); i++) {
        if ((chunk_files[i].time_begin_inclusive <= ts && chunk_files[i].time_end_inclusive >= ts)) {
            index_pc = i;
        }
    }

    if (index_pc != -1) {
        points.clear();
        for (int ind = index_pc - ext; ind <= index_pc + ext; ind++) {
            if (ind >= 0 && ind < chunk_files.size()) {
                std::vector<Point> points_tmp;
                load_vector_data(working_directory.c_str() + std::string("/") + std::to_string(chunk_files[ind].filename) + ".bin", points_tmp);

                for (size_t ii = 0; ii < points_tmp.size(); ii++) {
                    auto it = std::lower_bound(trajectory_container.fused_trajectory.begin(), trajectory_container.fused_trajectory.end(),
                        points_tmp[ii].time, [](Node lhs, double time) -> bool { return lhs.timestamp < time; });

                    int index1 = it - trajectory_container.fused_trajectory.begin() - 1;
                    if (index1 < 0) {
                        index1 = 0;
                    }

                    int index2 = index1 + 1;
                    if (index2 >= trajectory_container.fused_trajectory.size()) index2 = index1;

                    Eigen::Affine3d pi = pose_interpolation(points_tmp[ii].time,
                        trajectory_container.fused_trajectory[index1].timestamp,
                        trajectory_container.fused_trajectory[index2].timestamp,
                        trajectory_container.fused_trajectory[index1].m_pose,
                        trajectory_container.fused_trajectory[index2].m_pose);

                    Eigen::Vector3d p(points_tmp[ii].x, points_tmp[ii].y, points_tmp[ii].z);
                    Eigen::Vector3d pt = pi * p;

                    if (p.norm() > 10 && p.norm() < 50) {
                        points_tmp[ii].x = pt.x();
                        points_tmp[ii].y = pt.y();
                        points_tmp[ii].z = pt.z();

                        if (pt.x() >= (roi.x() - roi_size) && pt.x() <= (roi.x() + roi_size)) {
                            if (pt.y() >= (roi.y() - roi_size) && pt.y() <= (roi.y() + roi_size)) {
                                pp.emplace_back(points_tmp[ii]);
                            }
                        }
                    }

                    if ((trajectory_container.fused_trajectory[index1].m_pose.translation() - trajectory_container.fused_trajectory[index2].m_pose.translation()).norm() < 0.1) {
                        ii += 10000;
                        //std::cout << "pp " << ii << " ";
                    }

                }
            }
        }
    }
    return pp;
}

void SingleTrajectoryViewer::imgui(const CommonData& common_data){
	ImGui::Begin("single trajectory viewer");
	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

    if (ImGui::Button("load fused trajectory")) {
        load_fused_trajectory();
    }

    if (trajectory_container.fused_trajectory.size() > 0) {
        double cts = current_time_stamp;
      
        ImGui::InputDouble("input current_time_stamp", &current_time_stamp, 1, 10, "%.3f");

        if (current_time_stamp < trajectory_container.fused_trajectory[0].timestamp) {
            current_time_stamp = trajectory_container.fused_trajectory[0].timestamp;
        }

        if (current_time_stamp >= trajectory_container.fused_trajectory[trajectory_container.fused_trajectory.size() - 1].timestamp) {
            current_time_stamp = trajectory_container.fused_trajectory[trajectory_container.fused_trajectory.size() - 1].timestamp;
        }

        if (cts != current_time_stamp) {
            points = load_points_and_transform_to_global(current_time_stamp, common_data.roi, common_data.roi_size * 100000, 5);
        }

        if (ImGui::Button("get point clouds for roi")) {
            point_clouds = get_point_cloud_for_roi(common_data.roi, common_data.roi_size);
        }
    }

	ImGui::End();
}

void SingleTrajectoryViewer::render() {
    glColor3d(0.4, 0, 1);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < trajectory_container.fused_trajectory.size(); i++) {
        glVertex3f(trajectory_container.fused_trajectory[i].m_pose(0, 3), trajectory_container.fused_trajectory[i].m_pose(1, 3), trajectory_container.fused_trajectory[i].m_pose(2, 3));
    }
    glEnd();

    glBegin(GL_POINTS);
    for (const auto& p : points) {
        glColor3f((float(p.intensity) + 100) / 256.0, (float(p.intensity) + 100) / 256.0, (float(p.intensity) + 100) / 256.0);
        glVertex3f(p.x, p.y, p.z);
    }
    glEnd();

    for (const auto& pc : point_clouds) {
        glBegin(GL_LINES);
            glColor3f(1.0f, 0.0f, 0.0f);
            glVertex3f(pc.m_pose(0,3), pc.m_pose(1, 3), pc.m_pose(2, 3));
            glVertex3f(pc.m_pose(0, 3) + pc.m_pose(0, 0), pc.m_pose(1, 3) + pc.m_pose(1, 0), pc.m_pose(2, 3) + pc.m_pose(2, 0));

            glColor3f(0.0f, 1.0f, 0.0f);
            glVertex3f(pc.m_pose(0, 3), pc.m_pose(1, 3), pc.m_pose(2, 3));
            glVertex3f(pc.m_pose(0, 3) + pc.m_pose(0, 1), pc.m_pose(1, 3) + pc.m_pose(1, 1), pc.m_pose(2, 3) + pc.m_pose(2, 1));

            glColor3f(0.0f, 0.0f, 1.0f);
            glVertex3f(pc.m_pose(0, 3), pc.m_pose(1, 3), pc.m_pose(2, 3));
            glVertex3f(pc.m_pose(0, 3) + pc.m_pose(0, 2), pc.m_pose(1, 3) + pc.m_pose(1, 2), pc.m_pose(2, 3) + pc.m_pose(2, 2));
        glEnd();

        glBegin(GL_POINTS);
        for (const auto& p : pc.points_global) {
            glColor3f((float(p.intensity) + 100) / 256.0, (float(p.intensity) + 100) / 256.0, (float(p.intensity) + 100) / 256.0);
            glVertex3f(p.x, p.y, p.z);
        }
        glEnd();
        
    }
}


bool inside_roi(const Eigen::Affine3d& m_pose, const Eigen::Vector3d& roi, const float& roi_size) {
    if (m_pose.translation().x() >= (roi.x() - roi_size) && m_pose.translation().x() <= (roi.x() + roi_size)) {
        if (m_pose.translation().y() >= (roi.y() - roi_size) && m_pose.translation().y() <= (roi.y() + roi_size)) {
            return true;
        }
    }
    return false;
}


std::vector<PointCloudWithPose> SingleTrajectoryViewer::get_point_cloud_for_roi(Eigen::Vector3d roi, float roi_size)
{
    std::vector<PointCloudWithPose> pcs;

    for (size_t i = 1; i < trajectory_container.fused_trajectory.size(); i++) {
        if (!inside_roi(trajectory_container.fused_trajectory[i - 1].m_pose, roi, roi_size) && inside_roi(trajectory_container.fused_trajectory[i].m_pose, roi, roi_size)) {
            PointCloudWithPose pc;
            pc.m_pose = trajectory_container.fused_trajectory[i].m_pose;
            pc.timestamp = trajectory_container.fused_trajectory[i].timestamp;
            pc.trajectory_file_name = trajectory_filename;
            pcs.push_back(pc);
        }
    }

    for (auto& pc : pcs) {
        pc.points_global = load_points_and_transform_to_global(pc.timestamp, roi, roi_size, 5);
    }

    return pcs;
}
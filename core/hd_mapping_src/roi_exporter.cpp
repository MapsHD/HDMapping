#include <roi_exporter.h>
#include <single_trajectory_viewer.h>
#include <laz_wrapper.h>
#include <plycpp.h>
#include <portable-file-dialogs.h>

#include <GL/freeglut.h>

#include <thread>
#include <iostream>
#include <filesystem>
#include <fstream>

std::vector<Point> decimate(std::vector<Point> pc) {
    std::vector<Point> pc_out;

    double resolution_X = 0.2;
    double resolution_Y = 0.2;
    double resolution_Z = 0.2;

    double min_x = 1000000000;// std::numeric_limits<double>::max();
    double max_x = -1000000000;//std::numeric_limits<double>::min();

    double min_y = 1000000000;//std::numeric_limits<double>::max();
    double max_y = -1000000000;//std::numeric_limits<double>::min();

    double min_z = 1000000000;//std::numeric_limits<double>::max();
    double max_z = -1000000000;//std::numeric_limits<double>::min();

    for (size_t i = 0; i < pc.size(); i++) {
        if (pc[i].x < min_x) min_x = pc[i].x;
        if (pc[i].x > max_x) max_x = pc[i].x;

        if (pc[i].y < min_y) min_y = pc[i].y;
        if (pc[i].y > max_y) max_y = pc[i].y;

        if (pc[i].z < min_z) min_z = pc[i].z;
        if (pc[i].z > max_z) max_z = pc[i].z;
    }

    long long unsigned int number_of_buckets_X = ((max_x - min_x) / resolution_X) + 1;
    long long unsigned int number_of_buckets_Y = ((max_y - min_y) / resolution_Y) + 1;
    long long unsigned int number_of_buckets_Z = ((max_z - min_z) / resolution_Z) + 1;
    
    long long unsigned int number_of_buckets = static_cast<long long unsigned int>(number_of_buckets_X) *
        static_cast<long long unsigned int>(number_of_buckets_Y) * static_cast<long long unsigned int>(number_of_buckets_Z);

    std::vector<std::pair<long long unsigned int, long long unsigned int>> indexes;

    for (long long unsigned int i = 0; i < pc.size(); i++) {
        long long unsigned int ix = (pc[i].x - min_x) / resolution_X;
        long long unsigned int iy = (pc[i].y - min_y) / resolution_Y;
        long long unsigned int iz = (pc[i].z - min_z) / resolution_Z;

        long long unsigned int index_of_bucket = ix * static_cast<long long unsigned int>(number_of_buckets_Y) *
            static_cast<long long unsigned int>(number_of_buckets_Z) + iy * static_cast<long long unsigned int>(number_of_buckets_Z) + iz;

        indexes.emplace_back(i, index_of_bucket);
    }

    std::sort(indexes.begin(), indexes.end(), [](std::pair<long long unsigned int, long long unsigned int>& a, 
        const std::pair<long long unsigned int, long long unsigned int>& b) { return (a.second < b.second) ; });

    for (int i = 1; i < indexes.size(); i++) {
        if (indexes[i - 1].second != indexes[i].second) {
            pc_out.emplace_back(pc[indexes[i].first]);
        }
    }
    return pc_out;
}

void get_point_cloud_for_roi_job(int i, Job* job, std::vector<PointCloudWithPose>* pc, const ProjectSettings& project_setings, Eigen::Vector3d roi, float roi_size) {
    for (size_t ii = job->index_begin_inclusive; ii < job->index_end_exclusive; ii++) {
        SingleTrajectoryViewer tv;
        tv.load_fused_trajectory(project_setings.trajectories[ii].trajectory_file);
        //update trajectories from project
        //for(size_t i = 0 ; i < tv.trajectory_container.fused_trajectory)
        tv.trajectory_container.fused_trajectory = project_setings.trajectories[ii].fused_trajectory;
        
        auto point_clouds = tv.get_point_cloud_for_roi(roi, roi_size);
        
        if (point_clouds.size() > 0) {
            for (auto& pc : point_clouds) {
                pc.points_global = decimate(pc.points_global);
            }
            (*pc) = point_clouds;
        }
    }
}

void RoiExporter::imgui(CommonData& common_data, const ProjectSettings& project_setings, std::vector<LAZSector>& sectors) {
	ImGui::Begin("ROI exporter");
	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

    //if (ImGui::Button("reset view (angles)")) {
    //    common_data.rotate_x = 0.0;
    //    common_data.rotate_y = 0.0;
    //}
    //ImGui::SameLine(); 
    ImGui::Checkbox("ROI::is_ortho", &common_data.is_ortho);

    if (common_data.is_ortho) {
        ImGui::PushButtonRepeat(true);
        float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
        if (ImGui::ArrowButton("##roi_size_left_left", ImGuiDir_Left)) { common_data.roi_size -= 1.0; }
        ImGui::SameLine(0.0f, spacing);
        if (ImGui::ArrowButton("##roi_size_right_left", ImGuiDir_Right)) { common_data.roi_size += 1.0; }
        ImGui::PopButtonRepeat();
        ImGui::SameLine();
        ImGui::Text("roi size %0.1f", common_data.roi_size);
        if (common_data.roi_size < 10) common_data.roi_size = 10;
        if (common_data.roi_size > 100) common_data.roi_size = 100;

        ImGui::SameLine();

        if (ImGui::Button("remove observations inside ROI")) {

            std::vector<ROIwithConstraints> filtred_rois_with_constraints;
            for (size_t i = 0; i < rois_with_constraints.size(); i++) {
                std::vector<ConstraintToGeoreference> constraints;
                for (size_t j = 0; j < rois_with_constraints[i].constraints.size(); j++) {
                    Eigen::Affine3d& m = rois_with_constraints[i].constraints[j].m_pose;

                    bool inside = false;
                    if ((common_data.roi.x() - common_data.roi_size < m(0,3)) && (common_data.roi.x() + common_data.roi_size > m(0, 3))) {
                        if ((common_data.roi.y() - common_data.roi_size < m(1, 3)) && (common_data.roi.y() + common_data.roi_size > m(1, 3))) {
                            inside = true;
                        }
                    }
                    if (!inside) {
                        constraints.push_back(rois_with_constraints[i].constraints[j]);
                    }
                }

                std::cout << "constraints.size() " << constraints.size() << std::endl;

                if (constraints.size() > 0) {
                    ROIwithConstraints roi;
                    roi.constraints = constraints;
                    filtred_rois_with_constraints.push_back(roi);
                }
            }
            rois_with_constraints = filtred_rois_with_constraints;
        }
    }



    if (ImGui::Button("get point clouds for ROI")) {
        roi_point_clouds.clear();

        //georeference from LAZ
        PointCloudWithPose pc_laz = get_pc_from_laz(common_data, sectors, project_setings.shift_x, project_setings.shift_y);
        pc_laz.m_pose = Eigen::Affine3d::Identity();
        if (pc_laz.points_global.size() > 0) {
            roi_point_clouds.push_back(pc_laz);
        }

        std::vector<Job> jobs = get_jobs(project_setings.trajectories.size(), num_threads);
        std::vector<std::thread> threads;

        std::vector < std::vector<PointCloudWithPose>> roi_point_clouds_per_job;
        roi_point_clouds_per_job.resize(project_setings.trajectories.size());

        for (size_t i = 0; i < jobs.size(); i++) {
            threads.push_back(std::thread(get_point_cloud_for_roi_job, i, &jobs[i], &(roi_point_clouds_per_job[i]), project_setings, common_data.roi, common_data.roi_size));
        }

        for (size_t j = 0; j < threads.size(); j++) {
            threads[j].join();
        }
        threads.clear();

        for (size_t i = 0; i < roi_point_clouds_per_job.size(); i++) {
            if (roi_point_clouds_per_job[i].size() > 0) {
                roi_point_clouds.insert(roi_point_clouds.end(), roi_point_clouds_per_job[i].begin(), roi_point_clouds_per_job[i].end());
            }
        }

        /*for (auto t : project_setings.trajectories) {
            SingleTrajectoryViewer tv;
            tv.load_fused_trajectory(t.trajectory_file);
            auto point_clouds = tv.get_point_cloud_for_roi(common_data.roi, common_data.roi_size);
            for (const auto& pp : point_clouds) {
                 roi_point_clouds.push_back(pp);
            }
        }*/
    }
    ImGui::SameLine();
    ImGui::SliderInt("roi threads", &num_threads, 1, 128);
    ImGui::SliderInt("roi decimation", &decimation, 1, 1000);



    if (ImGui::Button("export ROI in RESSO format")) {
        export_to_RESSO_format(roi_point_clouds);
    }
    ImGui::SameLine();
    if (ImGui::Button("import ROI in RESSO format as constraints to georeference")) {
        import_from_RESSO_format_as_constraints_to_georeference();
    }

    if (ImGui::Button("all ROI point clouds visible")) {
        for (int i = 0; i < roi_point_clouds.size(); i++) {
            roi_point_clouds[i].visible = true;
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("all ROI point clouds not visible")) {
        for (int i = 0; i < roi_point_clouds.size(); i++) {
            roi_point_clouds[i].visible = false;
        }
    }

    for (int i = 0; i < roi_point_clouds.size(); i++) {
        ImGui::Checkbox(std::string("pc[" + std::to_string(i) + "] " + roi_point_clouds[i].trajectory_file_name).c_str(), &roi_point_clouds[i].visible);
        ImGui::SameLine();
        ImGui::PushButtonRepeat(true);
        float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
        if (ImGui::ArrowButton(("[" + std::to_string(i) + "] ##roi_left").c_str(), ImGuiDir_Left)) { (roi_point_clouds[i].point_size)--; }
        ImGui::SameLine(0.0f, spacing);
        if (ImGui::ArrowButton(("[" + std::to_string(i) + "] ##roi_right").c_str(), ImGuiDir_Right)) { (roi_point_clouds[i].point_size)++; }
        ImGui::PopButtonRepeat();
        ImGui::SameLine();
        ImGui::Text("line width %d", roi_point_clouds[i].point_size);
        if (roi_point_clouds[i].point_size < 1) roi_point_clouds[i].point_size = 1;

    }
   
    ImGui::End(); 
}

void RoiExporter::render(const CommonData &common_data) {
    glColor3f(0, 0, 0);
    glBegin(GL_LINE_STRIP);
        glVertex3f(common_data.roi.x() - common_data.roi_size, common_data.roi.y() - common_data.roi_size, common_data.roi.z());
        glVertex3f(common_data.roi.x() + common_data.roi_size, common_data.roi.y() - common_data.roi_size, common_data.roi.z());
        glVertex3f(common_data.roi.x() + common_data.roi_size, common_data.roi.y() + common_data.roi_size, common_data.roi.z());
        glVertex3f(common_data.roi.x() - common_data.roi_size, common_data.roi.y() + common_data.roi_size, common_data.roi.z());
        glVertex3f(common_data.roi.x() - common_data.roi_size, common_data.roi.y() - common_data.roi_size, common_data.roi.z());
    glEnd();
    
    for (const auto& pp : roi_point_clouds) {
        if (pp.visible) {
            glPointSize(pp.point_size);
            glBegin(GL_POINTS);
            for (size_t i = 0; i < pp.points_global.size(); i += decimation) {
                const auto& p = pp.points_global[i];
                glColor3f((float(p.intensity) + 100) / 256.0, (float(p.intensity) + 100) / 256.0, (float(p.intensity) + 100) / 256.0);
                glVertex3f(p.x, p.y, p.z);
            }
            glPointSize(1);
            glEnd();
        }
    }

    for (size_t i = 0; i < rois_with_constraints.size(); i++) {
        for (size_t j = 0; j < rois_with_constraints[i].constraints.size(); j++) {
           // glBegin(GL_LINES);
            Eigen::Affine3d& m = rois_with_constraints[i].constraints[j].m_pose;
            /*float scale = 10;
                glColor3f(1, 0, 0);
                glVertex3f(m(0, 3), m(1, 3), m(2, 3));
                glVertex3f(m(0, 3) + m(0,0) * scale, m(1, 3) + m(1,0) * scale, m(2, 3) + m(2,0) * scale);

                glColor3f(0, 1, 0);
                glVertex3f(m(0, 3), m(1, 3), m(2, 3));
                glVertex3f(m(0, 3) + m(0, 1) * scale, m(1, 3) + m(1, 1) * scale, m(2, 3) + m(2, 1) * scale);

                glColor3f(0, 0, 1);
                glVertex3f(m(0, 3), m(1, 3), m(2, 3));
                glVertex3f(m(0, 3) + m(0, 2) * scale, m(1, 3) + m(1, 2) * scale, m(2, 3) + m(2, 2) * scale);
            glEnd();*/

            glColor3f(0, 0, 0);
            glPointSize(10);
            glBegin(GL_POINTS);
            glVertex3f(m(0, 3), m(1, 3), m(2, 3));
            glEnd();
            glPointSize(1);
        }
    }
}

PointCloudWithPose RoiExporter::get_pc_from_laz(CommonData& common_data, std::vector<LAZSector>& sectors, double shift_x, double shift_y)
{
    std::cout << "Number of LAZSectors: " << sectors.size() << std::endl;

    PointCloudWithPose pc;
    pc.trajectory_file_name = "georeference";
    pc.m_pose = Eigen::Affine3d::Identity();
    pc.timestamp = 0.0;
    pc.visible = true;

    std::vector<int> indexes;

    for (size_t i = 0; i < sectors.size(); i++) {
        if ((common_data.roi.x() - common_data.roi_size > sectors[i].min_x) && (common_data.roi.x() - common_data.roi_size < sectors[i].max_x)) {
            if ((common_data.roi.y() - common_data.roi_size > sectors[i].min_y) && (common_data.roi.y() - common_data.roi_size < sectors[i].max_y)) {
                indexes.push_back(i);
            }
        }
        if ((common_data.roi.x() - common_data.roi_size > sectors[i].min_x) && (common_data.roi.x() - common_data.roi_size < sectors[i].max_x)) {
            if ((common_data.roi.y() + common_data.roi_size > sectors[i].min_y) && (common_data.roi.y() + common_data.roi_size < sectors[i].max_y)) {
                indexes.push_back(i);
            }
        }
        if ((common_data.roi.x() + common_data.roi_size > sectors[i].min_x) && (common_data.roi.x() + common_data.roi_size < sectors[i].max_x)) {
            if ((common_data.roi.y() + common_data.roi_size > sectors[i].min_y) && (common_data.roi.y() + common_data.roi_size < sectors[i].max_y)) {
                indexes.push_back(i);
            }
        }
        if ((common_data.roi.x() + common_data.roi_size > sectors[i].min_x) && (common_data.roi.x() + common_data.roi_size < sectors[i].max_x)) {
            if ((common_data.roi.y() - common_data.roi_size > sectors[i].min_y) && (common_data.roi.y() - common_data.roi_size < sectors[i].max_y)) {
                indexes.push_back(i);
            }
        }
    }

    std::sort(indexes.begin(), indexes.end());
    auto last = std::unique(indexes.begin(), indexes.end());
    indexes.erase(last, indexes.end());

    for (const auto &i : indexes) {
        if (sectors[i].point_cloud.size() == 0) {
            LazWrapper lw;
            LAZSector sector = lw.load_sector(sectors[i].file_name, shift_x, shift_y);
            sectors[i].point_cloud = sector.point_cloud;
            sectors[i].visible = false;
        }

        for (const auto& p : sectors[i].point_cloud) {
            float roi_size_extended = common_data.roi_size * 1.5;
            if ((p.x > common_data.roi.x() - roi_size_extended) && (p.x < common_data.roi.x() + roi_size_extended)) {
                if ((p.y > common_data.roi.y() - roi_size_extended) && (p.y < common_data.roi.y() + roi_size_extended)) {
                    Point new_point;
                    new_point.x = p.x;
                    new_point.y = p.y;
                    new_point.z = p.z;
                    new_point.time = 0.0;
                    new_point.intensity = ((float(p.r) + float(p.g) + float(p.b) ) / 3.0f) *256;
                    pc.points_global.push_back(new_point);
                }
            }
        }
    }
   
    std::cout << pc.points_global.size() << std::endl;
    return pc;
}

typedef std::vector<std::vector<float> > Cloud;

void RoiExporter::export_to_RESSO_format(std::vector<PointCloudWithPose>& roi_point_clouds)
{
    static std::shared_ptr<pfd::save_file> save_file;
    std::string output_file_name = "";
    ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)save_file);
    const auto t = [&]() {
        auto sel = pfd::save_file("Choose file", "C:\\").result();
        output_file_name = sel;
        std::cout << "file to save: '" << output_file_name << "'" << std::endl;
    };
    std::thread t1(t);
    t1.join();

    if (output_file_name.size() > 0) {
        std::ofstream outfile;
        outfile.open(output_file_name);
        if (!outfile.good()) {
            std::cout << "can not save file: " << output_file_name << std::endl;
            return;
        }

        m_fake = roi_point_clouds[1].m_pose.inverse();

        outfile << roi_point_clouds.size() << std::endl;
        for (size_t i = 0; i < roi_point_clouds.size(); i++) {
            outfile << std::to_string(i)+".ply" << std::endl;
            outfile << (m_fake * roi_point_clouds[i].m_pose)(0, 0) << " " << (m_fake * roi_point_clouds[i].m_pose)(0, 1) << " " << (m_fake * roi_point_clouds[i].m_pose)(0, 2) << " " << (m_fake * roi_point_clouds[i].m_pose)(0, 3) << std::endl;
            outfile << (m_fake * roi_point_clouds[i].m_pose)(1, 0) << " " << (m_fake * roi_point_clouds[i].m_pose)(1, 1) << " " << (m_fake * roi_point_clouds[i].m_pose)(1, 2) << " " << (m_fake * roi_point_clouds[i].m_pose)(1, 3) << std::endl;
            outfile << (m_fake * roi_point_clouds[i].m_pose)(2, 0) << " " << (m_fake * roi_point_clouds[i].m_pose)(2, 1) << " " << (m_fake * roi_point_clouds[i].m_pose)(2, 2) << " " << (m_fake * roi_point_clouds[i].m_pose)(2, 3) << std::endl;
            outfile << "0 0 0 1" << std::endl;
        }
        outfile.close();

        std::filesystem::path my_path(output_file_name);
        std::cout << my_path.parent_path() << std::endl;

        for (size_t i = 0; i < roi_point_clouds.size(); i++) {
            std::filesystem::path pp = my_path.parent_path();
            pp /= std::to_string(i) + ".ply";
            std::string out_file_name = pp.string();

            plycpp::PLYData newPlyData;
            Cloud points;

            Eigen::Affine3d m_inv = ((roi_point_clouds[i].m_pose)).inverse();
            for (auto& p : roi_point_clouds[i].points_global) {
                Eigen::Vector3d p_src(p.x, p.y, p.z);
                Eigen::Vector3d p_trg = m_inv * p_src;

                std::vector<float> pp;
                pp.push_back(p_trg.x());
                pp.push_back(p_trg.y());
                pp.push_back(p_trg.z());
                points.push_back(pp);
            }
            
            plycpp::fromPointCloud<float, Cloud>(points, newPlyData);

            std::cout << "trying save to: '" << out_file_name << "'" << std::endl;
            plycpp::save(out_file_name, newPlyData, plycpp::FileFormat::BINARY);
        }
    }
}

void RoiExporter::import_from_RESSO_format_as_constraints_to_georeference()
{
    static std::shared_ptr<pfd::open_file> open_file;
    std::string input_file_name = "";
    ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
    const auto t = [&]() {
        auto sel = pfd::open_file("Choose folder", "C:\\").result();
        for (int i = 0; i < sel.size(); i++)
        {
            input_file_name = sel[i];
            std::cout << "RESSO file: '" << input_file_name << "'" << std::endl;
        }
    };
    std::thread t1(t);
    t1.join();

    if (input_file_name.size() > 0) {
        std::vector<Eigen::Affine3d> m_poses;


        std::ifstream infile(input_file_name);
        if (!infile.good()) {
            std::cout << "problem with file: '" << input_file_name << "'" << std::endl;
            return;
        }
        std::string line;
        std::getline(infile, line);
        std::istringstream iss(line);

        int num_scans;
        iss >> num_scans;

        std::cout << "number of constraints: " << num_scans << std::endl;

        for (size_t i = 0; i < num_scans; i++) {
            std::getline(infile, line);
            std::istringstream iss(line);
            std::string point_cloud_file_name;
            iss >> point_cloud_file_name;

            double r11, r12, r13, r21, r22, r23, r31, r32, r33;
            double t14, t24, t34;

            std::getline(infile, line);
            std::istringstream iss1(line);
            iss1 >> r11 >> r12 >> r13 >> t14;

            std::getline(infile, line);
            std::istringstream iss2(line);
            iss2 >> r21 >> r22 >> r23 >> t24;

            std::getline(infile, line);
            std::istringstream iss3(line);
            iss3 >> r31 >> r32 >> r33 >> t34;

            std::getline(infile, line);

            Eigen::Affine3d m_pose = Eigen::Affine3d::Identity();
            m_pose(0, 0) = r11;
            m_pose(0, 1) = r12;
            m_pose(0, 2) = r13;
            m_pose(1, 0) = r21;
            m_pose(1, 1) = r22;
            m_pose(1, 2) = r23;
            m_pose(2, 0) = r31;
            m_pose(2, 1) = r32;
            m_pose(2, 2) = r33;
            m_pose(0, 3) = t14;
            m_pose(1, 3) = t24;
            m_pose(2, 3) = t34;

            std::cout << "loaded pose: " << std::endl;
            std::cout << m_pose.matrix() << std::endl;

            m_poses.push_back(m_pose);
        }
        infile.close();

        for (size_t i = 0; i < m_poses.size(); i++) {
            m_poses[i] = m_fake.inverse() * m_poses[i];
        }

        ROIwithConstraints roi_with_constraints;

        for (size_t i = 1; i < m_poses.size(); i++) {
            ConstraintToGeoreference roi_c;
            roi_c.m_pose = m_poses[i];
            roi_c.time_stamp = roi_point_clouds[i].timestamp;
            roi_c.trajectory_file_name = roi_point_clouds[i].trajectory_file_name;
            roi_with_constraints.constraints.push_back(roi_c);
        }

        rois_with_constraints.push_back(roi_with_constraints);
    }
}
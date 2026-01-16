#pragma once

#include <Eigen/Eigen>
#include <GL/freeglut.h>
#include <imgui.h>
#include <map>
#include <nlohmann/json.hpp>
#include <structures.h>
#include <transformations.h>

struct PointInsideIntersection
{
    double x_local;
    double y_local;
    double z_local;
    int index_scan;
    std::string source_filename;
    Eigen::Vector3d color;
};

class Intersection
{
public:
    Intersection() = default;
    ~Intersection() = default;

    float color[3];
    float translation[3];
    float rotation[3];
    float width_length_height[3];
    std::vector<PointInsideIntersection> points;

    void render()
    {
        TaitBryanPose pose;
        pose.px = translation[0];
        pose.py = translation[1];
        pose.pz = translation[2];
        pose.om = rotation[0];
        pose.fi = rotation[1];
        pose.ka = rotation[2];
        Eigen::Affine3d m_pose = affine_matrix_from_pose_tait_bryan(pose);

        float& x_length = width_length_height[0];
        float& y_width = width_length_height[1];
        float& z_height = width_length_height[2];

        glColor3f(color[0], color[1], color[2]);

        Eigen::Vector3d v000(-x_length * 0.5, -y_width * 0.5, -z_height * 0.5);
        Eigen::Vector3d v100(x_length * 0.5, -y_width * 0.5, -z_height * 0.5);
        Eigen::Vector3d v110(x_length * 0.5, y_width * 0.5, -z_height * 0.5);
        Eigen::Vector3d v010(-x_length * 0.5, y_width * 0.5, -z_height * 0.5);

        Eigen::Vector3d v001(-x_length * 0.5, -y_width * 0.5, z_height * 0.5);
        Eigen::Vector3d v101(x_length * 0.5, -y_width * 0.5, z_height * 0.5);
        Eigen::Vector3d v111(x_length * 0.5, y_width * 0.5, z_height * 0.5);
        Eigen::Vector3d v011(-x_length * 0.5, y_width * 0.5, z_height * 0.5);

        Eigen::Vector3d v000t = m_pose * v000;
        Eigen::Vector3d v100t = m_pose * v100;
        Eigen::Vector3d v110t = m_pose * v110;
        Eigen::Vector3d v010t = m_pose * v010;

        Eigen::Vector3d v001t = m_pose * v001;
        Eigen::Vector3d v101t = m_pose * v101;
        Eigen::Vector3d v111t = m_pose * v111;
        Eigen::Vector3d v011t = m_pose * v011;

        glBegin(GL_LINES);

        glVertex3d(v000t.x(), v000t.y(), v000t.z());
        glVertex3d(v100t.x(), v100t.y(), v100t.z());

        glVertex3d(v100t.x(), v100t.y(), v100t.z());
        glVertex3d(v110t.x(), v110t.y(), v110t.z());

        glVertex3d(v110t.x(), v110t.y(), v110t.z());
        glVertex3d(v010t.x(), v010t.y(), v010t.z());

        glVertex3d(v010t.x(), v010t.y(), v010t.z());
        glVertex3d(v000t.x(), v000t.y(), v000t.z());

        glVertex3d(v001t.x(), v001t.y(), v001t.z());
        glVertex3d(v101t.x(), v101t.y(), v101t.z());

        glVertex3d(v101t.x(), v101t.y(), v101t.z());
        glVertex3d(v111t.x(), v111t.y(), v111t.z());

        glVertex3d(v111t.x(), v111t.y(), v111t.z());
        glVertex3d(v011t.x(), v011t.y(), v011t.z());

        glVertex3d(v011t.x(), v011t.y(), v011t.z());
        glVertex3d(v001t.x(), v001t.y(), v001t.z());

        glVertex3d(v000t.x(), v000t.y(), v000t.z());
        glVertex3d(v001t.x(), v001t.y(), v001t.z());

        glVertex3d(v100t.x(), v100t.y(), v100t.z());
        glVertex3d(v101t.x(), v101t.y(), v101t.z());

        glVertex3d(v110t.x(), v110t.y(), v110t.z());
        glVertex3d(v111t.x(), v111t.y(), v111t.z());

        glVertex3d(v010t.x(), v010t.y(), v010t.z());
        glVertex3d(v011t.x(), v011t.y(), v011t.z());

        //
        glVertex3d(v000t.x(), v000t.y(), v000t.z());
        glVertex3d(v110t.x(), v110t.y(), v110t.z());

        glVertex3d(v010t.x(), v010t.y(), v010t.z());
        glVertex3d(v100t.x(), v100t.y(), v100t.z());

        glVertex3d(v001t.x(), v001t.y(), v001t.z());
        glVertex3d(v111t.x(), v111t.y(), v111t.z());

        glVertex3d(v011t.x(), v011t.y(), v011t.z());
        glVertex3d(v101t.x(), v101t.y(), v101t.z());

        glEnd();
    }
};

class ObservationPicking
{
public:
    ObservationPicking() = default;
    ~ObservationPicking() = default;

    bool is_observation_picking_mode = false;
    float picking_plane_height = 0.0f;
    float picking_plane_threshold = 0.1f;
    float max_xy = 200.0f;
    int point_size = 1;
    // bool high_density_grid = false;
    bool grid10x10m = false;
    bool grid1x1m = false;
    bool grid01x01m = false;
    bool grid001x001m = false;
    void render();
    void add_picked_to_current_observation(int index_picked, Eigen::Vector3d p);
    void accept_current_observation(std::vector<Eigen::Affine3d> m_poses);
    void import_observations(const std::string& filename);
    void export_observation(const std::string& filename);
    void add_intersection(Eigen::Vector3d translation);

    std::map<int, Eigen::Vector3d> current_observation;
    std::vector<std::map<int, Eigen::Vector3d>> observations;
    std::vector<Intersection> intersections;
    float label_dist = 100.0f;
};

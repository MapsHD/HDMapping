#pragma once

#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <imgui_internal.h>

#include <ImGuizmo.h>

#include <Eigen/Eigen>

#include <structures.h>

class OdoWithGnssFusion
{
public:
    OdoWithGnssFusion() = default;
    ~OdoWithGnssFusion() = default;

    void imgui(CommonData& common_data);
    void render();

    std::vector<Node> load_trajectory(const std::string& file_name);
    bool save_trajectory(const std::string& file_name);
    void update_shift(const float shift_x, const float shift_y, CommonData& common_data);
    std::vector<std::pair<int, int>> find_correspondences_from_lo_to_shifted_gnss();
    std::vector<BetweenNode> find_between_nodes();

    bool rigid_registration(bool adaptive_robust_kernel);
    bool semi_rigid_registration();

    bool show_correspondences_rigid_registration = true;
    bool show_correspondences_semi_rigid_registration = true;

    std::vector<Node> gnss_trajectory = {};
    std::vector<Node> gnss_trajectory_shifted = {};
    std::vector<Node> fused_trajectory = {};
    std::vector<Node> fused_trajectory_motion_model = {};
    std::vector<Node> lidar_odometry_trajectory_initial = {};
    std::vector<std::pair<int, int>> correspondences_from_lo_to_shifted_gnss = {};
    std::vector<BetweenNode> between_nodes = {};
    std::string trajectory_file = {};
    std::string motion_model_file_name = {};
    float color_x = 0;
    float color_y = 0;
    float color_z = 0;
    bool visible = true;
    int line_width = 1;
    bool set_initial_offset_from_trajectory = true;
};

#pragma once

#include <point_clouds.h>
#include <string>
#include <vector>

#if WITH_GUI == 1
#include <control_points.h>
#include <ground_control_points.h>
#include <manual_pose_graph_loop_closure.h>

#else
#include <pose_graph_loop_closure.h>
#endif

class Session
{
public:
    Session() = default;
    ~Session() = default;

    PointClouds point_clouds_container;
    std::string working_directory = "";
    bool visible = true;
    bool is_gizmo = false;
    float render_color[3];
    std::string session_file_name = "";
    bool is_ground_truth = false;
    //bool show_rgb = true;
    bool load_cache_mode = false;

#if WITH_GUI == 1
    ManualPoseGraphLoopClosure pose_graph_loop_closure;
    GroundControlPoints ground_control_points;
    ControlPoints control_points;
#else
    PoseGraphLoopClosure pose_graph_loop_closure;
#endif

    bool load(const std::string& file_name, bool is_decimate, double bucket_x, double bucket_y, double bucket_z, bool calculate_offset);
    bool save(
        const std::string& file_name, const std::string& poses_file_name, const std::string& initial_poses_file_name, bool is_subsession);
    void fill_session_from_worker_data(
        const std::vector<WorkerData>& worker_data,
        bool save_selected,
        bool filter_on_export,
        bool apply_pose,
        double threshould_output_filter);
};

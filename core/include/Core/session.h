#pragma once

#include <Core/point_clouds.h>

#include <string>
#include <vector>

#include <Core/control_points.h>
#include <Core/ground_control_points.h>
#include <Core/manual_pose_graph_loop_closure.h>

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
    // bool show_rgb = true;
    bool load_cache_mode = false;

    // No WITH_GUI branches here: Session is shared between core_math (WITH_GUI=0) and GUI apps, so its layout must not depend on it.
    ManualPoseGraphLoopClosure pose_graph_loop_closure;
    GroundControlPoints ground_control_points;
    ControlPoints control_points;

    // file_name is a session (*.mjs, *.json) or a single point cloud (*.laz, *.las), see load_from_point_cloud
    bool load(const std::string& file_name, bool is_decimate, double bucket_x, double bucket_y, double bucket_z, bool calculate_offset);
    // Builds a session in memory from one point cloud: identity pose, one identity trajectory node, no session files
    bool load_from_point_cloud(
        const std::string& file_name, bool is_decimate, double bucket_x, double bucket_y, double bucket_z, bool calculate_offset);
    static bool is_point_cloud_file(const std::string& file_name);
    /**
     * @brief Converts a path to the form stored in a session file: relative to the session .mjs directory.
     * @param path Absolute path, or relative to the current working directory.
     * @param session_file_name Path of the session .mjs the result will be stored in.
     * @return Path relative to the .mjs directory with '/' separators, the absolute path when no relative one exists
     *         (e.g. another drive on Windows), or an empty string for an empty path.
     * @note load() accepts both these relative paths and the absolute paths of older sessions.
     */
    static std::string resolve_session_path(const std::string& path, const std::string& session_file_name);
    bool save(
        const std::string& file_name, const std::string& poses_file_name, const std::string& initial_poses_file_name, bool is_subsession);
    void fill_session_from_worker_data(
        const std::vector<WorkerData>& worker_data,
        bool save_selected,
        bool filter_on_export,
        bool apply_pose,
        double threshould_output_filter);
};

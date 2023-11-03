#ifndef _SESSION_H_
#define _SESSION_H_

#include <string>
#include <point_clouds.h>
#include <manual_pose_graph_loop_closure.h>

class Session
{
    public:
    Session(){;};
    ~Session(){;};

    bool load(const std::string &file_name, bool is_decimate, double bucket_x, double bucket_y, double bucket_z, bool calculate_offset);
    bool save(const std::string &file_name, const std::string &poses_file_name, const std::string &initial_poses_file_name, bool is_subsession);

    PointClouds point_clouds_container;
    ManualPoseGraphLoopClosure manual_pose_graph_loop_closure;
    std::string working_directory = "";
    bool visible = true;
    bool is_gizmo = false;
    float render_color[3];
    std::string session_file_name = "";
};

#endif
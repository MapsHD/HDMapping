#ifndef _SESSION_H_
#define _SESSION_H_

#include <string>
#include <vector>
#include <point_clouds.h>
#include <manual_pose_graph_loop_closure.h>
#include <ground_control_points.h>

std::string pathUpdater(std::string path, std::string newPath);

class Session
{
    public:
    Session(){;};
    ~Session(){;};

    bool load(const std::string &file_name, bool is_decimate, double bucket_x, double bucket_y, double bucket_z, bool calculate_offset);
    bool save(const std::string &file_name, const std::string &poses_file_name, const std::string &initial_poses_file_name, bool is_subsession);

    PointClouds point_clouds_container;
    ManualPoseGraphLoopClosure manual_pose_graph_loop_closure;
    GroundControlPoints ground_control_points;
  

    std::string working_directory = "";
    bool visible = true;
    bool is_gizmo = false;
    float render_color[3];
    std::string session_file_name = "";
    bool is_ground_truth = false;
    bool show_rgb = true;
};

#endif
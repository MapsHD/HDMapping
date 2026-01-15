#pragma once

#include <session.h>

struct Edge
{
    TaitBryanPose relative_pose_tb;
    TaitBryanPose relative_pose_tb_weights;
    int index_session_from;
    int index_session_to;
    int index_from;
    int index_to;
    bool is_fixed_px = false;
    bool is_fixed_py = false;
    bool is_fixed_pz = false;
    bool is_fixed_om = false;
    bool is_fixed_fi = false;
    bool is_fixed_ka = false;
};

bool optimize(std::vector<Session>& sessions, const std::vector<Edge>& edges);

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

    bool save(const std::string &file_name);

    PointClouds point_clouds_container;
    ManualPoseGraphLoopClosure manual_pose_graph_loop_closure;
};

#endif
#ifndef _manual_pose_graph_loop_closure_h_
#define _manual_pose_graph_loop_closure_h_

#include <point_clouds.h>

class ManualPoseGraphLoopClosure{
    public:
    ManualPoseGraphLoopClosure(){};
    ~ManualPoseGraphLoopClosure(){};

    void Gui();
    void Render(const PointClouds &point_clouds_container);
};

#endif

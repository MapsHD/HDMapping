#ifndef _manual_pose_graph_loop_closure_h_
#define _manual_pose_graph_loop_closure_h_

#include <point_clouds.h>

class ManualPoseGraphLoopClosure{
    public:
        struct Edge
        {
            TaitBryanPose relative_pose_tb;
            TaitBryanPose relative_pose_tb_weights;
            int index_from;
            int index_to;
        };

    ManualPoseGraphLoopClosure(){};
    ~ManualPoseGraphLoopClosure(){};

    void Gui(PointClouds &point_clouds_container, int &index_loop_closure_source, int &index_loop_closure_target);
    void Render(PointClouds &point_clouds_container, int index_loop_closure_source, int index_loop_closure_target);
};

#endif

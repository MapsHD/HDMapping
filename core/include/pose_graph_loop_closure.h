#ifndef _graph_loop_closure_h_
#define _graph_loop_closure_h_

#include <point_clouds.h>
#include <gnss.h>

class PoseGraphLoopClosure{
    public:
        enum EdgeType
        {
            motion_model_1,
            motion_model_2,
            loop_closure
        };

        struct Edge
        {
            TaitBryanPose relative_pose_tb;
            TaitBryanPose relative_pose_tb_weights;
            int index_from;
            int index_to;
            bool is_fixed_px = false;
            bool is_fixed_py = false;
            bool is_fixed_pz = false;
            bool is_fixed_om = false;
            bool is_fixed_fi = false;
            bool is_fixed_ka = false;
            EdgeType type;
        };

        std::vector<Edge> edges;
        std::vector<Eigen::Affine3d> poses_motion_model;

        PoseGraphLoopClosure(){};
        ~PoseGraphLoopClosure(){};
};

#endif

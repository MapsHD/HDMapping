#ifndef _graph_loop_closure_h_
#define _graph_loop_closure_h_

#include <point_clouds.h>
#include <gnss.h>
#include <ground_control_points.h>
#include <control_points.h>

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

        void add_edge(
            PointClouds &point_clouds_container, int index_loop_closure_source, int index_loop_closure_target);
        void set_initial_poses_as_motion_model(PointClouds &point_clouds_container);
        void set_current_poses_as_motion_model(PointClouds &point_clouds_container);
        void graph_slam(PointClouds &point_clouds_container, GNSS &gnss, GroundControlPoints &gcps, ControlPoints &cps);
        void FuseTrajectoryWithGNSS(PointClouds &point_clouds_container, GNSS &gnss);
        void run_icp(PointClouds &point_clouds_container, int index_active_edge, float search_radius, int number_of_iterations, int num_edge_extended_before, int num_edge_extended_after);
    };

#endif

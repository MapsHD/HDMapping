#ifndef _manual_pose_graph_loop_closure_h_
#define _manual_pose_graph_loop_closure_h_
#if WITH_GUI == 1

#include <point_clouds.h>
#include <gnss.h>
#include <observation_picking.h>
#include <pose_graph_loop_closure.h>
#include <imgui.h>
#include <GL/freeglut.h>

class ManualPoseGraphLoopClosure : public PoseGraphLoopClosure
{
public:
    int index_active_edge = 0;
    bool manipulate_active_edge = false;
    bool gizmo = false;
    double search_radious = 0.1;

    ManualPoseGraphLoopClosure() {};
    ~ManualPoseGraphLoopClosure() {};

    void Gui(PointClouds &point_clouds_container, int &index_loop_closure_source, int &index_loop_closure_target, float *m_gizmo, GNSS &gnss,
             GroundControlPoints &gcps, ControlPoints &cps, int num_edge_extended_before, int num_edge_extended_after);
    void Render(PointClouds &point_clouds_container,
                int index_loop_closure_source, int index_loop_closure_target, 
                int num_edge_extended_before, int num_edge_extended_after);
};

#endif
#endif

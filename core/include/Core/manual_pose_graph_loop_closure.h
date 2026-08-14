#pragma once

#if WITH_GUI == 1
#include <Core/gnss.h>
#include <Core/observation_picking.h>
#include <Core/point_clouds.h>
#include <Core/pose_graph_loop_closure.h>
#include <Core/tum.h>

class ManualPoseGraphLoopClosure : public PoseGraphLoopClosure
{
public:
    int index_active_edge = 0;
    bool manipulate_active_edge = false;
    bool gizmo = false;
    double search_radious = 0.1;

    ManualPoseGraphLoopClosure() = default;
    ~ManualPoseGraphLoopClosure() = default;

    void Gui(
        PointClouds& point_clouds_container,
        int& index_loop_closure_source,
        int& index_loop_closure_target,
        float* m_gizmo,
        GNSS& gnss,
        TUM& tum,
        GroundControlPoints& gcps,
        ControlPoints& cps,
        int num_edge_extended_before,
        int num_edge_extended_after);
    void Render(
        PointClouds& point_clouds_container,
        int index_loop_closure_source,
        int index_loop_closure_target,
        int num_edge_extended_before,
        int num_edge_extended_after);
};

#endif

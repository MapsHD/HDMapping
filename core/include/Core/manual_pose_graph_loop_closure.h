#pragma once

#include <Core/gnss.h>
#include <Core/point_clouds.h>
#include <Core/pose_graph_loop_closure.h>
#include <Core/tum.h>

#include <chrono>

#if WITH_GUI == 1
#include <Core/observation_picking.h>
#endif

// Defined in every build (not only WITH_GUI) so Session has one layout; only the GUI methods are conditional.
class ManualPoseGraphLoopClosure : public PoseGraphLoopClosure
{
public:
    int index_active_edge = 0;
    bool manipulate_active_edge = false;
    bool gizmo = false;
    //! N / P step the active edge in Gui(); the app turns this off while another list takes these keys.
    bool edge_hotkeys = true;
    double search_radious = 0.1;
    //! "ICP all edges": first search radius of its coarse-to-fine steps, see PoseGraphLoopClosure::icp_search_radii.
    double icp_all_edges_start_radius = 1.0;
    //! Edges the last "ICP all edges" could not refine.
    std::vector<int> icp_failed_edges;
    //! Number of edges when "ICP all edges" last finished; 0 when it hasn't, or the edges changed since.
    size_t icp_all_edges_count = 0;
    //! "ICP all edges" in progress: it runs one ICP step per frame, so the window keeps drawing its progress.
    struct IcpAllEdgesRun
    {
        bool running = false;
        int edge = 0; //!< edge being refined
        size_t step = 0; //!< index into radii
        std::vector<float> radii;
        std::vector<int> failed;
        std::chrono::steady_clock::time_point start;
        bool progress_drawn = false; //!< the progress bar has been shown at least once
    };
    IcpAllEdgesRun icp_all_edges_run;

    ManualPoseGraphLoopClosure() = default;
    ~ManualPoseGraphLoopClosure() = default;

#if WITH_GUI == 1
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

private:
    //! Runs the next "ICP all edges" step; finishes the run after the last one.
    void icp_all_edges_step(PointClouds& point_clouds_container, int num_edge_extended_before, int num_edge_extended_after);
    //! The "ICP all edges" button, start radius and progress.
    void icp_all_edges_gui();
#endif
};

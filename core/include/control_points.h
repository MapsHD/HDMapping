#ifndef _CONTROL_POINTS_H_
#define _CONTROL_POINTS_H_

#include <vector>
#include <string>
#include <point_clouds.h>
#if WITH_GUI == 1
#include <GL/freeglut.h>
#endif

struct ControlPoint{
    char name[64];
    double x_source_local;
    double y_source_local;
    double z_source_local;
    double x_target_global;
    double y_target_global;
    double z_target_global;
    double sigma_x;
    double sigma_y;
    double sigma_z;
    int index_to_pose;
};

class ControlPoints{
    public:
        ControlPoints(){;};
        ~ControlPoints(){;};

        bool is_imgui = false;
        std::vector<ControlPoint> cps;
#if WITH_GUI == 1
        bool picking_mode = false;
        //int picking_mode_index_to_node_inner = -1;
        //int picking_mode_index_to_node_outer = -1;
        bool draw_uncertainty = false;
        int index_picked_point = -1;

        int index_pose = 0;

        //bool found_picked = false;
        //ControlPoint picked_control_point;

        void imgui(PointClouds &point_clouds_container, Eigen::Vector3f &rotation_center);
        void render(const PointClouds &point_clouds_container);
        void draw_ellipse(const Eigen::Matrix3d &covar, Eigen::Vector3d &mean, Eigen::Vector3f color, float nstd = 1);
#endif
};

#endif
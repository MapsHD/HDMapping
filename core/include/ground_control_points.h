#pragma once

#include <point_clouds.h>

#include <string>
#include <vector>

#if WITH_GUI == 1
#include <GL/freeglut.h>
#endif

struct GroundControlPoint
{
    char name[64];
    double x;
    double y;
    double z;
    double sigma_x;
    double sigma_y;
    double sigma_z;
    double lidar_height_above_ground;
    int index_to_node_inner;
    int index_to_node_outer;
};

class GroundControlPoints
{
public:
    GroundControlPoints()
    {
        ;
    };
    ~GroundControlPoints()
    {
        ;
    };

    std::vector<GroundControlPoint> gpcs;
    double default_lidar_height_above_ground = 0.15;
#if WITH_GUI == 1
    bool is_imgui = false;
    bool picking_mode = false;
    int picking_mode_index_to_node_inner = -1;
    int picking_mode_index_to_node_outer = -1;
    bool draw_uncertainty = false;

    void imgui(PointClouds& point_clouds_container);
    void render(const PointClouds& point_clouds_container);
    void draw_ellipse(const Eigen::Matrix3d& covar, Eigen::Vector3d& mean, Eigen::Vector3f color, float nstd = 1);
#endif
};

#pragma once

#include <Eigen/Eigen>
#include <vector>

#include <hash_utils.h>

class Surface
{
public:
    // struct TripletIndexes
    //{
    //     int index_before;
    //     int index_curr;
    //     int index_after;
    // };

    Surface()
    {
        ;
    };
    ~Surface()
    {
        ;
    };

    std::vector<Eigen::Affine3d> vertices;
    std::vector<Eigen::Affine3d> vertices_odo;
    // std::vector<TripletIndexes> smoothness_indexes;
    double surface_resolution = 1.0;
    int number_rows = 0;
    int number_cols = 0;
    std::vector<std::pair<int, int>> odo_edges;
    double z_sigma_threshold = 0.1;
    double lowest_points_resolution = 0.2;
    double lowest_points_filter_resolution = 0.3;
    bool robust_kernel = false;

    void generate_initial_surface(const std::vector<Eigen::Vector3d>& point_cloud);
    void render();
    void align_surface_to_ground_points(const std::vector<Eigen::Vector3d>& point_cloud);
    bool find_nearest_neighbour(Eigen::Vector3d& p_t, Eigen::Affine3d vertex, const std::vector<Eigen::Vector3d>& reference_points);
    std::vector<int> get_filtered_indexes(
        const std::vector<Eigen::Vector3d>& pc, const std::vector<int>& lowest_points_indexes, Eigen::Vector2d bucket_dim_xy);
    std::vector<Eigen::Vector3d> get_points_without_surface(
        const std::vector<Eigen::Vector3d>& points,
        double distance_to_ground_threshold_bottom,
        double distance_to_ground_threshold_up,
        Eigen::Vector2d bucket_dim_xy);
};

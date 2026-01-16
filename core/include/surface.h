#pragma once

#include <Eigen/Eigen>
#include <vector>


inline unsigned long long int get_index_2D(const int16_t x, const int16_t y /*, const int16_t z*/)
{
    // return ((static_cast<unsigned long long int>(x) << 32) & (0x0000FFFF00000000ull)) |
    //        ((static_cast<unsigned long long int>(y) << 16) & (0x00000000FFFF0000ull)) |
    //        ((static_cast<unsigned long long int>(z) << 0) & (0x000000000000FFFFull));
    return ((static_cast<unsigned long long int>(x) << 16) & (0x00000000FFFF0000ull)) |
        ((static_cast<unsigned long long int>(y) << 0) & (0x000000000000FFFFull));
}

inline unsigned long long int get_rgd_index_2D(const Eigen::Vector3d p, const Eigen::Vector2d b)
{
    int16_t x = static_cast<int16_t>(p.x() / b.x());
    int16_t y = static_cast<int16_t>(p.y() / b.y());
    // int16_t z = static_cast<int16_t>(p.z() / b.z());
    return get_index_2D(x, y);
}

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

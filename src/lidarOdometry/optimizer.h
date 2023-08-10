#pragma once

void optimize(std::vector<Point3Di>& intermediate_points, std::vector<Eigen::Affine3d>& intermediate_trajectory,
    std::vector<Eigen::Affine3d>& intermediate_trajectory_motion_model,
    NDT::GridParameters& rgd_params, NDTBucketMapType& buckets);

void update_rgd(NDT::GridParameters& rgd_params, NDTBucketMapType& buckets,
    std::vector<Point3Di>& points_global);

void align_to_reference(NDT::GridParameters& rgd_params, std::vector<Point3Di>& initial_points, Eigen::Affine3d& m_g, NDTBucketMapType& reference_buckets);
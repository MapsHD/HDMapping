#pragma once

#include <Eigen/Eigen>
#include <vector>

class PairWiseICP
{
public:
    PairWiseICP() = default;
    ~PairWiseICP() = default;

    bool compute(
        const std::vector<Eigen::Vector3d>& source,
        const std::vector<Eigen::Vector3d>& target,
        double search_radious,
        int number_of_iterations,
        Eigen::Affine3d& m_pose_result);
    bool compute_fast(
        const std::vector<Eigen::Vector3d>& source,
        const std::vector<Eigen::Vector3d>& target,
        double search_radious,
        int number_of_iterations,
        Eigen::Affine3d& m_pose_result,
        int dec);
};

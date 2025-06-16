#ifndef _PAIR_WISE_ITERATIVE_CLOSEST_POINT_H_
#define _PAIR_WISE_ITERATIVE_CLOSEST_POINT_H_

#include <vector>
#include <Eigen/Eigen>

class PairWiseICP{
public:
    PairWiseICP(){;};
    ~PairWiseICP() { ; };

    bool compute(const std::vector<Eigen::Vector3d> &source, const std::vector<Eigen::Vector3d> &target, double search_radious, int number_of_iterations, Eigen::Affine3d &m_pose_result);
    bool compute_fast(const std::vector<Eigen::Vector3d> &source, const std::vector<Eigen::Vector3d> &target, double search_radious, int number_of_iterations, Eigen::Affine3d &m_pose_result, int dec);
};

#endif
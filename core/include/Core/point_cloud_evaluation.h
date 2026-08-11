#ifndef _POINT_CLOUD_EVALUATION_H_
#define _POINT_CLOUD_EVALUATION_H_

#include <Eigen/Eigen>
#include <vector>

// skills
// tools https://deepwiki.com/fwilliams/point-cloud-utils/5.3-point-cloud-metrics
// https://arxiv.org/pdf/2604.18205

class PointCloudEvaluation
{
public:
    PointCloudEvaluation() = default;
    ~PointCloudEvaluation() = default;

    double ChamferDistance(const std::vector<Eigen::Vector3d>& cloud1, const std::vector<Eigen::Vector3d>& cloud2);
    double HausdorffDistance(const std::vector<Eigen::Vector3d>& cloud1, const std::vector<Eigen::Vector3d>& cloud2);
    double EarthMoverDistance(const std::vector<Eigen::Vector3d>& cloud1, const std::vector<Eigen::Vector3d>& cloud2);
    double Precision(const std::vector<Eigen::Vector3d>& cloud1, const std::vector<Eigen::Vector3d>& cloud2);
    double Recall(const std::vector<Eigen::Vector3d>& cloud1, const std::vector<Eigen::Vector3d>& cloud2);
    double F1Score(const std::vector<Eigen::Vector3d>& cloud1, const std::vector<Eigen::Vector3d>& cloud2);
};

#endif
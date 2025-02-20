#include <Eigen/Dense>

// this function draws ellipse for each bucket
void draw_ellipse(const Eigen::Matrix3d& covar, const Eigen::Vector3d& mean, Eigen::Vector3f color, float nstd);
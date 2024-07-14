#ifndef _LOCAL_SHAPE_FEATURES_H_
#define _LOCAL_SHAPE_FEATURES_H_

#include <Eigen/Eigen>
#include <vector>

class LocalShapeFeatures
{
public:
    struct PointWithLocalShapeFeatures
    {
        Eigen::Vector3d coordinates_global;
        bool valid;
        Eigen::Vector3d eigen_values;
        Eigen::Matrix3d eigen_vectors;
        Eigen::Vector3d normal_vector;
        double planarity;
        double cylindrical_likeness;
        double plane_likeness;
        double sphericity;
        double change_of_curvature;
        double omnivariance;
        double eigen_entropy;
    };

    struct Params{
        Eigen::Vector3d search_radious;
        double radious;
    };

    LocalShapeFeatures(){;};
    ~LocalShapeFeatures(){;};

    bool calculate_local_shape_features(std::vector<PointWithLocalShapeFeatures> &points, const Params &params);
};

#endif
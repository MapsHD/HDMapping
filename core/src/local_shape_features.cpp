#include <local_shape_features.h>
#include <hash_utils.h>
#include <iostream>
#include <execution>

bool LocalShapeFeatures::calculate_local_shape_features(std::vector<PointWithLocalShapeFeatures> &points, const Params &params)
{
    std::vector<std::pair<unsigned long long int, unsigned int>> indexes;

    for (int i = 0; i < points.size(); i++)
    {
        unsigned long long int index = get_rgd_index(points[i].coordinates_global, params.search_radious);
        indexes.emplace_back(index, i);
    }

    std::sort(indexes.begin(), indexes.end(),
              [](const std::pair<unsigned long long int, unsigned int> &a, const std::pair<unsigned long long int, unsigned int> &b)
              { return a.first < b.first; });

    std::unordered_map<unsigned long long int, std::pair<unsigned int, unsigned int>> buckets;

    for (unsigned int i = 0; i < indexes.size(); i++)
    {
        unsigned long long int index_of_bucket = indexes[i].first;
        if (buckets.contains(index_of_bucket))
        {
            buckets[index_of_bucket].second = i;
        }
        else
        {
            buckets[index_of_bucket].first = i;
            buckets[index_of_bucket].second = i;
        }
    }

    const auto hessian_fun = [&](const std::pair<unsigned long long int, unsigned int> &index)
    {
        int index_element_source = index.second; 
        Eigen::Vector3d source = points[index_element_source].coordinates_global;
        Eigen::Vector3d mean(0.0, 0.0, 0.0);
        Eigen::Matrix3d cov;
        int number_of_points_nn = 0;
        std::vector<Eigen::Vector3d> batch_of_points;

        for (double x = -params.search_radious.x(); x <= params.search_radious.x(); x += params.search_radious.x())
        {
            for (double y = -params.search_radious.y(); y <= params.search_radious.y(); y += params.search_radious.y())
            {
                for (double z = -params.search_radious.z(); z <= params.search_radious.z(); z += params.search_radious.z())
                {

                    Eigen::Vector3d position_global = source + Eigen::Vector3d(x, y, z);
                    unsigned long long int index_of_bucket = get_rgd_index(position_global, params.search_radious);

                    if (buckets.contains(index_of_bucket))
                    {
                        for (int index = buckets[index_of_bucket].first; index < buckets[index_of_bucket].second; index++)
                        {
                            int index_element_target = indexes[index].second;
                            Eigen::Vector3d target = points[index_element_target].coordinates_global;

                            if ((source - target).norm() < params.radious)
                            {
                                mean += target;
                                number_of_points_nn++;
                                batch_of_points.push_back(target);
                            }
                        }
                    }
                }
            }
        }
        batch_of_points.push_back(source);
        number_of_points_nn++;

        if (number_of_points_nn >= 3)
        {
            points[index_element_source].valid = true;
            mean /= batch_of_points.size();
            cov.setZero();

            for (int j = 0; j < batch_of_points.size(); j++)
            {
                cov(0, 0) += (mean.x() - batch_of_points[j].x()) * (mean.x() - batch_of_points[j].x());
                cov(0, 1) += (mean.x() - batch_of_points[j].x()) * (mean.y() - batch_of_points[j].y());
                cov(0, 2) += (mean.x() - batch_of_points[j].x()) * (mean.z() - batch_of_points[j].z());
                cov(1, 0) += (mean.y() - batch_of_points[j].y()) * (mean.x() - batch_of_points[j].x());
                cov(1, 1) += (mean.y() - batch_of_points[j].y()) * (mean.y() - batch_of_points[j].y());
                cov(1, 2) += (mean.y() - batch_of_points[j].y()) * (mean.z() - batch_of_points[j].z());
                cov(2, 0) += (mean.z() - batch_of_points[j].z()) * (mean.x() - batch_of_points[j].x());
                cov(2, 1) += (mean.z() - batch_of_points[j].z()) * (mean.y() - batch_of_points[j].y());
                cov(2, 2) += (mean.z() - batch_of_points[j].z()) * (mean.z() - batch_of_points[j].z());
            }

            cov /= batch_of_points.size();

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(cov, Eigen::ComputeEigenvectors);
            points[index_element_source].eigen_values = eigen_solver.eigenvalues();
            points[index_element_source].eigen_vectors = eigen_solver.eigenvectors();

            points[index_element_source].normal_vector = Eigen::Vector3d(points[index_element_source].eigen_vectors(0, 0), points[index_element_source].eigen_vectors(1, 0), points[index_element_source].eigen_vectors(2, 0));

            double ev1 = points[index_element_source].eigen_values.x();
            double ev2 = points[index_element_source].eigen_values.y();
            double ev3 = points[index_element_source].eigen_values.z();

            double sum_ev = ev1 + ev2 + ev3;

            points[index_element_source].planarity = 1 - ((3 * ev1 / sum_ev) * (3 * ev2 / sum_ev) * (3 * ev3 / sum_ev));
            points[index_element_source].cylindrical_likeness = (ev3 - ev2) / sum_ev;
            points[index_element_source].plane_likeness = 2 * (ev2 - ev1) / (sum_ev);

            points[index_element_source].sphericity = ev1 / ev3;
            points[index_element_source].change_of_curvature = ev3 / (sum_ev);
            points[index_element_source].omnivariance = std::cbrt(ev1 * ev2 * ev3);

            points[index_element_source].eigen_entropy = -ev1 * log(ev1) - ev2 * log(ev2) - ev3 * log(ev3);
        }
        else
        {
            points[index_element_source].valid = false;
        }
    };

    if (params.multithread)
    {
        std::for_each(std::execution::par_unseq, std::begin(indexes), std::end(indexes), hessian_fun);
    }
    else
    {
        std::for_each(std::begin(indexes), std::end(indexes), hessian_fun);
    }

    return true;
}
#include <surface.h>
#include <iostream>

#include <GL/freeglut.h>

#include <structures.h>
#include <transformations.h>
#include <../../3rdparty/observation_equations/codes/common/include/cauchy.h>
#include <../../3rdparty/observation_equations/codes/python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian.h>
#include <../../3rdparty/observation_equations/codes/python-scripts/constraints/relative_pose_tait_bryan_wc_jacobian.h>

#include <execution>

std::random_device rd;
std::mt19937 gen(rd());

inline double random(double low, double high)
{
    std::uniform_real_distribution<double> dist(low, high);
    return dist(gen);
}

void Surface::generate_initial_surface(const std::vector<Eigen::Vector3d> &point_cloud)
{
    std::cout << "Surface::generate_initial_surface" << std::endl;

    vertices.clear();
    vertices_odo.clear();
    // smoothness_indexes.clear();
    //  double surface_resolution = 0.5;
    number_rows = 0;
    number_cols = 0;
    odo_edges.clear();

    double min_x = 1000000.0;
    double max_x = -1000000.0;

    double min_y = 1000000.0;
    double max_y = -1000000.0;

    double min_z = 1000000.0;
    double max_z = -1000000.0;

    for (const auto &p : point_cloud)
    {
        if (p.x() < min_x)
        {
            min_x = p.x();
        }
        if (p.x() > max_x)
        {
            max_x = p.x();
        }

        if (p.y() < min_y)
        {
            min_y = p.y();
        }
        if (p.y() > max_y)
        {
            max_y = p.y();
        }

        if (p.z() < min_z)
        {
            min_z = p.z();
        }
        if (p.z() > max_z)
        {
            max_z = p.z();
        }
    }

    min_x -= 1.0;
    max_x += 1.0;

    min_y -= 1.0;
    max_y += 1.0;

    min_z -= 1.0;
    max_z += 1.0;

    std::cout << "min_x: " << min_x << " max_x: " << max_x << std::endl;
    std::cout << "min_y: " << min_y << " max_y: " << max_y << std::endl;
    std::cout << "min_z: " << min_z << " max_z: " << max_z << std::endl;

    for (double x = min_x; x <= max_x; x += surface_resolution)
    {
        for (double y = min_y; y <= max_y; y += surface_resolution)
        {
            // if (number_rows == 0)
            //{
            //     number_cols++;
            // }
            Eigen::Affine3d pose = Eigen::Affine3d::Identity();
            pose(0, 3) = x;
            pose(1, 3) = y;
            pose(2, 3) = min_z;
            vertices.push_back(pose);
            // std::cout << pose(0, 3) << " " << pose(1, 3) << " " << pose(2, 3) << std::endl;
        }
        // number_rows++;
    }

    // number_rows = floor(double(max_x - min_x) / surface_resolution);
    // number_cols = floor(double(max_y - min_y) / surface_resolution);
    number_rows = 0;
    for (double x = min_x; x <= max_x; x += surface_resolution)
    {
        number_rows++;
    }

    number_cols = 0;
    for (double y = min_y; y <= max_y; y += surface_resolution)
    {
        number_cols++;
    }

    std::cout << "number_rows: " << number_rows << " number_cols: " << number_cols << std::endl;

    vertices_odo = vertices;

    for (int row = 0; row < number_rows; row++)
    {
        for (int col = 0; col < number_cols; col++)
        {
            int index = col + row * number_cols;

            if (col + 1 < number_cols)
            {
                int index_next_col = col + 1 + row * number_cols;
                odo_edges.emplace_back(index, index_next_col);
            }

            if (row + 1 < number_rows)
            {
                int index_next_row = col + (row + 1) * number_cols;
                odo_edges.emplace_back(index, index_next_row);
            }
        }
    }
}

void Surface::render()
{
    glColor3f(0.3, 0.3, 0.3);
    for (size_t i = 0; i < odo_edges.size(); i++)
    {
        glBegin(GL_LINES);
        glVertex3f(vertices[odo_edges[i].first](0, 3), vertices[odo_edges[i].first](1, 3), vertices[odo_edges[i].first](2, 3));
        glVertex3f(vertices[odo_edges[i].second](0, 3), vertices[odo_edges[i].second](1, 3), vertices[odo_edges[i].second](2, 3));
        glEnd();
    }
}

void Surface::align_surface_to_ground_points(const std::vector<Eigen::Vector3d> &point_cloud)
{
    std::cout << "Surface::align_surface_to_ground_points" << std::endl;
    std::vector<Eigen::Triplet<double>> tripletListA;
    std::vector<Eigen::Triplet<double>> tripletListP;
    std::vector<Eigen::Triplet<double>> tripletListB;

    std::vector<TaitBryanPose> poses;
    std::vector<TaitBryanPose> poses_odo;

    for (size_t i = 0; i < vertices.size(); i++)
    {
        poses.push_back(pose_tait_bryan_from_affine_matrix(vertices[i]));
    }
    for (size_t i = 0; i < vertices_odo.size(); i++)
    {
        poses_odo.push_back(pose_tait_bryan_from_affine_matrix(vertices_odo[i]));
    }

    for (size_t i = 0; i < odo_edges.size(); i++)
    {
        Eigen::Matrix<double, 6, 1> relative_pose_measurement_odo;
        relative_pose_tait_bryan_wc_case1(relative_pose_measurement_odo,
                                          poses_odo[odo_edges[i].first].px,
                                          poses_odo[odo_edges[i].first].py,
                                          poses_odo[odo_edges[i].first].pz,
                                          poses_odo[odo_edges[i].first].om,
                                          poses_odo[odo_edges[i].first].fi,
                                          poses_odo[odo_edges[i].first].ka,
                                          poses_odo[odo_edges[i].second].px,
                                          poses_odo[odo_edges[i].second].py,
                                          poses_odo[odo_edges[i].second].pz,
                                          poses_odo[odo_edges[i].second].om,
                                          poses_odo[odo_edges[i].second].fi,
                                          poses_odo[odo_edges[i].second].ka);

        Eigen::Matrix<double, 6, 1> delta;
        relative_pose_obs_eq_tait_bryan_wc_case1(
            delta,
            poses[odo_edges[i].first].px,
            poses[odo_edges[i].first].py,
            poses[odo_edges[i].first].pz,
            poses[odo_edges[i].first].om,
            poses[odo_edges[i].first].fi,
            poses[odo_edges[i].first].ka,
            poses[odo_edges[i].second].px,
            poses[odo_edges[i].second].py,
            poses[odo_edges[i].second].pz,
            poses[odo_edges[i].second].om,
            poses[odo_edges[i].second].fi,
            poses[odo_edges[i].second].ka,
            relative_pose_measurement_odo(0, 0),
            relative_pose_measurement_odo(1, 0),
            relative_pose_measurement_odo(2, 0),
            relative_pose_measurement_odo(3, 0),
            relative_pose_measurement_odo(4, 0),
            relative_pose_measurement_odo(5, 0));

        Eigen::Matrix<double, 6, 12, Eigen::RowMajor> jacobian;
        relative_pose_obs_eq_tait_bryan_wc_case1_jacobian(jacobian,
                                                          poses[odo_edges[i].first].px,
                                                          poses[odo_edges[i].first].py,
                                                          poses[odo_edges[i].first].pz,
                                                          poses[odo_edges[i].first].om,
                                                          poses[odo_edges[i].first].fi,
                                                          poses[odo_edges[i].first].ka,
                                                          poses[odo_edges[i].second].px,
                                                          poses[odo_edges[i].second].py,
                                                          poses[odo_edges[i].second].pz,
                                                          poses[odo_edges[i].second].om,
                                                          poses[odo_edges[i].second].fi,
                                                          poses[odo_edges[i].second].ka);

        int ir = tripletListB.size();

        int ic_1 = odo_edges[i].first * 6;
        int ic_2 = odo_edges[i].second * 6;

        for (size_t row = 0; row < 6; row++)
        {
            tripletListA.emplace_back(ir + row, ic_1, -jacobian(row, 0));
            tripletListA.emplace_back(ir + row, ic_1 + 1, -jacobian(row, 1));
            tripletListA.emplace_back(ir + row, ic_1 + 2, -jacobian(row, 2));
            tripletListA.emplace_back(ir + row, ic_1 + 3, -jacobian(row, 3));
            tripletListA.emplace_back(ir + row, ic_1 + 4, -jacobian(row, 4));
            tripletListA.emplace_back(ir + row, ic_1 + 5, -jacobian(row, 5));

            tripletListA.emplace_back(ir + row, ic_2, -jacobian(row, 6));
            tripletListA.emplace_back(ir + row, ic_2 + 1, -jacobian(row, 7));
            tripletListA.emplace_back(ir + row, ic_2 + 2, -jacobian(row, 8));
            tripletListA.emplace_back(ir + row, ic_2 + 3, -jacobian(row, 9));
            tripletListA.emplace_back(ir + row, ic_2 + 4, -jacobian(row, 10));
            tripletListA.emplace_back(ir + row, ic_2 + 5, -jacobian(row, 11));
        }

        tripletListB.emplace_back(ir, 0, delta(0, 0));
        tripletListB.emplace_back(ir + 1, 0, delta(1, 0));
        tripletListB.emplace_back(ir + 2, 0, delta(2, 0));
        tripletListB.emplace_back(ir + 3, 0, delta(3, 0));
        tripletListB.emplace_back(ir + 4, 0, delta(4, 0));
        tripletListB.emplace_back(ir + 5, 0, delta(5, 0));

        tripletListP.emplace_back(ir, ir, 100);
        tripletListP.emplace_back(ir + 1, ir + 1, 100);
        tripletListP.emplace_back(ir + 2, ir + 2, 100);
        tripletListP.emplace_back(ir + 3, ir + 3, 100);
        tripletListP.emplace_back(ir + 4, ir + 4, 100);
        tripletListP.emplace_back(ir + 5, ir + 5, 100);
    }

    for (size_t i = 0; i < vertices.size(); i++)
    {
        TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(vertices[i]);

        Eigen::Vector3d p_t; //(georeference_data[i].first(0,3), georeference_data[i].first(1,3), georeference_data[i].first(2,3));
        if (!find_nearest_neighbour(p_t, vertices[i], point_cloud))
        {
            continue;
        }

        int ir = tripletListB.size();

        Eigen::Vector3d p_s(0, 0, 0);
        double delta_x;
        double delta_y;
        double delta_z;
        point_to_point_source_to_target_tait_bryan_wc(delta_x, delta_y, delta_z, pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka, p_s.x(), p_s.y(), p_s.z(), p_t.x(), p_t.y(), p_t.z());

        Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
        point_to_point_source_to_target_tait_bryan_wc_jacobian(jacobian, pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka, p_s.x(), p_s.y(), p_s.z());

        int ic = i * 6;
        tripletListA.emplace_back(ir, ic + 0, -jacobian(0, 0));
        tripletListA.emplace_back(ir + 1, ic + 1, -jacobian(1, 1));
        tripletListA.emplace_back(ir + 2, ic + 2, -jacobian(2, 2));

        tripletListP.emplace_back(ir, ir, 1);
        tripletListP.emplace_back(ir + 1, ir + 1, 1);
        tripletListP.emplace_back(ir + 2, ir + 2, 1);

        if (robust_kernel){
            tripletListB.emplace_back(ir,  0,    cauchy (delta_x, 1));
            tripletListB.emplace_back(ir + 1, 0, cauchy (delta_y, 1));
            tripletListB.emplace_back(ir + 2, 0, cauchy (delta_z, 1));
        }else{
            tripletListB.emplace_back(ir, 0, delta_x);
            tripletListB.emplace_back(ir + 1, 0, delta_y);
            tripletListB.emplace_back(ir + 2, 0, delta_z);
        }
    }

    Eigen::SparseMatrix<double> matA(tripletListB.size(), vertices.size() * 6);
    Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
    Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

    matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
    matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
    matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

    Eigen::SparseMatrix<double> AtPA(vertices.size() * 6, vertices.size() * 6);
    Eigen::SparseMatrix<double> AtPB(vertices.size() * 6, 1);

    {
        Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
        AtPA = (AtP)*matA;
        AtPB = (AtP)*matB;
    }

    tripletListA.clear();
    tripletListP.clear();
    tripletListB.clear();

    // Eigen::SparseMatrix<double> AtPA_I(vertices.size() * 6, vertices.size() * 6);
    // AtPA_I.setIdentity();
    // AtPA += AtPA_I;

    std::cout << "AtPA.size: " << AtPA.size() << std::endl;
    std::cout << "AtPB.size: " << AtPB.size() << std::endl;

    std::cout << "start solving AtPA=AtPB" << std::endl;
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);

    std::cout << "x = solver.solve(AtPB)" << std::endl;
    Eigen::SparseMatrix<double> x = solver.solve(AtPB);

    std::vector<double> h_x;

    for (int k = 0; k < x.outerSize(); ++k)
    {
        for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
        {
            h_x.push_back(it.value());
        }
    }

    std::cout << "h_x.size(): " << h_x.size() << std::endl;

    std::cout << "AtPA=AtPB SOLVED" << std::endl;

    // for(size_t i = 0 ; i < h_x.size(); i++){
    //	std::cout << h_x[i] << std::endl;
    // }

    if (h_x.size() == 6 * vertices.size())
    {
        int counter = 0;

        for (size_t i = 0; i < vertices.size(); i++)
        {
            TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(vertices[i]);
            pose.px += h_x[counter++];
            pose.py += h_x[counter++];
            pose.pz += h_x[counter++];
            pose.om += h_x[counter++];
            pose.fi += h_x[counter++];
            pose.ka += h_x[counter++];
            vertices[i] = affine_matrix_from_pose_tait_bryan(pose);
        }
        std::cout << "optimizing with tait bryan finished" << std::endl;
    }
    else
    {
        std::cout << "optimizing with tait bryan FAILED" << std::endl;
        std::cout << "h_x.size(): " << h_x.size() << " 6 * vertices.size(): " << 6 * vertices.size() << std::endl;
    }
}

bool Surface::find_nearest_neighbour(Eigen::Vector3d &p_t, Eigen::Affine3d vertex, const std::vector<Eigen::Vector3d> &reference_points)
{
    double min_dist_xy = 1000000.0;
    double search_radious = 1.0;
    bool found = false;

    for (size_t i = 0; i < reference_points.size(); i++)
    {
        float dist = sqrt((vertex(0, 3) - reference_points[i].x()) * (vertex(0, 3) - reference_points[i].x()) + (vertex(1, 3) - reference_points[i].y()) * (vertex(1, 3) - reference_points[i].y()));

        if (dist < search_radious)
        {
            if (dist < min_dist_xy)
            {
                min_dist_xy = dist;
                p_t = reference_points[i];
                p_t.x() = vertex(0, 3) + random(-0.000001, 0.000001);
                p_t.y() = vertex(1, 3) + random(-0.000001, 0.000001);
                found = true;
            }
        }
    }
    return found;
}

std::vector<int> Surface::get_filtered_indexes(const std::vector<Eigen::Vector3d> &pc, const std::vector<int> &lowest_points_indexes, Eigen::Vector2d bucket_dim_xy)
{
    bool multithread = true;
    std::vector<int> out_indexes;

    std::vector<std::pair<Eigen::Vector3d, bool>> lowest_points;
    for (int i = 0; i < lowest_points_indexes.size(); i++)
    {
        lowest_points.emplace_back(pc[lowest_points_indexes[i]], false);
    }

    std::vector<std::pair<unsigned long long int, unsigned int>> indexes;

    for (int i = 0; i < lowest_points.size(); i++)
    {
        unsigned long long int index = get_rgd_index_2D(lowest_points[i].first, bucket_dim_xy);
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
        Eigen::Vector3d source = lowest_points[index_element_source].first;
        Eigen::Vector3d mean(0.0, 0.0, 0.0);
        Eigen::Matrix3d cov;
        int number_of_points_nn = 0;
        std::vector<Eigen::Vector3d> batch_of_points;

        for (double x = -bucket_dim_xy.x(); x <= bucket_dim_xy.x(); x += bucket_dim_xy.x())
        {
            for (double y = -bucket_dim_xy.y(); y <= bucket_dim_xy.y(); y += bucket_dim_xy.y())
            {
                // for (double z = -params.search_radious.z(); z <= params.search_radious.z(); z += params.search_radious.z())
                //{

                Eigen::Vector3d position_global = source + Eigen::Vector3d(x, y, 0);
                unsigned long long int index_of_bucket = get_rgd_index_2D(position_global, bucket_dim_xy);

                if (buckets.contains(index_of_bucket))
                {
                    for (int index = buckets[index_of_bucket].first; index < buckets[index_of_bucket].second; index++)
                    {
                        int index_element_target = indexes[index].second;
                        Eigen::Vector3d target = lowest_points[index_element_target].first;

                        if (fabs(source.z() - target.z()) < 3 * z_sigma_threshold)
                        // if ((source - target).norm() < params.radious)
                        {
                            mean += target;
                            number_of_points_nn++;
                            batch_of_points.push_back(target);
                        }
                    }
                }
                //}
            }
        }
        batch_of_points.push_back(source);
        number_of_points_nn++;

        if (number_of_points_nn >= 3)
        {
            // points[index_element_source].valid = true;
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

            // std::cout <<
            if (sqrt(cov(2, 2)) < z_sigma_threshold)
            {
                lowest_points[index_element_source].second = true;
            }

            // std::cout << "cov " << cov << std::endl;
            //  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(cov, Eigen::ComputeEigenvectors);
            //  points[index_element_source].eigen_values = eigen_solver.eigenvalues();
            //  points[index_element_source].eigen_vectors = eigen_solver.eigenvectors();

            // points[index_element_source].normal_vector = Eigen::Vector3d(points[index_element_source].eigen_vectors(0, 0), points[index_element_source].eigen_vectors(1, 0), points[index_element_source].eigen_vectors(2, 0));

            // double ev1 = points[index_element_source].eigen_values.x();
            // double ev2 = points[index_element_source].eigen_values.y();
            // double ev3 = points[index_element_source].eigen_values.z();

            // double sum_ev = ev1 + ev2 + ev3;

            // points[index_element_source].planarity = 1 - ((3 * ev1 / sum_ev) * (3 * ev2 / sum_ev) * (3 * ev3 / sum_ev));
            // points[index_element_source].cylindrical_likeness = (ev3 - ev2) / sum_ev;
            // points[index_element_source].plane_likeness = 2 * (ev2 - ev1) / (sum_ev);

            // points[index_element_source].sphericity = ev1 / ev3;
            // points[index_element_source].change_of_curvature = ev3 / (sum_ev);
            // points[index_element_source].omnivariance = std::cbrt(ev1 * ev2 * ev3);

            // points[index_element_source].eigen_entropy = -ev1 * log(ev1) - ev2 * log(ev2) - ev3 * log(ev3);
        }
        else
        {
            // points[index_element_source].valid = false;
        }
    };

    if (multithread)
    {
        std::for_each(std::execution::par_unseq, std::begin(indexes), std::end(indexes), hessian_fun);
    }
    else
    {
        std::for_each(std::begin(indexes), std::end(indexes), hessian_fun);
    }

    for (int i = 0; i < lowest_points_indexes.size(); i++)
    {
        if (lowest_points[i].second)
        {
            out_indexes.push_back(lowest_points_indexes[i]);
        }
    }

    //

    // for (const auto &p : lowest_points){
    // std::cout << (int)p.second;
    //}

    /*std::vector<std::pair<unsigned long long int, unsigned int>> indexes;

    for (int i = 0; i < lowest_points.size(); i++)
    {
        unsigned long long int index = get_rgd_index_2D(lowest_points[i], bucket_dim_xy);
        indexes.emplace_back(index, i);
    }*/

    return out_indexes;
}

std::vector<Eigen::Vector3d> Surface::get_points_without_surface(const std::vector<Eigen::Vector3d> &points, double distance_to_ground_threshold_bottom,
                                                                 double distance_to_ground_threshold_up, Eigen::Vector2d bucket_dim_xy)
{
    bool multithread = true;

    std::vector<Eigen::Vector3d> out_points;
    std::vector<bool> to_remove;

    for (int i = 0; i < points.size(); i++)
    {
        to_remove.push_back(false);
    }

    ////////////////////////////////////////////

    // std::vector<int> out_indexes;

    std::vector<std::pair<unsigned long long int, unsigned int>> indexes;

    for (int i = 0; i < vertices.size(); i++)
    {
        unsigned long long int index = get_rgd_index_2D(vertices[i].translation(), bucket_dim_xy);
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

    for (int i = 0; i < points.size(); i++)
    {
        unsigned long long int index_of_bucket = get_rgd_index_2D(points[i], bucket_dim_xy);
        if (buckets.contains(index_of_bucket))
        {
            int index = buckets[index_of_bucket].first;

            int index_element_target = indexes[index].second;
            double z_ground = vertices[index_element_target].translation().z();

            if (points[i].z() < z_ground + distance_to_ground_threshold_bottom ||
                points[i].z() > z_ground + distance_to_ground_threshold_up)
            {
                to_remove[i] = true;
                // std::cout << vertices[index_element_target].translation().x() << " " << vertices[index_element_target].translation().y() << " "
                //          << vertices[index_element_target].translation().z() << " " << points[i].x() << " " << points[i].y() << " " << points[i].z() << std::endl;
            }
        }
    }

    //////////////////////////////////
    for (int i = 0; i < points.size(); i++)
    {
        if (!to_remove[i])
        {
            out_points.push_back(points[i]);
        }
    }

    return out_points;
}
#include <Core/export_laz.h>
#include <Core/session.h>
#include <Eigen/SVD>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-point-metrics/point_to_point_tait_bryan_wc_jacobian.h>
#include <stdexcept>
#include <vector>

std::random_device rd;
std::mt19937 gen(rd());

inline double random(double low, double high)
{
    std::uniform_real_distribution<double> dist(low, high);
    return dist(gen);
}

inline double cauchy(double delta, double b)
{
    return 1.0 / (M_PI * b * (1.0 + ((delta) / b) * ((delta) / b)));
}

Eigen::Affine3d find_rigid_transform(
    const std::vector<std::pair<double, Eigen::Affine3d>>& target_trajectory,
    const std::vector<std::pair<double, Eigen::Affine3d>>& source_trajectory)
{
    Eigen::Affine3d source_to_target = Eigen::Affine3d::Identity();

    std::vector<std::pair<int, int>> correspondences;

    for (int i = 0; i < source_trajectory.size(); ++i)
    {
        double source_timestamp = source_trajectory[i].first;
        double min_time_diff = std::numeric_limits<double>::max();
        int best_match_index = -1;

        // Use lower_bound with a lambda comparator
        auto it = std::lower_bound(
            target_trajectory.begin(),
            target_trajectory.end(),
            source_timestamp,
            [](const std::pair<double, Eigen::Affine3d>& a, double timestamp)
            {
                return a.first < timestamp;
            });

        if (it != target_trajectory.end())
        {
            if (i % 1000 == 0)
            {
                std::cout << "Element " << source_timestamp << " found at index " << std::distance(target_trajectory.begin(), it) << "\n";
            }

            correspondences.emplace_back(i, std::distance(target_trajectory.begin(), it));
        }
        else
        {
            std::cout << "Element " << source_timestamp << " not found.\n";
        }
    }

    if (correspondences.size() < 3)
    {
        throw std::runtime_error("Not enough correspondences found for rigid registration.");
    }

    std::cout << "Found " << correspondences.size() << " correspondences." << std::endl;

    int number_of_iterations = 100;
    for (int iter = 0; iter < number_of_iterations; ++iter)
    {
        std::cout << "ICP iteration: " << iter + 1 << " of " << number_of_iterations << std::endl;

        std::vector<Eigen::Triplet<double>> tripletListA;
        std::vector<Eigen::Triplet<double>> tripletListP;
        std::vector<Eigen::Triplet<double>> tripletListB;

        TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(source_to_target);

        double rmse = 0.0;
        for (int nn = 0; nn < correspondences.size(); ++nn)
        {
            int nn_source_local = correspondences[nn].first;
            int nn_target_global = correspondences[nn].second;

            Eigen::Vector3d p_s(
                source_trajectory[nn_source_local].second(0, 3),
                source_trajectory[nn_source_local].second(1, 3),
                source_trajectory[nn_source_local].second(2, 3));
            Eigen::Vector3d p_t(
                target_trajectory[nn_target_global].second(0, 3),
                target_trajectory[nn_target_global].second(1, 3),
                target_trajectory[nn_target_global].second(2, 3));

            double delta_x;
            double delta_y;
            double delta_z;
            point_to_point_source_to_target_tait_bryan_wc(
                delta_x,
                delta_y,
                delta_z,
                pose_s.px,
                pose_s.py,
                pose_s.pz,
                pose_s.om,
                pose_s.fi,
                pose_s.ka,
                p_s.x(),
                p_s.y(),
                p_s.z(),
                p_t.x(),
                p_t.y(),
                p_t.z());

            Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
            point_to_point_source_to_target_tait_bryan_wc_jacobian(
                jacobian, pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka, p_s.x(), p_s.y(), p_s.z());

            int ir = tripletListB.size();

            for (int row = 0; row < 3; row++)
            {
                for (int col = 0; col < 6; col++)
                {
                    if (jacobian(row, col) != 0.0)
                    {
                        tripletListA.emplace_back(ir + row, col, -jacobian(row, col));
                    }
                }
            }

            // tripletListP.emplace_back(ir, ir, cauchy(delta_x, 1));
            // tripletListP.emplace_back(ir + 1, ir + 1, cauchy(delta_y, 1));
            // tripletListP.emplace_back(ir + 2, ir + 2, cauchy(delta_z, 1));
            tripletListP.emplace_back(ir, ir, 1);
            tripletListP.emplace_back(ir + 1, ir + 1, 1);
            tripletListP.emplace_back(ir + 2, ir + 2, 1);

            tripletListB.emplace_back(ir, 0, delta_x);
            tripletListB.emplace_back(ir + 1, 0, delta_y);
            tripletListB.emplace_back(ir + 2, 0, delta_z);

            rmse += sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
        }

        rmse /= correspondences.size();
        std::cout << "RMSE: " << rmse << std::endl;

        Eigen::SparseMatrix<double> matA(tripletListB.size(), 6);
        Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
        Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

        matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
        matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
        matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

        Eigen::SparseMatrix<double> AtPA(6, 6);
        Eigen::SparseMatrix<double> AtPB(6, 1);

        Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
        AtPA = AtP * matA;
        AtPB = AtP * matB;

        // Create an n x n sparse matrix of doubles
        // Eigen::SparseMatrix<double> I(6, 6);
        // Set it to identity
        // I.setIdentity();
        // AtPA += I;

        tripletListA.clear();
        tripletListP.clear();
        tripletListB.clear();

        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);
        Eigen::SparseMatrix<double> x = solver.solve(AtPB);

        std::vector<double> h_x;

        for (int k = 0; k < x.outerSize(); ++k)
        {
            for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            {
                h_x.push_back(it.value());
            }
        }

        if (h_x.size() == 6)
        {
            std::cout << "ICP solution" << std::endl;
            std::cout << "x,y,z,om,fi,ka" << std::endl;

            std::cout << h_x[0] << "," << h_x[1] << "," << h_x[2] << "," << h_x[3] << "," << h_x[4] << "," << h_x[5] << std::endl;

            int counter = 0;
            pose_s.px += h_x[counter++];
            pose_s.py += h_x[counter++];
            pose_s.pz += h_x[counter++];
            pose_s.om += h_x[counter++];
            pose_s.fi += h_x[counter++];
            pose_s.ka += h_x[counter++];

            source_to_target = affine_matrix_from_pose_tait_bryan(pose_s);
        }
        else
        {
            std::cout << "AtPA=AtPB FAILED" << std::endl;
            return source_to_target;
        }
    } // for(int iter = 0; iter < number_of_iterations; ++iter)

    return source_to_target;
}

int main(int argc, char* argv[])
{
    std::cout << "Session to Session Rigid Registration" << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "This program saves the source session's global point cloud to a LAZ file." << std::endl;
    std::cout << "Trajectories are saved to sanity_check_trg.txt and sanity_check_src.txt for sanity check." << std::endl;

    if (argc < 5)
    {
        std::cerr << "Usage: " << argv[0] << " <session_target (ground_truth)> <session_source> <output_laz> <downsampling bucket size>"
                  << std::endl;
        return 1;
    }

    Session session_target;
    if (!session_target.load(argv[1], true, atof(argv[4]), atof(argv[4]), atof(argv[4]), false))
    {
        std::cerr << "Error loading session target." << std::endl;
        return 1;
    }

    Session session_source;
    if (!session_source.load(argv[2], true, atof(argv[4]), atof(argv[4]), atof(argv[4]), false))
    {
        std::cerr << "Error loading session source." << std::endl;
        return 1;
    }

    std::cout << "Loaded sessions successfully." << std::endl;
    std::cout << "Performing rigid registration..." << std::endl;

    std::vector<std::pair<double, Eigen::Affine3d>> target_trajectory_global;
    std::vector<std::pair<double, Eigen::Affine3d>> source_trajectory_global;

    for (const auto& pcs : session_target.point_clouds_container.point_clouds)
    {
        for (const auto& trj_node : pcs.local_trajectory)
        {
            target_trajectory_global.push_back(std::make_pair(trj_node.timestamps.first, pcs.m_pose * trj_node.m_pose));
        }
    }

    for (const auto& pcs : session_source.point_clouds_container.point_clouds)
    {
        for (const auto& trj_node : pcs.local_trajectory)
        {
            source_trajectory_global.push_back(std::make_pair(trj_node.timestamps.first, pcs.m_pose * trj_node.m_pose));
        }
    }

    std::ofstream trajectory_file_trg("sanity_check_trg.txt");
    if (!trajectory_file_trg)
    {
        std::cerr << "Error opening output trajectory file: trg.txt" << std::endl;
        return 1;
    }

    trajectory_file_trg << std::setprecision(17);
    for (const auto& trajectory_node : target_trajectory_global)
    {
        trajectory_file_trg << trajectory_node.second.translation().x() << ' ' << trajectory_node.second.translation().y() << ' '
                            << trajectory_node.second.translation().z() << std::endl;
    }

    Eigen::Affine3d rigid_transform = Eigen::Affine3d::Identity();
    try
    {
        rigid_transform = find_rigid_transform(target_trajectory_global, source_trajectory_global);
    } catch (const std::exception& error)
    {
        std::cerr << "Rigid registration failed: " << error.what() << std::endl;
        return 1;
    }

    for (int i = 0; i < source_trajectory_global.size(); ++i)
    {
        source_trajectory_global[i].second = rigid_transform * source_trajectory_global[i].second;
    }

    std::ofstream trajectory_file_src("sanity_check_src.txt");
    if (!trajectory_file_src)
    {
        std::cerr << "Error opening output trajectory file: src.txt" << std::endl;
        return 1;
    }

    trajectory_file_src << std::setprecision(17);
    for (const auto& trajectory_node : source_trajectory_global)
    {
        trajectory_file_src << trajectory_node.second.translation().x() << ' ' << trajectory_node.second.translation().y() << ' '
                            << trajectory_node.second.translation().z() << std::endl;
    }

    std::cout << "Applying rigid transform to source session's point clouds..." << std::endl;
    std::cout << "Rigid transform: " << std::endl;
    std::cout << rigid_transform.matrix() << std::endl;

    for (auto& pcs : session_source.point_clouds_container.point_clouds)
    {
        pcs.m_pose = rigid_transform * pcs.m_pose;
    }

    std::cout << "Saving source session's global point cloud to LAZ file: " << argv[3] << std::endl;

    save_all_to_las(session_source, argv[3], false, false);

    std::cout << "Rigid registration completed successfully. Output saved to: " << argv[3] << std::endl;

    return 0;
}
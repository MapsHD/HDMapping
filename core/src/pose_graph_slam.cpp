#include <pose_graph_slam.h>
#include <iostream>
#include <python-scripts/constraints/relative_pose_tait_bryan_wc_jacobian.h>
#include <python-scripts/constraints/relative_pose_tait_bryan_cw_jacobian.h>

#include <python-scripts/constraints/relative_pose_rodrigues_wc_jacobian.h>
//#include <python-scripts/constraints/relative_pose_rodrigues_cw_jacobian.h>

#include <python-scripts/constraints/relative_pose_quaternion_wc_jacobian.h>
#include <python-scripts/constraints/relative_pose_quaternion_cw_jacobian.h>
#include <python-scripts/constraints/quaternion_constraint_jacobian.h>

#include <transformations.h>

#include <ndt.h>
#include <registration_plane_feature.h>

#ifdef WITH_PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#endif

#if WITH_GTSAM
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#endif

#if WITH_MANIF
#include <manif/SE3.h>
#endif

#include <m_estimators.h>

std::random_device rd;
std::mt19937 gen(rd());

inline double random(double low, double high)
{
    std::uniform_real_distribution<double> dist(low, high);
    return dist(gen);
}

void PoseGraphSLAM::add_random_noise(PointClouds& point_clouds_container) {
    for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++) {
        TaitBryanPose pose;
        pose.px = random(-0.000001, 0.000001); 
        pose.py = random(-0.000001, 0.000001);
        pose.pz = random(-0.000001, 0.000001);
        pose.om = random(-0.000001, 0.000001);
        pose.fi = random(-0.000001, 0.000001);
        pose.ka = random(-0.000001, 0.000001);
        Eigen::Affine3d m = affine_matrix_from_pose_tait_bryan(pose);
        point_clouds_container.point_clouds[i].m_pose = point_clouds_container.point_clouds[i].m_pose * m;
    }
}

void add_noise_to_poses(std::vector<TaitBryanPose>& poses) {
    for (size_t i = 0; i < poses.size(); i++) {
        poses[i].px += random(-0.000001, 0.000001);
        poses[i].py += random(-0.000001, 0.000001);
        poses[i].pz += random(-0.000001, 0.000001);
        poses[i].om += random(-0.000001, 0.000001);
        poses[i].fi += random(-0.000001, 0.000001);
        poses[i].ka += random(-0.000001, 0.000001);
    }
}

void add_noise_to_poses(std::vector<RodriguesPose>& poses) {
    for (size_t i = 0; i < poses.size(); i++) {
        TaitBryanPose pose;
        pose.px = random(-0.000001, 0.000001);
        pose.py = random(-0.000001, 0.000001);
        pose.pz = random(-0.000001, 0.000001);
        pose.om = random(-0.000001, 0.000001);
        pose.fi = random(-0.000001, 0.000001);
        pose.ka = random(-0.000001, 0.000001);
        Eigen::Affine3d m = affine_matrix_from_pose_tait_bryan(pose);

        Eigen::Affine3d mr = affine_matrix_from_pose_rodrigues(poses[i]);
        mr = mr * m;
        poses[i] = pose_rodrigues_from_affine_matrix(mr);
    }
}

void add_noise_to_poses(std::vector<QuaternionPose>& poses) {
    for (size_t i = 0; i < poses.size(); i++) {
        TaitBryanPose pose;
        pose.px = random(-0.000001, 0.000001);
        pose.py = random(-0.000001, 0.000001);
        pose.pz = random(-0.000001, 0.000001);
        pose.om = random(-0.000001, 0.000001);
        pose.fi = random(-0.000001, 0.000001);
        pose.ka = random(-0.000001, 0.000001);
        Eigen::Affine3d m = affine_matrix_from_pose_tait_bryan(pose);

        Eigen::Affine3d mr = affine_matrix_from_pose_quaternion(poses[i]);
        mr = mr * m;
        poses[i] = pose_quaternion_from_affine_matrix(mr);
    }
}

bool PoseGraphSLAM::optimize(PointClouds& point_clouds_container)
{
    add_random_noise(point_clouds_container);
    edges.clear();

    int number_of_unknowns = 6;
    if (is_quaternion)number_of_unknowns = 7;

    for (auto& pc : point_clouds_container.point_clouds) {
        pc.build_rgd();
        pc.cout_rgd();
        pc.compute_normal_vectors(0.5);
    }

    //get edges based on overlap
    auto min_overlap = std::numeric_limits<double>::max();
    auto max_overlap = std::numeric_limits<double>::lowest();

    for (int i = 0; i < point_clouds_container.point_clouds.size(); i++) {
        for (int j = i + 1; j < point_clouds_container.point_clouds.size(); j++) {
            std::vector<std::pair<int, int>> nns = point_clouds_container.point_clouds[i].nns(point_clouds_container.point_clouds[j], 0.5);

            double overlap = double(nns.size()) / double(point_clouds_container.point_clouds[i].points_local.size());
            std::cout << "overlap: " << overlap << " between " << i << "," << j << std::endl;
            if (overlap > overlap_threshold) {
                Edge edge;
                edge.index_from = i;
                edge.index_to = j;
                edges.push_back(edge);
            }

            if (overlap < min_overlap) {
                min_overlap = overlap;
            }
            if (overlap > max_overlap) {
                max_overlap = overlap;
            }
        }
    }

    std::cout << "min_overlap: " << min_overlap * 100 << "[%]" << std::endl;
    std::cout << "max_overlap: " << max_overlap * 100 << "[%]" << std::endl;


    calculate_edges(point_clouds_container.point_clouds);

    //graph slam
    bool is_ok = true;
    std::vector<Eigen::Affine3d> m_poses;
    for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++) {
        m_poses.push_back(point_clouds_container.point_clouds[i].m_pose);
    }

    if (is_tait_bryan_angles) {
        std::vector<TaitBryanPose> poses;

        for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++) {
            if (is_wc) {
                poses.push_back(pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose));
            }
            else if (is_cw) {
                poses.push_back(pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose.inverse()));
            }
        }

        for (int iter = 0; iter < iterations; iter++) {
            
            add_noise_to_poses(poses);

            std::vector<Eigen::Triplet<double>> tripletListA;
            std::vector<Eigen::Triplet<double>> tripletListP;
            std::vector<Eigen::Triplet<double>> tripletListB;

            for (size_t i = 0; i < edges.size(); i++) {
                Eigen::Matrix<double, 6, 1> delta;
                Eigen::Matrix<double, 6, 12, Eigen::RowMajor> jacobian;
                auto relative_pose = pose_tait_bryan_from_affine_matrix(edges[i].m_relative_pose);
                if (is_wc) {
                    relative_pose_obs_eq_tait_bryan_wc_case1(
                        delta,
                        poses[edges[i].index_from].px,
                        poses[edges[i].index_from].py,
                        poses[edges[i].index_from].pz,
                        normalize_angle(poses[edges[i].index_from].om),
                        normalize_angle(poses[edges[i].index_from].fi),
                        normalize_angle(poses[edges[i].index_from].ka),
                        poses[edges[i].index_to].px,
                        poses[edges[i].index_to].py,
                        poses[edges[i].index_to].pz,
                        normalize_angle(poses[edges[i].index_to].om),
                        normalize_angle(poses[edges[i].index_to].fi),
                        normalize_angle(poses[edges[i].index_to].ka),
                        relative_pose.px,
                        relative_pose.py,
                        relative_pose.pz,
                        normalize_angle(relative_pose.om),
                        normalize_angle(relative_pose.fi),
                        normalize_angle(relative_pose.ka));
                    relative_pose_obs_eq_tait_bryan_wc_case1_jacobian(jacobian,
                        poses[edges[i].index_from].px,
                        poses[edges[i].index_from].py,
                        poses[edges[i].index_from].pz,
                        normalize_angle(poses[edges[i].index_from].om),
                        normalize_angle(poses[edges[i].index_from].fi),
                        normalize_angle(poses[edges[i].index_from].ka),
                        poses[edges[i].index_to].px,
                        poses[edges[i].index_to].py,
                        poses[edges[i].index_to].pz,
                        normalize_angle(poses[edges[i].index_to].om),
                        normalize_angle(poses[edges[i].index_to].fi),
                        normalize_angle(poses[edges[i].index_to].ka));
                }
                else if (is_cw) {
                    relative_pose_obs_eq_tait_bryan_cw_case1(
                        delta,
                        poses[edges[i].index_from].px,
                        poses[edges[i].index_from].py,
                        poses[edges[i].index_from].pz,
                        normalize_angle(poses[edges[i].index_from].om),
                        normalize_angle(poses[edges[i].index_from].fi),
                        normalize_angle(poses[edges[i].index_from].ka),
                        poses[edges[i].index_to].px,
                        poses[edges[i].index_to].py,
                        poses[edges[i].index_to].pz,
                        normalize_angle(poses[edges[i].index_to].om),
                        normalize_angle(poses[edges[i].index_to].fi),
                        normalize_angle(poses[edges[i].index_to].ka),
                        relative_pose.px,
                        relative_pose.py,
                        relative_pose.pz,
                        normalize_angle(relative_pose.om),
                        normalize_angle(relative_pose.fi),
                        normalize_angle(relative_pose.ka));
                    relative_pose_obs_eq_tait_bryan_cw_case1_jacobian(jacobian,
                        poses[edges[i].index_from].px,
                        poses[edges[i].index_from].py,
                        poses[edges[i].index_from].pz,
                        normalize_angle(poses[edges[i].index_from].om),
                        normalize_angle(poses[edges[i].index_from].fi),
                        normalize_angle(poses[edges[i].index_from].ka),
                        poses[edges[i].index_to].px,
                        poses[edges[i].index_to].py,
                        poses[edges[i].index_to].pz,
                        normalize_angle(poses[edges[i].index_to].om),
                        normalize_angle(poses[edges[i].index_to].fi),
                        normalize_angle(poses[edges[i].index_to].ka));
                }

                int ir = tripletListB.size();

                int ic_1 = edges[i].index_from * 6;
                int ic_2 = edges[i].index_to * 6;

                for (size_t row = 0; row < 6; row++) {
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
                tripletListB.emplace_back(ir + 3, 0, normalize_angle(delta(3, 0)));
                tripletListB.emplace_back(ir + 4, 0, normalize_angle(delta(4, 0)));
                tripletListB.emplace_back(ir + 5, 0, normalize_angle(delta(5, 0)));

                std::cout << "delta(0, 0): " << delta(0, 0) << std::endl;
                std::cout << "delta(1, 0): " << delta(0, 0) << std::endl;
                std::cout << "delta(2, 0): " << delta(0, 0) << std::endl;
                std::cout << "normalize_angle(delta(3, 0)): " << normalize_angle(delta(3, 0)) << std::endl;
                std::cout << "normalize_angle(delta(4, 0)): " << normalize_angle(delta(4, 0)) << std::endl;
                std::cout << "normalize_angle(delta(5, 0)): " << normalize_angle(delta(5, 0)) << std::endl;

                //for (int r = 0; r < 6; r++) {
                //    for (int c = 0; c < 6; c++) {
                //        tripletListP.emplace_back(ir + r, ir + c, edges[i].information_matrix.coeffRef(r, c));
                //   }
                //}

                tripletListP.emplace_back(ir    , ir    , get_cauchy_w(delta(0, 0), 10));
                tripletListP.emplace_back(ir + 1, ir + 1, get_cauchy_w(delta(1, 0), 10));
                tripletListP.emplace_back(ir + 2, ir + 2, get_cauchy_w(delta(2, 0), 10));
                tripletListP.emplace_back(ir + 3, ir + 3, get_cauchy_w(delta(3, 0), 10));
                tripletListP.emplace_back(ir + 4, ir + 4, get_cauchy_w(delta(4, 0), 10));
                tripletListP.emplace_back(ir + 5, ir + 5, get_cauchy_w(delta(5, 0), 10));
            }
            if (is_fix_first_node) {
                int ir = tripletListB.size();
                tripletListA.emplace_back(ir, 0,     1);
                tripletListA.emplace_back(ir + 1, 1, 1);
                tripletListA.emplace_back(ir + 2, 2, 1);
                tripletListA.emplace_back(ir + 3, 3, 1);
                tripletListA.emplace_back(ir + 4, 4, 1);
                tripletListA.emplace_back(ir + 5, 5, 1);

                tripletListP.emplace_back(ir, ir,         1);
                tripletListP.emplace_back(ir + 1, ir + 1, 1);
                tripletListP.emplace_back(ir + 2, ir + 2, 1);
                tripletListP.emplace_back(ir + 3, ir + 3, 1);
                tripletListP.emplace_back(ir + 4, ir + 4, 1);
                tripletListP.emplace_back(ir + 5, ir + 5, 1);

                tripletListB.emplace_back(ir, 0, 0);
                tripletListB.emplace_back(ir + 1, 0, 0);
                tripletListB.emplace_back(ir + 2, 0, 0);
                tripletListB.emplace_back(ir + 3, 0, 0);
                tripletListB.emplace_back(ir + 4, 0, 0);
                tripletListB.emplace_back(ir + 5, 0, 0);
            }

            Eigen::SparseMatrix<double> matA(tripletListB.size(), point_clouds_container.point_clouds.size() * 6);
            Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
            Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

            matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
            matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
            matB.setFromTriplets(tripletListB.begin(), tripletListB.end());


            Eigen::SparseMatrix<double> AtPA(point_clouds_container.point_clouds.size() * 6, point_clouds_container.point_clouds.size() * 6);
            Eigen::SparseMatrix<double> AtPB(point_clouds_container.point_clouds.size() * 6, 1);

            {
                Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
                AtPA = (AtP)*matA;
                AtPB = (AtP)*matB;
            }

            tripletListA.clear();
            tripletListP.clear();
            tripletListB.clear();

            std::cout << "AtPA.size: " << AtPA.size() << std::endl;
            std::cout << "AtPB.size: " << AtPB.size() << std::endl;

            std::cout << "start solving AtPA=AtPB" << std::endl;
            Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);

            std::cout << "x = solver.solve(AtPB)" << std::endl;
            Eigen::SparseMatrix<double> x = solver.solve(AtPB);

            std::vector<double> h_x;

            for (int k = 0; k < x.outerSize(); ++k) {
                for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it) {
                    h_x.push_back(it.value());
                }
            }

            std::cout << "h_x.size(): " << h_x.size() << std::endl;

            std::cout << "AtPA=AtPB SOLVED" << std::endl;
            std::cout << "updates:" << std::endl;
            for (size_t i = 0; i < h_x.size(); i += 6) {
                std::cout << h_x[i] << "," << h_x[i + 1] << "," << h_x[i + 2] << "," << h_x[i + 3] << "," << h_x[i + 4] << "," << h_x[i + 5] << std::endl;
            }

            if (h_x.size() == 6 * point_clouds_container.point_clouds.size()) {
                int counter = 0;

                for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++) {
                    TaitBryanPose pose = poses[i];
                    poses[i].px += h_x[counter++];
                    poses[i].py += h_x[counter++];
                    poses[i].pz += h_x[counter++];
                    poses[i].om += h_x[counter++];
                    poses[i].fi += h_x[counter++];
                    poses[i].ka += h_x[counter++];

                    if (i == 0 && is_fix_first_node) {
                        poses[i] = pose;
                    }
                }
                std::cout << "optimizing with tait bryan finished" << std::endl;
            }
            else {
                std::cout << "optimizing with tait bryan FAILED" << std::endl;
                std::cout << "h_x.size(): " << h_x.size() << " should be: " << 6 * point_clouds_container.point_clouds.size() << std::endl;
                is_ok = false;
                break;
            }
        }
        if (is_ok) {
            for (size_t i = 0; i < m_poses.size(); i++) {
                if (is_wc) {
                    m_poses[i] = affine_matrix_from_pose_tait_bryan(poses[i]);
                }
                else if (is_cw) {
                    m_poses[i] = affine_matrix_from_pose_tait_bryan(poses[i]).inverse();
                }
            }
        }
    }

    if (is_rodrigues) {
        std::vector<RodriguesPose> poses;

        for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++) {
            if (is_wc) {
                poses.push_back(pose_rodrigues_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose));
            }
            else if (is_cw) {
                poses.push_back(pose_rodrigues_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose.inverse()));
            }
        }

        for (int iter = 0; iter < iterations; iter++) {
            add_noise_to_poses(poses);

            std::vector<Eigen::Triplet<double>> tripletListA;
            std::vector<Eigen::Triplet<double>> tripletListP;
            std::vector<Eigen::Triplet<double>> tripletListB;

            for (size_t i = 0; i < edges.size(); i++) {
                Eigen::Matrix<double, 6, 1> delta;
                Eigen::Matrix<double, 6, 12, Eigen::RowMajor> jacobian;
                auto relative_pose = pose_rodrigues_from_affine_matrix(edges[i].m_relative_pose);
                if (is_wc) {

                    std::cout << "relative_pose.px " << relative_pose.px << std::endl;
                    std::cout << "relative_pose.py " << relative_pose.py << std::endl;
                    std::cout << "relative_pose.py " << relative_pose.pz << std::endl;
                    std::cout << "relative_pose.sx " << relative_pose.sx << std::endl;
                    std::cout << "relative_pose.sy " << relative_pose.sy << std::endl;
                    std::cout << "relative_pose.sz " << relative_pose.sz << std::endl;

                    relative_pose_obs_eq_rodrigues_wc(
                        delta,
                        poses[edges[i].index_from].px,
                        poses[edges[i].index_from].py,
                        poses[edges[i].index_from].pz,
                        poses[edges[i].index_from].sx,
                        poses[edges[i].index_from].sy,
                        poses[edges[i].index_from].sz,
                        poses[edges[i].index_to].px,
                        poses[edges[i].index_to].py,
                        poses[edges[i].index_to].pz,
                        poses[edges[i].index_to].sx,
                        poses[edges[i].index_to].sy,
                        poses[edges[i].index_to].sz,
                        relative_pose.px,
                        relative_pose.py,
                        relative_pose.pz,
                        relative_pose.sx,
                        relative_pose.sy,
                        relative_pose.sz);
                    relative_pose_obs_eq_rodrigues_wc_jacobian(jacobian,
                        poses[edges[i].index_from].px,
                        poses[edges[i].index_from].py,
                        poses[edges[i].index_from].pz,
                        poses[edges[i].index_from].sx,
                        poses[edges[i].index_from].sy,
                        poses[edges[i].index_from].sz,
                        poses[edges[i].index_to].px,
                        poses[edges[i].index_to].py,
                        poses[edges[i].index_to].pz,
                        poses[edges[i].index_to].sx,
                        poses[edges[i].index_to].sy,
                        poses[edges[i].index_to].sz);
                }
                else if (is_cw) {
                    /*relative_pose_obs_eq_rodrigues_cw(
                        delta,
                        poses[edges[i].index_from].px,
                        poses[edges[i].index_from].py,
                        poses[edges[i].index_from].pz,
                        poses[edges[i].index_from].sx,
                        poses[edges[i].index_from].sy,
                        poses[edges[i].index_from].sz,
                        poses[edges[i].index_to].px,
                        poses[edges[i].index_to].py,
                        poses[edges[i].index_to].pz,
                        poses[edges[i].index_to].sx,
                        poses[edges[i].index_to].sy,
                        poses[edges[i].index_to].sz,
                        relative_pose.px,
                        relative_pose.py,
                        relative_pose.pz,
                        relative_pose.sx,
                        relative_pose.sy,
                        relative_pose.sz);
                    relative_pose_obs_eq_rodrigues_cw_jacobian(jacobian,
                        poses[edges[i].index_from].px,
                        poses[edges[i].index_from].py,
                        poses[edges[i].index_from].pz,
                        poses[edges[i].index_from].sx,
                        poses[edges[i].index_from].sy,
                        poses[edges[i].index_from].sz,
                        poses[edges[i].index_to].px,
                        poses[edges[i].index_to].py,
                        poses[edges[i].index_to].pz,
                        poses[edges[i].index_to].sx,
                        poses[edges[i].index_to].sy,
                        poses[edges[i].index_to].sz);
                        */
                }

                int ir = tripletListB.size();

                int ic_1 = edges[i].index_from * 6;
                int ic_2 = edges[i].index_to * 6;

                for (size_t row = 0; row < 6; row++) {
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

                std::cout << "delta(0, 0): " << delta(0, 0) << std::endl;
                std::cout << "delta(1, 0): " << delta(1, 0) << std::endl;
                std::cout << "delta(2, 0): " << delta(2, 0) << std::endl;
                std::cout << "delta(3, 0): " << delta(3, 0) << std::endl;
                std::cout << "delta(4, 0): " << delta(4, 0) << std::endl;
                std::cout << "delta(5, 0): " << delta(5, 0) << std::endl;

                //for (int r = 0; r < 6; r++) {
                //    for (int c = 0; c < 6; c++) {
                //        tripletListP.emplace_back(ir + r, ir + c, edges[i].information_matrix.coeffRef(r, c));
                //   }
                //}

                tripletListP.emplace_back(ir, ir,         get_cauchy_w(delta(0, 0), 10));
                tripletListP.emplace_back(ir + 1, ir + 1, get_cauchy_w(delta(1, 0), 10));
                tripletListP.emplace_back(ir + 2, ir + 2, get_cauchy_w(delta(2, 0), 10));
                tripletListP.emplace_back(ir + 3, ir + 3, get_cauchy_w(delta(3, 0), 10));
                tripletListP.emplace_back(ir + 4, ir + 4, get_cauchy_w(delta(4, 0), 10));
                tripletListP.emplace_back(ir + 5, ir + 5, get_cauchy_w(delta(5, 0), 10));
            }

            if (is_fix_first_node) {
                int ir = tripletListB.size();
                tripletListA.emplace_back(ir, 0,     1);
                tripletListA.emplace_back(ir + 1, 1, 1);
                tripletListA.emplace_back(ir + 2, 2, 1);
                tripletListA.emplace_back(ir + 3, 3, 1);
                tripletListA.emplace_back(ir + 4, 4, 1);
                tripletListA.emplace_back(ir + 5, 5, 1);

                tripletListP.emplace_back(ir, ir,         1);
                tripletListP.emplace_back(ir + 1, ir + 1, 1);
                tripletListP.emplace_back(ir + 2, ir + 2, 1);
                tripletListP.emplace_back(ir + 3, ir + 3, 1);
                tripletListP.emplace_back(ir + 4, ir + 4, 1);
                tripletListP.emplace_back(ir + 5, ir + 5, 1);

                tripletListB.emplace_back(ir, 0, 0);
                tripletListB.emplace_back(ir + 1, 0, 0);
                tripletListB.emplace_back(ir + 2, 0, 0);
                tripletListB.emplace_back(ir + 3, 0, 0);
                tripletListB.emplace_back(ir + 4, 0, 0);
                tripletListB.emplace_back(ir + 5, 0, 0);
            }

            Eigen::SparseMatrix<double> matA(tripletListB.size(), point_clouds_container.point_clouds.size() * 6);
            Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
            Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

            matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
            matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
            matB.setFromTriplets(tripletListB.begin(), tripletListB.end());


            Eigen::SparseMatrix<double> AtPA(point_clouds_container.point_clouds.size() * 6, point_clouds_container.point_clouds.size() * 6);
            Eigen::SparseMatrix<double> AtPB(point_clouds_container.point_clouds.size() * 6, 1);

            {
                Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
                AtPA = (AtP)*matA;
                AtPB = (AtP)*matB;
            }

            tripletListA.clear();
            tripletListP.clear();
            tripletListB.clear();

            std::cout << "AtPA.size: " << AtPA.size() << std::endl;
            std::cout << "AtPB.size: " << AtPB.size() << std::endl;

            std::cout << "start solving AtPA=AtPB" << std::endl;
            Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);

            std::cout << "x = solver.solve(AtPB)" << std::endl;
            Eigen::SparseMatrix<double> x = solver.solve(AtPB);

            std::vector<double> h_x;

            std::cout << "solution [row,col,value]" << std::endl;
            for (int k = 0; k < x.outerSize(); ++k) {
                for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it) {
                    h_x.push_back(it.value());
                    std::cout << "[" << it.row() << "," << it.col() << "," << it.value() << "]" << std::endl;
                }
            }

            std::cout << "h_x.size(): " << h_x.size() << std::endl;

            std::cout << "AtPA=AtPB SOLVED" << std::endl;
            std::cout << "updates:" << std::endl;
            for (size_t i = 0; i < h_x.size(); i += 6) {
                std::cout << h_x[i] << "," << h_x[i + 1] << "," << h_x[i + 2] << "," << h_x[i + 3] << "," << h_x[i + 4] << "," << h_x[i + 5] << std::endl;
            }

            if (h_x.size() == 6 * point_clouds_container.point_clouds.size()) {
                int counter = 0;

                for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++) {
                    RodriguesPose pose = poses[i];
                    poses[i].px += h_x[counter++];
                    poses[i].py += h_x[counter++];
                    poses[i].pz += h_x[counter++];
                    poses[i].sx += h_x[counter++];
                    poses[i].sy += h_x[counter++];
                    poses[i].sz += h_x[counter++];

                    if (i == 0 && is_fix_first_node) {
                        poses[i] = pose;
                    }
                }
                std::cout << "optimizing with rodrigues finished" << std::endl;
            }
            else {
                std::cout << "optimizing with rodrigues FAILED" << std::endl;
                std::cout << "h_x.size(): " << h_x.size() << " should be: " << 6 * point_clouds_container.point_clouds.size() << std::endl;
                is_ok = false;
                break;
            }
        }
        if (is_ok) {
            for (size_t i = 0; i < m_poses.size(); i++) {
                if (is_wc) {
                    m_poses[i] = affine_matrix_from_pose_rodrigues(poses[i]);
                }
                else if (is_cw) {
                    m_poses[i] = affine_matrix_from_pose_rodrigues(poses[i]).inverse();
                }
            }
        }
    }

    if (is_quaternion) {
        std::vector<QuaternionPose> poses;

        for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++) {
            if (is_wc) {
                poses.push_back(pose_quaternion_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose));
            }
            else if (is_cw) {
                poses.push_back(pose_quaternion_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose.inverse()));
            }
        }

        for (int iter = 0; iter < iterations; iter++) {
            add_noise_to_poses(poses);

            std::vector<Eigen::Triplet<double>> tripletListA;
            std::vector<Eigen::Triplet<double>> tripletListP;
            std::vector<Eigen::Triplet<double>> tripletListB;

            for (size_t i = 0; i < edges.size(); i++) {
                Eigen::Matrix<double, 7, 1> delta;
                Eigen::Matrix<double, 7, 14, Eigen::RowMajor> jacobian;
                auto relative_pose = pose_quaternion_from_affine_matrix(edges[i].m_relative_pose);
                if (is_wc) {
                    relative_pose_obs_eq_quaternion_wc(
                        delta,
                        poses[edges[i].index_from].px,
                        poses[edges[i].index_from].py,
                        poses[edges[i].index_from].pz,
                        (poses[edges[i].index_from].q0),
                        (poses[edges[i].index_from].q1),
                        (poses[edges[i].index_from].q2),
                        poses[edges[i].index_from].q3,
                        poses[edges[i].index_to].px,
                        poses[edges[i].index_to].py,
                        poses[edges[i].index_to].pz,
                        poses[edges[i].index_to].q0,
                        poses[edges[i].index_to].q1,
                        poses[edges[i].index_to].q2,
                        poses[edges[i].index_to].q3,
                        relative_pose.px,
                        relative_pose.py,
                        relative_pose.pz,
                        relative_pose.q0,
                        relative_pose.q1,
                        relative_pose.q2,
                        relative_pose.q3);
                    relative_pose_obs_eq_quaternion_wc_jacobian(jacobian,
                        poses[edges[i].index_from].px,
                        poses[edges[i].index_from].py,
                        poses[edges[i].index_from].pz,
                        poses[edges[i].index_from].q0,
                        poses[edges[i].index_from].q1,
                        poses[edges[i].index_from].q2,
                        poses[edges[i].index_from].q3,
                        poses[edges[i].index_to].px,
                        poses[edges[i].index_to].py,
                        poses[edges[i].index_to].pz,
                        poses[edges[i].index_to].q0,
                        poses[edges[i].index_to].q1,
                        poses[edges[i].index_to].q2,
                        poses[edges[i].index_to].q3);
                }
                else if (is_cw) {
                    relative_pose_obs_eq_quaternion_cw(
                        delta,
                        poses[edges[i].index_from].px,
                        poses[edges[i].index_from].py,
                        poses[edges[i].index_from].pz,
                        poses[edges[i].index_from].q0,
                        poses[edges[i].index_from].q1,
                        poses[edges[i].index_from].q2,
                        poses[edges[i].index_from].q3,
                        poses[edges[i].index_to].px,
                        poses[edges[i].index_to].py,
                        poses[edges[i].index_to].pz,
                        poses[edges[i].index_to].q0,
                        poses[edges[i].index_to].q1,
                        poses[edges[i].index_to].q2,
                        poses[edges[i].index_to].q3,
                        relative_pose.px,
                        relative_pose.py,
                        relative_pose.pz,
                        relative_pose.q0,
                        relative_pose.q1,
                        relative_pose.q2,
                        relative_pose.q3);
                    relative_pose_obs_eq_quaternion_cw_jacobian(jacobian,
                        poses[edges[i].index_from].px,
                        poses[edges[i].index_from].py,
                        poses[edges[i].index_from].pz,
                        poses[edges[i].index_from].q0,
                        poses[edges[i].index_from].q1,
                        poses[edges[i].index_from].q2,
                        poses[edges[i].index_from].q3,
                        poses[edges[i].index_to].px,
                        poses[edges[i].index_to].py,
                        poses[edges[i].index_to].pz,
                        poses[edges[i].index_to].q0,
                        poses[edges[i].index_to].q1,
                        poses[edges[i].index_to].q2,
                        poses[edges[i].index_to].q3);
                }

                int ir = tripletListB.size();

                int ic_1 = edges[i].index_from * 7;
                int ic_2 = edges[i].index_to * 7;

                for (size_t row = 0; row < 7; row++) {
                    tripletListA.emplace_back(ir + row, ic_1, -jacobian(row, 0));
                    tripletListA.emplace_back(ir + row, ic_1 + 1, -jacobian(row, 1));
                    tripletListA.emplace_back(ir + row, ic_1 + 2, -jacobian(row, 2));
                    tripletListA.emplace_back(ir + row, ic_1 + 3, -jacobian(row, 3));
                    tripletListA.emplace_back(ir + row, ic_1 + 4, -jacobian(row, 4));
                    tripletListA.emplace_back(ir + row, ic_1 + 5, -jacobian(row, 5));
                    tripletListA.emplace_back(ir + row, ic_1 + 6, -jacobian(row, 6));

                    tripletListA.emplace_back(ir + row, ic_2, -jacobian(row, 7));
                    tripletListA.emplace_back(ir + row, ic_2 + 1, -jacobian(row, 8));
                    tripletListA.emplace_back(ir + row, ic_2 + 2, -jacobian(row, 9));
                    tripletListA.emplace_back(ir + row, ic_2 + 3, -jacobian(row, 10));
                    tripletListA.emplace_back(ir + row, ic_2 + 4, -jacobian(row, 11));
                    tripletListA.emplace_back(ir + row, ic_2 + 5, -jacobian(row, 12));
                    tripletListA.emplace_back(ir + row, ic_2 + 6, -jacobian(row, 13));
                }

                tripletListB.emplace_back(ir, 0, delta(0, 0));
                tripletListB.emplace_back(ir + 1, 0, delta(1, 0));
                tripletListB.emplace_back(ir + 2, 0, delta(2, 0));
                tripletListB.emplace_back(ir + 3, 0, delta(3, 0));
                tripletListB.emplace_back(ir + 4, 0, delta(4, 0));
                tripletListB.emplace_back(ir + 5, 0, delta(5, 0));
                tripletListB.emplace_back(ir + 6, 0, delta(6, 0));

                //for (int r = 0; r < 6; r++) {
                //    for (int c = 0; c < 6; c++) {
                //        tripletListP.emplace_back(ir + r, ir + c, edges[i].information_matrix.coeffRef(r, c));
                //   }
                //}

                tripletListP.emplace_back(ir, ir, get_cauchy_w(delta(0, 0), 1));
                tripletListP.emplace_back(ir + 1, ir + 1, get_cauchy_w(delta(1, 0), 10));
                tripletListP.emplace_back(ir + 2, ir + 2, get_cauchy_w(delta(2, 0), 10));
                tripletListP.emplace_back(ir + 3, ir + 3, get_cauchy_w(delta(3, 0), 10));
                tripletListP.emplace_back(ir + 4, ir + 4, get_cauchy_w(delta(4, 0), 10));
                tripletListP.emplace_back(ir + 5, ir + 5, get_cauchy_w(delta(5, 0), 10));
                tripletListP.emplace_back(ir + 6, ir + 6, get_cauchy_w(delta(6, 0), 10));
            }

            if (is_fix_first_node) {
                int ir = tripletListB.size();
                tripletListA.emplace_back(ir, 0, 1);
                tripletListA.emplace_back(ir + 1, 1, 1);
                tripletListA.emplace_back(ir + 2, 2, 1);
                tripletListA.emplace_back(ir + 3, 3, 1);
                tripletListA.emplace_back(ir + 4, 4, 1);
                tripletListA.emplace_back(ir + 5, 5, 1);
                tripletListA.emplace_back(ir + 6, 6, 1);

                tripletListP.emplace_back(ir, ir,         1);
                tripletListP.emplace_back(ir + 1, ir + 1, 1);
                tripletListP.emplace_back(ir + 2, ir + 2, 1);
                tripletListP.emplace_back(ir + 3, ir + 3, 1);
                tripletListP.emplace_back(ir + 4, ir + 4, 1);
                tripletListP.emplace_back(ir + 5, ir + 5, 1);
                tripletListP.emplace_back(ir + 6, ir + 6, 1);

                tripletListB.emplace_back(ir, 0, 0);
                tripletListB.emplace_back(ir + 1, 0, 0);
                tripletListB.emplace_back(ir + 2, 0, 0);
                tripletListB.emplace_back(ir + 3, 0, 0);
                tripletListB.emplace_back(ir + 4, 0, 0);
                tripletListB.emplace_back(ir + 5, 0, 0);
                tripletListB.emplace_back(ir + 6, 0, 0);
            }

            for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++) {
                int ic = i * 7;
                int ir = tripletListB.size();
                QuaternionPose pose;
                pose = poses[i];

                double delta;
                quaternion_constraint(delta, pose.q0, pose.q1, pose.q2, pose.q3);

                Eigen::Matrix<double, 1, 4> jacobian;
                quaternion_constraint_jacobian(jacobian, pose.q0, pose.q1, pose.q2, pose.q3);

                tripletListA.emplace_back(ir, ic + 3, -jacobian(0, 0));
                tripletListA.emplace_back(ir, ic + 4, -jacobian(0, 1));
                tripletListA.emplace_back(ir, ic + 5, -jacobian(0, 2));
                tripletListA.emplace_back(ir, ic + 6, -jacobian(0, 3));

                tripletListP.emplace_back(ir, ir, 1000000.0);

                tripletListB.emplace_back(ir, 0, delta);
            }

            Eigen::SparseMatrix<double> matA(tripletListB.size(), point_clouds_container.point_clouds.size() * 7);
            Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
            Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

            matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
            matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
            matB.setFromTriplets(tripletListB.begin(), tripletListB.end());


            Eigen::SparseMatrix<double> AtPA(point_clouds_container.point_clouds.size() * 7, point_clouds_container.point_clouds.size() * 7);
            Eigen::SparseMatrix<double> AtPB(point_clouds_container.point_clouds.size() * 7, 1);

            {
                Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
                AtPA = (AtP)*matA;
                AtPB = (AtP)*matB;
            }

            tripletListA.clear();
            tripletListP.clear();
            tripletListB.clear();

            std::cout << "AtPA.size: " << AtPA.size() << std::endl;
            std::cout << "AtPB.size: " << AtPB.size() << std::endl;

            std::cout << "start solving AtPA=AtPB" << std::endl;
            Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);

            std::cout << "x = solver.solve(AtPB)" << std::endl;
            Eigen::SparseMatrix<double> x = solver.solve(AtPB);

            std::vector<double> h_x;

            for (int k = 0; k < x.outerSize(); ++k) {
                for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it) {
                    h_x.push_back(it.value());
                }
            }

            std::cout << "h_x.size(): " << h_x.size() << std::endl;

            std::cout << "AtPA=AtPB SOLVED" << std::endl;
            std::cout << "updates:" << std::endl;
            for (size_t i = 0; i < h_x.size(); i += 7) {
                std::cout << h_x[i] << "," << h_x[i + 1] << "," << h_x[i + 2] << "," << h_x[i + 3] << "," << h_x[i + 4] << "," << h_x[i + 5] << "," << h_x[i + 6] << std::endl;
            }

            if (h_x.size() == 7 * point_clouds_container.point_clouds.size()) {
                int counter = 0;

                for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++) {
                    QuaternionPose pose = poses[i];
                    poses[i].px += h_x[counter++];
                    poses[i].py += h_x[counter++];
                    poses[i].pz += h_x[counter++];
                    poses[i].q0 += h_x[counter++];
                    poses[i].q1 += h_x[counter++];
                    poses[i].q2 += h_x[counter++];
                    poses[i].q3 += h_x[counter++];

                    if (i == 0 && is_fix_first_node) {
                        poses[i] = pose;
                    }
                }
                std::cout << "optimizing with quaternion finished" << std::endl;
            }
            else {
                std::cout << "optimizing with quaternion FAILED" << std::endl;
                is_ok = false;
                break;
            }
        }
        if (is_ok) {
            for (size_t i = 0; i < m_poses.size(); i++) {
                if (is_wc) {
                    m_poses[i] = affine_matrix_from_pose_quaternion(poses[i]);
                }
                else if (is_cw) {
                    m_poses[i] = affine_matrix_from_pose_quaternion(poses[i]).inverse();
                }
            }
        }
    }

    if (is_ok) {
        for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++) {
            point_clouds_container.point_clouds[i].m_pose = m_poses[i];
            point_clouds_container.point_clouds[i].pose = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);
            point_clouds_container.point_clouds[i].gui_translation[0] = point_clouds_container.point_clouds[i].pose.px;
            point_clouds_container.point_clouds[i].gui_translation[1] = point_clouds_container.point_clouds[i].pose.py;
            point_clouds_container.point_clouds[i].gui_translation[2] = point_clouds_container.point_clouds[i].pose.pz;
            point_clouds_container.point_clouds[i].gui_rotation[0] = rad2deg(point_clouds_container.point_clouds[i].pose.om);
            point_clouds_container.point_clouds[i].gui_rotation[1] = rad2deg(point_clouds_container.point_clouds[i].pose.fi);
            point_clouds_container.point_clouds[i].gui_rotation[2] = rad2deg(point_clouds_container.point_clouds[i].pose.ka);
        }
    }

    //clean
    for (auto& pc : point_clouds_container.point_clouds) {
        pc.clean();
    }
    edges.clear();

    //-
    //covariance_matrices_after6x6 = compute_covariance_matrices_wc(point_clouds_container);

    //double mui = get_mean_uncertainty_xyz_impact6x6(covariance_matrices_before6x6, covariance_matrices_after6x6);
    //std::cout << "mean uncertainty_xyz impact: " << mui << std::endl;
    //--
    return true;
}

std::vector<Eigen::SparseMatrix<double>> PoseGraphSLAM::compute_covariance_matrices_and_rms(std::vector<PointCloud>& point_clouds, double& rms)
{
    std::vector<Eigen::SparseMatrix<double>> covariance_matrices;

    double ssr = 0.0;
    rms = 0.0;
    int num_obs = 0;
    int number_of_unknowns = 6;
    if (is_quaternion)number_of_unknowns = 7;
    Eigen::SparseMatrix<double> AtPA(point_clouds.size() * 7, point_clouds.size() * 7);


    for (auto& pc : point_clouds) {
        pc.build_rgd();
        pc.cout_rgd();
        pc.compute_normal_vectors(0.5);
    }

    //get edges based on overlap
    for (int i = 0; i < point_clouds.size(); i++) {
        for (int j = i + 1; j < point_clouds.size(); j++) {
            std::vector<std::pair<int, int>> nns = point_clouds[i].nns(point_clouds[j], 0.5);

            float overlap = float(nns.size()) / float(point_clouds[i].points_local.size());
            std::cout << "overlap: " << overlap << " between " << i << "," << j << std::endl;
            if (overlap > overlap_threshold) {
                Edge edge;
                edge.index_from = i;
                edge.index_to = j;
                edges.push_back(edge);
            }
        }
    }

    calculate_edges(point_clouds);


    //graph slam
    bool is_ok = true;
    std::vector<Eigen::Affine3d> m_poses;
    for (size_t i = 0; i < point_clouds.size(); i++) {
        m_poses.push_back(point_clouds[i].m_pose);
    }

    if (is_tait_bryan_angles) {
        std::vector<TaitBryanPose> poses;

        for (size_t i = 0; i < point_clouds.size(); i++) {
            if (is_wc) {
                poses.push_back(pose_tait_bryan_from_affine_matrix(point_clouds[i].m_pose));
            }
            else if (is_cw) {
                poses.push_back(pose_tait_bryan_from_affine_matrix(point_clouds[i].m_pose.inverse()));
            }
        }

        for (int iter = 0; iter < iterations; iter++) {
            std::vector<Eigen::Triplet<double>> tripletListA;
            std::vector<Eigen::Triplet<double>> tripletListP;
            std::vector<Eigen::Triplet<double>> tripletListB;

            for (size_t i = 0; i < edges.size(); i++) {
                Eigen::Matrix<double, 6, 1> delta;
                Eigen::Matrix<double, 6, 12, Eigen::RowMajor> jacobian;
                auto relative_pose = pose_tait_bryan_from_affine_matrix(edges[i].m_relative_pose);
                if (is_wc) {
                    relative_pose_obs_eq_tait_bryan_wc_case1(
                        delta,
                        poses[edges[i].index_from].px,
                        poses[edges[i].index_from].py,
                        poses[edges[i].index_from].pz,
                        normalize_angle(poses[edges[i].index_from].om),
                        normalize_angle(poses[edges[i].index_from].fi),
                        normalize_angle(poses[edges[i].index_from].ka),
                        poses[edges[i].index_to].px,
                        poses[edges[i].index_to].py,
                        poses[edges[i].index_to].pz,
                        normalize_angle(poses[edges[i].index_to].om),
                        normalize_angle(poses[edges[i].index_to].fi),
                        normalize_angle(poses[edges[i].index_to].ka),
                        relative_pose.px,
                        relative_pose.py,
                        relative_pose.pz,
                        normalize_angle(relative_pose.om),
                        normalize_angle(relative_pose.fi),
                        normalize_angle(relative_pose.ka));
                    relative_pose_obs_eq_tait_bryan_wc_case1_jacobian(jacobian,
                        poses[edges[i].index_from].px,
                        poses[edges[i].index_from].py,
                        poses[edges[i].index_from].pz,
                        normalize_angle(poses[edges[i].index_from].om),
                        normalize_angle(poses[edges[i].index_from].fi),
                        normalize_angle(poses[edges[i].index_from].ka),
                        poses[edges[i].index_to].px,
                        poses[edges[i].index_to].py,
                        poses[edges[i].index_to].pz,
                        normalize_angle(poses[edges[i].index_to].om),
                        normalize_angle(poses[edges[i].index_to].fi),
                        normalize_angle(poses[edges[i].index_to].ka));
                }
                else if (is_cw) {
                    relative_pose_obs_eq_tait_bryan_cw_case1(
                        delta,
                        poses[edges[i].index_from].px,
                        poses[edges[i].index_from].py,
                        poses[edges[i].index_from].pz,
                        normalize_angle(poses[edges[i].index_from].om),
                        normalize_angle(poses[edges[i].index_from].fi),
                        normalize_angle(poses[edges[i].index_from].ka),
                        poses[edges[i].index_to].px,
                        poses[edges[i].index_to].py,
                        poses[edges[i].index_to].pz,
                        normalize_angle(poses[edges[i].index_to].om),
                        normalize_angle(poses[edges[i].index_to].fi),
                        normalize_angle(poses[edges[i].index_to].ka),
                        relative_pose.px,
                        relative_pose.py,
                        relative_pose.pz,
                        normalize_angle(relative_pose.om),
                        normalize_angle(relative_pose.fi),
                        normalize_angle(relative_pose.ka));
                    relative_pose_obs_eq_tait_bryan_cw_case1_jacobian(jacobian,
                        poses[edges[i].index_from].px,
                        poses[edges[i].index_from].py,
                        poses[edges[i].index_from].pz,
                        normalize_angle(poses[edges[i].index_from].om),
                        normalize_angle(poses[edges[i].index_from].fi),
                        normalize_angle(poses[edges[i].index_from].ka),
                        poses[edges[i].index_to].px,
                        poses[edges[i].index_to].py,
                        poses[edges[i].index_to].pz,
                        normalize_angle(poses[edges[i].index_to].om),
                        normalize_angle(poses[edges[i].index_to].fi),
                        normalize_angle(poses[edges[i].index_to].ka));
                }

                int ir = tripletListB.size();

                int ic_1 = edges[i].index_from * 6;
                int ic_2 = edges[i].index_to * 6;

                for (size_t row = 0; row < 6; row++) {
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
                tripletListB.emplace_back(ir + 3, 0, normalize_angle(delta(3, 0)));
                tripletListB.emplace_back(ir + 4, 0, normalize_angle(delta(4, 0)));
                tripletListB.emplace_back(ir + 5, 0, normalize_angle(delta(5, 0)));


                ssr += delta(0, 0) * delta(0, 0);
                ssr += delta(1, 0) * delta(1, 0);
                ssr += delta(2, 0) * delta(2, 0);
                ssr += normalize_angle(delta(3, 0) * delta(3, 0));
                ssr += normalize_angle(delta(4, 0) * delta(4, 0));
                ssr += normalize_angle(delta(5, 0) * delta(5, 0));
                num_obs += 6;
               

                //for (int r = 0; r < 6; r++) {
                //    for (int c = 0; c < 6; c++) {
                //        tripletListP.emplace_back(ir + r, ir + c, edges[i].information_matrix.coeffRef(r, c));
                //   }
                //}

                tripletListP.emplace_back(ir, ir, 1);
                tripletListP.emplace_back(ir + 1, ir + 1, 1);
                tripletListP.emplace_back(ir + 2, ir + 2, 1);
                tripletListP.emplace_back(ir + 3, ir + 3, 1);
                tripletListP.emplace_back(ir + 4, ir + 4, 1);
                tripletListP.emplace_back(ir + 5, ir + 5, 1);
            }

            int ir = tripletListB.size();
			tripletListA.emplace_back(ir     , 0, 1);
			tripletListA.emplace_back(ir + 1 , 1, 1);
			tripletListA.emplace_back(ir + 2 , 2, 1);
			tripletListA.emplace_back(ir + 3 , 3, 1);
			tripletListA.emplace_back(ir + 4 , 4, 1);
			tripletListA.emplace_back(ir + 5 , 5, 1);

			tripletListP.emplace_back(ir     , ir,     1);
			tripletListP.emplace_back(ir + 1 , ir + 1, 1);
			tripletListP.emplace_back(ir + 2 , ir + 2, 1);
			tripletListP.emplace_back(ir + 3 , ir + 3, 1);
			tripletListP.emplace_back(ir + 4 , ir + 4, 1);
			tripletListP.emplace_back(ir + 5 , ir + 5, 1);

			tripletListB.emplace_back(ir     , 0, 0);
			tripletListB.emplace_back(ir + 1 , 0, 0);
			tripletListB.emplace_back(ir + 2 , 0, 0);
			tripletListB.emplace_back(ir + 3 , 0, 0);
			tripletListB.emplace_back(ir + 4 , 0, 0);
			tripletListB.emplace_back(ir + 5 , 0, 0);

            Eigen::SparseMatrix<double> matA(tripletListB.size(), point_clouds.size() * 6);
            Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
            Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

            matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
            matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
            matB.setFromTriplets(tripletListB.begin(), tripletListB.end());


            Eigen::SparseMatrix<double> AtPA(point_clouds.size() * 6, point_clouds.size() * 6);
           
            {
                Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
                AtPA = (AtP)*matA;
            }

            tripletListA.clear();
            tripletListP.clear();
            tripletListB.clear();
        }
    }

    if (is_rodrigues) {
        std::vector<RodriguesPose> poses;

        for (size_t i = 0; i < point_clouds.size(); i++) {
            if (is_wc) {
                poses.push_back(pose_rodrigues_from_affine_matrix(point_clouds[i].m_pose));
            }
            else if (is_cw) {
                poses.push_back(pose_rodrigues_from_affine_matrix(point_clouds[i].m_pose.inverse()));
            }
        }

        for (int iter = 0; iter < iterations; iter++) {
            std::vector<Eigen::Triplet<double>> tripletListA;
            std::vector<Eigen::Triplet<double>> tripletListP;
            std::vector<Eigen::Triplet<double>> tripletListB;

            for (size_t i = 0; i < edges.size(); i++) {
                Eigen::Matrix<double, 6, 1> delta;
                Eigen::Matrix<double, 6, 12, Eigen::RowMajor> jacobian;
                auto relative_pose = pose_rodrigues_from_affine_matrix(edges[i].m_relative_pose);
                if (is_wc) {
                    relative_pose_obs_eq_rodrigues_wc(
                        delta,
                        poses[edges[i].index_from].px,
                        poses[edges[i].index_from].py,
                        poses[edges[i].index_from].pz,
                        poses[edges[i].index_from].sx,
                        poses[edges[i].index_from].sy,
                        poses[edges[i].index_from].sz,
                        poses[edges[i].index_to].px,
                        poses[edges[i].index_to].py,
                        poses[edges[i].index_to].pz,
                        poses[edges[i].index_to].sx,
                        poses[edges[i].index_to].sy,
                        poses[edges[i].index_to].sz,
                        relative_pose.px,
                        relative_pose.py,
                        relative_pose.pz,
                        relative_pose.sx,
                        relative_pose.sy,
                        relative_pose.sz);
                    relative_pose_obs_eq_rodrigues_wc_jacobian(jacobian,
                        poses[edges[i].index_from].px,
                        poses[edges[i].index_from].py,
                        poses[edges[i].index_from].pz,
                        poses[edges[i].index_from].sx,
                        poses[edges[i].index_from].sy,
                        poses[edges[i].index_from].sz,
                        poses[edges[i].index_to].px,
                        poses[edges[i].index_to].py,
                        poses[edges[i].index_to].pz,
                        poses[edges[i].index_to].sx,
                        poses[edges[i].index_to].sy,
                        poses[edges[i].index_to].sz);
                }
                else if (is_cw) {
                    /*relative_pose_obs_eq_rodrigues_cw(
                        delta,
                        poses[edges[i].index_from].px,
                        poses[edges[i].index_from].py,
                        poses[edges[i].index_from].pz,
                        poses[edges[i].index_from].sx,
                        poses[edges[i].index_from].sy,
                        poses[edges[i].index_from].sz,
                        poses[edges[i].index_to].px,
                        poses[edges[i].index_to].py,
                        poses[edges[i].index_to].pz,
                        poses[edges[i].index_to].sx,
                        poses[edges[i].index_to].sy,
                        poses[edges[i].index_to].sz,
                        relative_pose.px,
                        relative_pose.py,
                        relative_pose.pz,
                        relative_pose.sx,
                        relative_pose.sy,
                        relative_pose.sz);
                    relative_pose_obs_eq_rodrigues_cw_jacobian(jacobian,
                        poses[edges[i].index_from].px,
                        poses[edges[i].index_from].py,
                        poses[edges[i].index_from].pz,
                        poses[edges[i].index_from].sx,
                        poses[edges[i].index_from].sy,
                        poses[edges[i].index_from].sz,
                        poses[edges[i].index_to].px,
                        poses[edges[i].index_to].py,
                        poses[edges[i].index_to].pz,
                        poses[edges[i].index_to].sx,
                        poses[edges[i].index_to].sy,
                        poses[edges[i].index_to].sz);*/
                }

                int ir = tripletListB.size();

                int ic_1 = edges[i].index_from * 6;
                int ic_2 = edges[i].index_to * 6;

                for (size_t row = 0; row < 6; row++) {
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

                ssr += delta(0, 0) * delta(0, 0);
                ssr += delta(1, 0) * delta(1, 0);
                ssr += delta(2, 0) * delta(2, 0);
                ssr += delta(3, 0) * delta(3, 0);
                ssr += delta(4, 0) * delta(4, 0);
                ssr += delta(5, 0) * delta(5, 0);
                num_obs += 6;

                //for (int r = 0; r < 6; r++) {
                //    for (int c = 0; c < 6; c++) {
                //        tripletListP.emplace_back(ir + r, ir + c, edges[i].information_matrix.coeffRef(r, c));
                //   }
                //}

                tripletListP.emplace_back(ir, ir, 1);
                tripletListP.emplace_back(ir + 1, ir + 1, 1);
                tripletListP.emplace_back(ir + 2, ir + 2, 1);
                tripletListP.emplace_back(ir + 3, ir + 3, 1);
                tripletListP.emplace_back(ir + 4, ir + 4, 1);
                tripletListP.emplace_back(ir + 5, ir + 5, 1);
            }

            int ir = tripletListB.size();
			tripletListA.emplace_back(ir     , 0, 1);
			tripletListA.emplace_back(ir + 1 , 1, 1);
			tripletListA.emplace_back(ir + 2 , 2, 1);
			tripletListA.emplace_back(ir + 3 , 3, 1);
			tripletListA.emplace_back(ir + 4 , 4, 1);
			tripletListA.emplace_back(ir + 5 , 5, 1);

			tripletListP.emplace_back(ir     , ir,     1);
			tripletListP.emplace_back(ir + 1 , ir + 1, 1);
			tripletListP.emplace_back(ir + 2 , ir + 2, 1);
			tripletListP.emplace_back(ir + 3 , ir + 3, 1);
			tripletListP.emplace_back(ir + 4 , ir + 4, 1);
			tripletListP.emplace_back(ir + 5 , ir + 5, 1);

			tripletListB.emplace_back(ir     , 0, 0);
			tripletListB.emplace_back(ir + 1 , 0, 0);
			tripletListB.emplace_back(ir + 2 , 0, 0);
			tripletListB.emplace_back(ir + 3 , 0, 0);
			tripletListB.emplace_back(ir + 4 , 0, 0);
			tripletListB.emplace_back(ir + 5 , 0, 0);

            Eigen::SparseMatrix<double> matA(tripletListB.size(), point_clouds.size() * 6);
            Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
            Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

            matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
            matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
            matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

            {
                Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
                AtPA = (AtP)*matA;
            }

            tripletListA.clear();
            tripletListP.clear();
            tripletListB.clear();
        }
    }

    if (is_quaternion) {
        std::vector<QuaternionPose> poses;

        for (size_t i = 0; i < point_clouds.size(); i++) {
            if (is_wc) {
                poses.push_back(pose_quaternion_from_affine_matrix(point_clouds[i].m_pose));
            }
            else if (is_cw) {
                poses.push_back(pose_quaternion_from_affine_matrix(point_clouds[i].m_pose.inverse()));
            }
        }

        for (int iter = 0; iter < iterations; iter++) {
            std::vector<Eigen::Triplet<double>> tripletListA;
            std::vector<Eigen::Triplet<double>> tripletListP;
            std::vector<Eigen::Triplet<double>> tripletListB;

            for (size_t i = 0; i < edges.size(); i++) {
                Eigen::Matrix<double, 7, 1> delta;
                Eigen::Matrix<double, 7, 14, Eigen::RowMajor> jacobian;
                auto relative_pose = pose_quaternion_from_affine_matrix(edges[i].m_relative_pose);
                if (is_wc) {
                    relative_pose_obs_eq_quaternion_wc(
                        delta,
                        poses[edges[i].index_from].px,
                        poses[edges[i].index_from].py,
                        poses[edges[i].index_from].pz,
                        poses[edges[i].index_from].q0,
                        poses[edges[i].index_from].q1,
                        poses[edges[i].index_from].q2,
                        poses[edges[i].index_from].q3,
                        poses[edges[i].index_to].px,
                        poses[edges[i].index_to].py,
                        poses[edges[i].index_to].pz,
                        poses[edges[i].index_to].q0,
                        poses[edges[i].index_to].q1,
                        poses[edges[i].index_to].q2,
                        poses[edges[i].index_to].q3,
                        relative_pose.px,
                        relative_pose.py,
                        relative_pose.pz,
                        relative_pose.q0,
                        relative_pose.q1,
                        relative_pose.q2,
                        relative_pose.q3);
                    relative_pose_obs_eq_quaternion_wc_jacobian(jacobian,
                        poses[edges[i].index_from].px,
                        poses[edges[i].index_from].py,
                        poses[edges[i].index_from].pz,
                        poses[edges[i].index_from].q0,
                        poses[edges[i].index_from].q1,
                        poses[edges[i].index_from].q2,
                        poses[edges[i].index_from].q3,
                        poses[edges[i].index_to].px,
                        poses[edges[i].index_to].py,
                        poses[edges[i].index_to].pz,
                        poses[edges[i].index_to].q0,
                        poses[edges[i].index_to].q1,
                        poses[edges[i].index_to].q2,
                        poses[edges[i].index_to].q3);
                }
                else if (is_cw) {
                    relative_pose_obs_eq_quaternion_cw(
                        delta,
                        poses[edges[i].index_from].px,
                        poses[edges[i].index_from].py,
                        poses[edges[i].index_from].pz,
                        poses[edges[i].index_from].q0,
                        poses[edges[i].index_from].q1,
                        poses[edges[i].index_from].q2,
                        poses[edges[i].index_from].q3,
                        poses[edges[i].index_to].px,
                        poses[edges[i].index_to].py,
                        poses[edges[i].index_to].pz,
                        poses[edges[i].index_to].q0,
                        poses[edges[i].index_to].q1,
                        poses[edges[i].index_to].q2,
                        poses[edges[i].index_to].q3,
                        relative_pose.px,
                        relative_pose.py,
                        relative_pose.pz,
                        relative_pose.q0,
                        relative_pose.q1,
                        relative_pose.q2,
                        relative_pose.q3);
                    relative_pose_obs_eq_quaternion_cw_jacobian(jacobian,
                        poses[edges[i].index_from].px,
                        poses[edges[i].index_from].py,
                        poses[edges[i].index_from].pz,
                        poses[edges[i].index_from].q0,
                        poses[edges[i].index_from].q1,
                        poses[edges[i].index_from].q2,
                        poses[edges[i].index_from].q3,
                        poses[edges[i].index_to].px,
                        poses[edges[i].index_to].py,
                        poses[edges[i].index_to].pz,
                        poses[edges[i].index_to].q0,
                        poses[edges[i].index_to].q1,
                        poses[edges[i].index_to].q2,
                        poses[edges[i].index_to].q3);
                }

                int ir = tripletListB.size();

                int ic_1 = edges[i].index_from * 7;
                int ic_2 = edges[i].index_to * 7;

                for (size_t row = 0; row < 7; row++) {
                    tripletListA.emplace_back(ir + row, ic_1, -jacobian(row, 0));
                    tripletListA.emplace_back(ir + row, ic_1 + 1, -jacobian(row, 1));
                    tripletListA.emplace_back(ir + row, ic_1 + 2, -jacobian(row, 2));
                    tripletListA.emplace_back(ir + row, ic_1 + 3, -jacobian(row, 3));
                    tripletListA.emplace_back(ir + row, ic_1 + 4, -jacobian(row, 4));
                    tripletListA.emplace_back(ir + row, ic_1 + 5, -jacobian(row, 5));
                    tripletListA.emplace_back(ir + row, ic_1 + 6, -jacobian(row, 6));

                    tripletListA.emplace_back(ir + row, ic_2, -jacobian(row, 7));
                    tripletListA.emplace_back(ir + row, ic_2 + 1, -jacobian(row, 8));
                    tripletListA.emplace_back(ir + row, ic_2 + 2, -jacobian(row, 9));
                    tripletListA.emplace_back(ir + row, ic_2 + 3, -jacobian(row, 10));
                    tripletListA.emplace_back(ir + row, ic_2 + 4, -jacobian(row, 11));
                    tripletListA.emplace_back(ir + row, ic_2 + 5, -jacobian(row, 12));
                    tripletListA.emplace_back(ir + row, ic_2 + 6, -jacobian(row, 13));
                }

                tripletListB.emplace_back(ir, 0, delta(0, 0));
                tripletListB.emplace_back(ir + 1, 0, delta(1, 0));
                tripletListB.emplace_back(ir + 2, 0, delta(2, 0));
                tripletListB.emplace_back(ir + 3, 0, delta(3, 0));
                tripletListB.emplace_back(ir + 4, 0, delta(4, 0));
                tripletListB.emplace_back(ir + 5, 0, delta(5, 0));
                tripletListB.emplace_back(ir + 6, 0, delta(6, 0));

                ssr += delta(0, 0) * delta(0, 0);
                ssr += delta(1, 0) * delta(1, 0);
                ssr += delta(2, 0) * delta(2, 0);
                ssr += delta(3, 0) * delta(3, 0);
                ssr += delta(4, 0) * delta(4, 0);
                ssr += delta(5, 0) * delta(5, 0);
                ssr += delta(6, 0) * delta(6, 0);
                num_obs += 7;

                //for (int r = 0; r < 6; r++) {
                //    for (int c = 0; c < 6; c++) {
                //        tripletListP.emplace_back(ir + r, ir + c, edges[i].information_matrix.coeffRef(r, c));
                //   }
                //}

                tripletListP.emplace_back(ir, ir, 1);
                tripletListP.emplace_back(ir + 1, ir + 1, 1);
                tripletListP.emplace_back(ir + 2, ir + 2, 1);
                tripletListP.emplace_back(ir + 3, ir + 3, 1);
                tripletListP.emplace_back(ir + 4, ir + 4, 1);
                tripletListP.emplace_back(ir + 5, ir + 5, 1);
                tripletListP.emplace_back(ir + 6, ir + 6, 1);
            }

            int ir = tripletListB.size();
			tripletListA.emplace_back(ir     , 0, 1);
			tripletListA.emplace_back(ir + 1 , 1, 1);
			tripletListA.emplace_back(ir + 2 , 2, 1);
			tripletListA.emplace_back(ir + 3 , 3, 1);
			tripletListA.emplace_back(ir + 4 , 4, 1);
			tripletListA.emplace_back(ir + 5 , 5, 1);
			tripletListA.emplace_back(ir + 6 , 6, 1);

			tripletListP.emplace_back(ir     , ir,     1);
			tripletListP.emplace_back(ir + 1 , ir + 1, 1);
			tripletListP.emplace_back(ir + 2 , ir + 2, 1);
			tripletListP.emplace_back(ir + 3 , ir + 3, 1);
			tripletListP.emplace_back(ir + 4 , ir + 4, 1);
			tripletListP.emplace_back(ir + 5 , ir + 5, 1);
			tripletListP.emplace_back(ir + 6 , ir + 6, 1);

			tripletListB.emplace_back(ir     , 0, 0);
			tripletListB.emplace_back(ir + 1 , 0, 0);
			tripletListB.emplace_back(ir + 2 , 0, 0);
			tripletListB.emplace_back(ir + 3 , 0, 0);
			tripletListB.emplace_back(ir + 4 , 0, 0);
			tripletListB.emplace_back(ir + 5 , 0, 0);
			tripletListB.emplace_back(ir + 6 , 0, 0);

            for (size_t i = 0; i < point_clouds.size(); i++) {
                int ic = i * 7;
                int ir = tripletListB.size();
                QuaternionPose pose;
                pose = poses[i];

                double delta;
                quaternion_constraint(delta, pose.q0, pose.q1, pose.q2, pose.q3);

                Eigen::Matrix<double, 1, 4> jacobian;
                quaternion_constraint_jacobian(jacobian, pose.q0, pose.q1, pose.q2, pose.q3);

                tripletListA.emplace_back(ir, ic + 3, -jacobian(0, 0));
                tripletListA.emplace_back(ir, ic + 4, -jacobian(0, 1));
                tripletListA.emplace_back(ir, ic + 5, -jacobian(0, 2));
                tripletListA.emplace_back(ir, ic + 6, -jacobian(0, 3));

                tripletListP.emplace_back(ir, ir, 1000000.0);

                tripletListB.emplace_back(ir, 0, delta);
            }

            Eigen::SparseMatrix<double> matA(tripletListB.size(), point_clouds.size() * 7);
            Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
            Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

            matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
            matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
            matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

            Eigen::SparseMatrix<double> AtPA(point_clouds.size() * 7, point_clouds.size() * 7);
            {
                Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
                AtPA = (AtP)*matA;
            }

            tripletListA.clear();
            tripletListP.clear();
            tripletListB.clear();
        }
    }

    //double sq = ssr / ((double)num_obs - point_clouds.size() * number_of_unknowns);
    double sq = ssr / ((double)num_obs);
    rms = ssr / (double)num_obs;

    std::cout << "sq: " << sq << std::endl;

    AtPA = 0.5 * AtPA;

    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);
    Eigen::SparseMatrix<double> I(point_clouds.size() * number_of_unknowns, point_clouds.size() * number_of_unknowns);
    I.setIdentity();

    Eigen::SparseMatrix<double> AtAinv(point_clouds.size() * number_of_unknowns, point_clouds.size() * number_of_unknowns);
    AtAinv = solver.solve(I);

    AtAinv = AtAinv * sq;

    for (int i = 0; i < point_clouds.size(); i++) {
        Eigen::SparseMatrix<double> cm(number_of_unknowns, number_of_unknowns);
        for (int r = 0; r < number_of_unknowns; r++) {
            for (int c = 0; c < number_of_unknowns; c++) {
                cm.coeffRef(r, c) = AtAinv.coeff(i * number_of_unknowns + r, i * number_of_unknowns + c);
            }
        }
        covariance_matrices.push_back(cm);
        std::cout << "cm" << std::endl;
        std::cout << cm << std::endl;
        std::cout << "AtPA" << std::endl;
        std::cout << AtPA << std::endl;
    }

    return covariance_matrices;
}

void  PoseGraphSLAM::calculate_edges(std::vector<PointCloud>& point_clouds){
    if(pair_wise_matching_type == PairWiseMatchingType::general)
    {
		for (size_t i = 0; i < edges.size(); i++) {
			std::cout << "PROGRESS calculating edges: " << i + 1 << " of " << edges.size() << "[" << edges[i].index_from << "," << edges[i].index_to << "]" << std::endl;

			PointClouds pcs;
			pcs.point_clouds.push_back(point_clouds[edges[i].index_from]);
			pcs.point_clouds.push_back(point_clouds[edges[i].index_to]);

			Eigen::Affine3d m_inv = pcs.point_clouds[0].m_pose.inverse();

			auto m_initial_pose = pcs.point_clouds[0].m_pose.inverse() * pcs.point_clouds[1].m_pose;

			pcs.point_clouds[0].m_pose = m_inv * pcs.point_clouds[0].m_pose;
			pcs.point_clouds[1].m_pose = m_inv * pcs.point_clouds[1].m_pose;

			if (is_ndt) {
				NDT ndt;

				ndt.is_fix_first_node = true;
				ndt.is_gauss_newton = is_gauss_newton;
				ndt.is_levenberg_marguardt = is_levenberg_marguardt;
				ndt.is_wc = is_wc;
				ndt.is_cw = is_cw;
				ndt.is_tait_bryan_angles = is_tait_bryan_angles;
				ndt.is_quaternion = is_quaternion;
				ndt.is_rodrigues = is_rodrigues;

				ndt.bucket_size[0] = ndt_bucket_size[0];
				ndt.bucket_size[1] = ndt_bucket_size[1];
				ndt.bucket_size[2] = ndt_bucket_size[2];

				ndt.number_of_threads = number_of_threads;
				ndt.number_of_iterations = number_of_iterations_pair_wise_matching;

				ndt.optimize(pcs.point_clouds, true);
				
				edges[i].m_relative_pose = pcs.point_clouds[0].m_pose.inverse() * pcs.point_clouds[1].m_pose;
			}
			else if (is_optimization_point_to_point_source_to_target) {
				ICP icp;

				icp.search_radious = search_radious;
				icp.number_of_threads = number_of_threads;
				icp.number_of_iterations = number_of_iterations_pair_wise_matching;
				icp.is_adaptive_robust_kernel = is_adaptive_robust_kernel;

				icp.is_ballanced_horizontal_vs_vertical = true;
				icp.is_fix_first_node = is_fix_first_node;
				icp.is_gauss_newton = is_gauss_newton;
				icp.is_levenberg_marguardt = is_levenberg_marguardt;
				icp.is_cw = is_cw;
				icp.is_wc = is_wc;
				icp.is_tait_bryan_angles = is_tait_bryan_angles;
				icp.is_quaternion = is_quaternion;
				icp.is_rodrigues = is_rodrigues;

				icp.optimization_point_to_point_source_to_target(pcs);

				edges[i].m_relative_pose = pcs.point_clouds[0].m_pose.inverse() * pcs.point_clouds[1].m_pose;
			}
			else if (is_optimize_point_to_projection_onto_plane_source_to_target) {
				RegistrationPlaneFeature registration_plane_feature;

				registration_plane_feature.search_radious = search_radious;
				registration_plane_feature.number_of_threads = number_of_threads;
				registration_plane_feature.number_of_iterations = number_of_iterations_pair_wise_matching;
				registration_plane_feature.is_adaptive_robust_kernel = is_adaptive_robust_kernel;

				registration_plane_feature.is_gauss_newton = is_gauss_newton;
				registration_plane_feature.is_levenberg_marguardt = is_levenberg_marguardt;

				registration_plane_feature.is_wc = is_wc;
				registration_plane_feature.is_cw = is_cw;

				registration_plane_feature.is_tait_bryan_angles = is_tait_bryan_angles;
				registration_plane_feature.is_quaternion = is_quaternion;
				registration_plane_feature.is_rodrigues = is_rodrigues;

				registration_plane_feature.is_fix_first_node = is_fix_first_node;

				registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(pcs);

				edges[i].m_relative_pose = pcs.point_clouds[0].m_pose.inverse() * pcs.point_clouds[1].m_pose;
			}
			else if (is_optimize_point_to_projection_onto_plane_source_to_target) {
				RegistrationPlaneFeature registration_plane_feature;

				registration_plane_feature.search_radious = search_radious;
				registration_plane_feature.number_of_threads = number_of_threads;
				registration_plane_feature.number_of_iterations = number_of_iterations_pair_wise_matching;
				registration_plane_feature.is_adaptive_robust_kernel = is_adaptive_robust_kernel;

				registration_plane_feature.is_gauss_newton = is_gauss_newton;
				registration_plane_feature.is_levenberg_marguardt = is_levenberg_marguardt;

				registration_plane_feature.is_wc = is_wc;
				registration_plane_feature.is_cw = is_cw;

				registration_plane_feature.is_tait_bryan_angles = is_tait_bryan_angles;
				registration_plane_feature.is_quaternion = is_quaternion;
				registration_plane_feature.is_rodrigues = is_rodrigues;

				registration_plane_feature.is_fix_first_node = is_fix_first_node;

				registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(pcs);

				edges[i].m_relative_pose = pcs.point_clouds[0].m_pose.inverse() * pcs.point_clouds[1].m_pose;
			}
			else if (is_optimize_point_to_plane_source_to_target) {
				RegistrationPlaneFeature registration_plane_feature;

				registration_plane_feature.search_radious = search_radious;
				registration_plane_feature.number_of_threads = number_of_threads;
				registration_plane_feature.number_of_iterations = number_of_iterations_pair_wise_matching;
				registration_plane_feature.is_adaptive_robust_kernel = is_adaptive_robust_kernel;

				registration_plane_feature.is_gauss_newton = is_gauss_newton;
				registration_plane_feature.is_levenberg_marguardt = is_levenberg_marguardt;

				registration_plane_feature.is_wc = is_wc;
				registration_plane_feature.is_cw = is_cw;

				registration_plane_feature.is_tait_bryan_angles = is_tait_bryan_angles;
				registration_plane_feature.is_quaternion = is_quaternion;
				registration_plane_feature.is_rodrigues = is_rodrigues;

				registration_plane_feature.is_fix_first_node = is_fix_first_node;

				registration_plane_feature.optimize_point_to_plane_source_to_target(pcs);

				edges[i].m_relative_pose = pcs.point_clouds[0].m_pose.inverse() * pcs.point_clouds[1].m_pose;
			}
			else if (is_optimize_distance_point_to_plane_source_to_target) {
				RegistrationPlaneFeature registration_plane_feature;

				registration_plane_feature.search_radious = search_radious;
				registration_plane_feature.number_of_threads = number_of_threads;
				registration_plane_feature.number_of_iterations = number_of_iterations_pair_wise_matching;
				registration_plane_feature.is_adaptive_robust_kernel = is_adaptive_robust_kernel;

				registration_plane_feature.is_gauss_newton = is_gauss_newton;
				registration_plane_feature.is_levenberg_marguardt = is_levenberg_marguardt;

				registration_plane_feature.is_wc = is_wc;
				registration_plane_feature.is_cw = is_cw;

				registration_plane_feature.is_tait_bryan_angles = is_tait_bryan_angles;
				registration_plane_feature.is_quaternion = is_quaternion;
				registration_plane_feature.is_rodrigues = is_rodrigues;

				registration_plane_feature.is_fix_first_node = is_fix_first_node;

				registration_plane_feature.optimize_distance_point_to_plane_source_to_target(pcs);

				edges[i].m_relative_pose = pcs.point_clouds[0].m_pose.inverse() * pcs.point_clouds[1].m_pose;
			}
			else if (is_optimize_plane_to_plane_source_to_target) {
				RegistrationPlaneFeature registration_plane_feature;

				registration_plane_feature.search_radious = search_radious;
				registration_plane_feature.number_of_threads = number_of_threads;
				registration_plane_feature.number_of_iterations = number_of_iterations_pair_wise_matching;
				registration_plane_feature.is_adaptive_robust_kernel = is_adaptive_robust_kernel;

				registration_plane_feature.is_gauss_newton = is_gauss_newton;
				registration_plane_feature.is_levenberg_marguardt = is_levenberg_marguardt;

				registration_plane_feature.is_wc = is_wc;
				registration_plane_feature.is_cw = is_cw;

				registration_plane_feature.is_tait_bryan_angles = is_tait_bryan_angles;
				registration_plane_feature.is_quaternion = is_quaternion;
				registration_plane_feature.is_rodrigues = is_rodrigues;

				registration_plane_feature.is_fix_first_node = is_fix_first_node;

				registration_plane_feature.optimize_plane_to_plane_source_to_target(pcs);

				edges[i].m_relative_pose = pcs.point_clouds[0].m_pose.inverse() * pcs.point_clouds[1].m_pose;
            }
            else if (is_optimize_point_to_point_source_to_target_lie_algebra_left_jacobian) {
                ICP icp;

                icp.search_radious = search_radious;
                icp.number_of_threads = number_of_threads;
                icp.number_of_iterations = number_of_iterations_pair_wise_matching;
                icp.is_adaptive_robust_kernel = is_adaptive_robust_kernel;

                icp.is_ballanced_horizontal_vs_vertical = true;
                icp.is_fix_first_node = is_fix_first_node;
                icp.is_gauss_newton = is_gauss_newton;
                icp.is_levenberg_marguardt = is_levenberg_marguardt;
                icp.is_cw = is_cw;
                icp.is_wc = is_wc;
                icp.is_tait_bryan_angles = is_tait_bryan_angles;
                icp.is_quaternion = is_quaternion;
                icp.is_rodrigues = is_rodrigues;

                icp.optimize_source_to_target_lie_algebra_left_jacobian(pcs);

                edges[i].m_relative_pose = pcs.point_clouds[0].m_pose.inverse() * pcs.point_clouds[1].m_pose;
            }
            else if (is_optimize_point_to_point_source_to_target_lie_algebra_right_jacobian) {
                ICP icp;

                icp.search_radious = search_radious;
                icp.number_of_threads = number_of_threads;
                icp.number_of_iterations = number_of_iterations_pair_wise_matching;
                icp.is_adaptive_robust_kernel = is_adaptive_robust_kernel;

                icp.is_ballanced_horizontal_vs_vertical = true;
                icp.is_fix_first_node = is_fix_first_node;
                icp.is_gauss_newton = is_gauss_newton;
                icp.is_levenberg_marguardt = is_levenberg_marguardt;
                icp.is_cw = is_cw;
                icp.is_wc = is_wc;
                icp.is_tait_bryan_angles = is_tait_bryan_angles;
                icp.is_quaternion = is_quaternion;
                icp.is_rodrigues = is_rodrigues;

                icp.optimize_source_to_target_lie_algebra_right_jacobian(pcs);

                edges[i].m_relative_pose = pcs.point_clouds[0].m_pose.inverse() * pcs.point_clouds[1].m_pose;
            }
            else if (is_optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_left_jacobian) {
                RegistrationPlaneFeature registration_plane_feature;

                registration_plane_feature.search_radious = search_radious;
                registration_plane_feature.number_of_threads = number_of_threads;
                registration_plane_feature.number_of_iterations = number_of_iterations_pair_wise_matching;
                registration_plane_feature.is_adaptive_robust_kernel = is_adaptive_robust_kernel;

                registration_plane_feature.is_gauss_newton = is_gauss_newton;
                registration_plane_feature.is_levenberg_marguardt = is_levenberg_marguardt;

                registration_plane_feature.is_wc = is_wc;
                registration_plane_feature.is_cw = is_cw;

                registration_plane_feature.is_tait_bryan_angles = is_tait_bryan_angles;
                registration_plane_feature.is_quaternion = is_quaternion;
                registration_plane_feature.is_rodrigues = is_rodrigues;

                registration_plane_feature.is_fix_first_node = is_fix_first_node;

                registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_left_jacobian(pcs);

                edges[i].m_relative_pose = pcs.point_clouds[0].m_pose.inverse() * pcs.point_clouds[1].m_pose;
            }
            else if (is_optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_right_jacobian) {
                RegistrationPlaneFeature registration_plane_feature;

                registration_plane_feature.search_radious = search_radious;
                registration_plane_feature.number_of_threads = number_of_threads;
                registration_plane_feature.number_of_iterations = number_of_iterations_pair_wise_matching;
                registration_plane_feature.is_adaptive_robust_kernel = is_adaptive_robust_kernel;

                registration_plane_feature.is_gauss_newton = is_gauss_newton;
                registration_plane_feature.is_levenberg_marguardt = is_levenberg_marguardt;

                registration_plane_feature.is_wc = is_wc;
                registration_plane_feature.is_cw = is_cw;

                registration_plane_feature.is_tait_bryan_angles = is_tait_bryan_angles;
                registration_plane_feature.is_quaternion = is_quaternion;
                registration_plane_feature.is_rodrigues = is_rodrigues;

                registration_plane_feature.is_fix_first_node = is_fix_first_node;

                registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_right_jacobian(pcs);

                edges[i].m_relative_pose = pcs.point_clouds[0].m_pose.inverse() * pcs.point_clouds[1].m_pose;
            }else if (is_ndt_lie_algebra_left_jacobian) {
                NDT ndt;

                ndt.is_fix_first_node = true;
                ndt.is_gauss_newton = is_gauss_newton;
                ndt.is_levenberg_marguardt = is_levenberg_marguardt;
                ndt.is_wc = true;
                ndt.is_cw = false;
                ndt.is_tait_bryan_angles = false;
                ndt.is_quaternion = false;
                ndt.is_rodrigues = false;
                ndt.is_lie_algebra_left_jacobian = true;
                ndt.is_lie_algebra_right_jacobian = false;

                ndt.bucket_size[0] = ndt_bucket_size[0];
                ndt.bucket_size[1] = ndt_bucket_size[1];
                ndt.bucket_size[2] = ndt_bucket_size[2];

                ndt.number_of_threads = number_of_threads;
                ndt.number_of_iterations = number_of_iterations_pair_wise_matching;

                ndt.optimize(pcs.point_clouds, true);

                edges[i].m_relative_pose = pcs.point_clouds[0].m_pose.inverse() * pcs.point_clouds[1].m_pose;
            }
            else if (is_ndt_lie_algebra_right_jacobian) {
                NDT ndt;

                ndt.is_fix_first_node = true;
                ndt.is_gauss_newton = is_gauss_newton;
                ndt.is_levenberg_marguardt = is_levenberg_marguardt;
                ndt.is_wc = true;
                ndt.is_cw = false;
                ndt.is_tait_bryan_angles = false;
                ndt.is_quaternion = false;
                ndt.is_rodrigues = false;
                ndt.is_lie_algebra_left_jacobian = false;
                ndt.is_lie_algebra_right_jacobian = true;

                ndt.bucket_size[0] = ndt_bucket_size[0];
                ndt.bucket_size[1] = ndt_bucket_size[1];
                ndt.bucket_size[2] = ndt_bucket_size[2];

                ndt.number_of_threads = number_of_threads;
                ndt.number_of_iterations = number_of_iterations_pair_wise_matching;

                ndt.optimize(pcs.point_clouds, true);

                edges[i].m_relative_pose = pcs.point_clouds[0].m_pose.inverse() * pcs.point_clouds[1].m_pose;
            }
        }
    }

#ifdef WITH_PCL

    else if(pair_wise_matching_type == PairWiseMatchingType::pcl_ndt){
    	std::cout << "pair_wise_matching_type == PairWiseMatchingType::pcl_ndt" << std::endl;
    	for (size_t i = 0; i < edges.size(); i++) {
			std::cout << "PROGRESS calculating edges: " << i + 1 << " of " << edges.size() << "[" << edges[i].index_from << "," << edges[i].index_to << "]" << std::endl;

			pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			for(size_t j = 0; j < point_clouds[edges[i].index_to].points_local.size(); j++){
				Eigen::Vector3d p(point_clouds[edges[i].index_to].points_local[j].x(),
													point_clouds[edges[i].index_to].points_local[j].y(),
													point_clouds[edges[i].index_to].points_local[j].z());

				Eigen::Vector3d pt = point_clouds[edges[i].index_to].m_pose * p;
				pcl::PointXYZ p_pcl(pt.x(), pt.y(), pt.z());
				target_cloud->push_back(p_pcl);
			}
			//Eigen::Affine3d m_inv = point_clouds[edges[i].index_to].m_pose.inverse();
			//auto m_initial_pose = m_inv * point_clouds[edges[i].index_from].m_pose;

			pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			for(size_t j = 0; j < point_clouds[edges[i].index_from].points_local.size(); j++){
				Eigen::Vector3d p(point_clouds[edges[i].index_from].points_local[j].x(),
										point_clouds[edges[i].index_from].points_local[j].y(),
										point_clouds[edges[i].index_from].points_local[j].z());
				//Eigen::Vector3d pt = point_clouds[edges[i].index_from].m_pose * p;
				pcl::PointXYZ p_pcl(p.x(), p.y(), p.z());
				input_cloud->push_back(p_pcl);
			}
			//from https://pointclouds.org/documentation/tutorials/normal_distributions_transform.html
			// Filtering input scan to roughly 10% of original size to increase speed of registration.
			pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
			approximate_voxel_filter.setLeafSize (0.05, 0.05, 0.05);
			approximate_voxel_filter.setInputCloud (input_cloud);
			approximate_voxel_filter.filter (*filtered_cloud);

			// Initializing Normal Distributions Transform (NDT).
			pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
			// Setting scale dependent NDT parameters
			// Setting minimum transformation difference for termination condition.
			ndt.setTransformationEpsilon (0.01);
			// Setting maximum step size for More-Thuente line search.
			ndt.setStepSize (0.1);
			//Setting Resolution of NDT grid structure (VoxelGridCovariance).
			ndt.setResolution (ndt_bucket_size[0]);
			// Setting max number of registration iterations.
			ndt.setMaximumIterations (number_of_iterations_pair_wise_matching);
			// Setting point cloud to be aligned.
			ndt.setInputSource (filtered_cloud);
			// Setting point cloud to be aligned to.
			ndt.setInputTarget (target_cloud);

			Eigen::Matrix4f init_guess = point_clouds[edges[i].index_from].m_pose.matrix ().cast<float> ();


			// Calculating required rigid transform to align the input cloud to the target cloud.
			pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			ndt.align (*output_cloud, init_guess);
			std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
			          << " score: " << ndt.getFitnessScore () << std::endl;
			// Transforming unfiltered, input cloud using found transform.
			// pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());

			std::cout << "ndt.getFinalTransformation (): " << std::endl <<  ndt.getFinalTransformation () << std::endl;
			edges[i].m_relative_pose.matrix() = ndt.getFinalTransformation ().cast<double> ();
			edges[i].m_relative_pose = edges[i].m_relative_pose.inverse() * point_clouds[edges[i].index_to].m_pose;


			//ndt.getFinalTransformation ().inverse().matrix().cast<double> () * point_clouds[edges[i].index_to].m_pose;
    	}
    }
    else if(pair_wise_matching_type == PairWiseMatchingType::pcl_icp){
		std::cout << "pair_wise_matching_type == PairWiseMatchingType::pcl_icp" << std::endl;
		for (size_t i = 0; i < edges.size(); i++) {
			std::cout << "PROGRESS calculating edges: " << i + 1 << " of " << edges.size() << "[" << edges[i].index_from << "," << edges[i].index_to << "]" << std::endl;

			Eigen::Affine3d m_source = point_clouds[edges[i].index_from].m_pose;
			Eigen::Affine3d m_source_inv = m_source.inverse();
			Eigen::Affine3d m_target = point_clouds[edges[i].index_to].m_pose;
			Eigen::Affine3d mt = m_source_inv * m_target;

			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
			for(size_t j = 0; j < point_clouds[edges[i].index_to].points_local.size(); j++){
				Eigen::Vector3d p(point_clouds[edges[i].index_to].points_local[j].x(),
								  point_clouds[edges[i].index_to].points_local[j].y(),
								  point_clouds[edges[i].index_to].points_local[j].z());

				Eigen::Vector3d nv(point_clouds[edges[i].index_to].normal_vectors_local[j].x(),
								   point_clouds[edges[i].index_to].normal_vectors_local[j].y(),
								   point_clouds[edges[i].index_to].normal_vectors_local[j].z());

				Eigen::Vector3d pt = mt * p;
				Eigen::Vector3d nvt = mt.rotation() * nv;

				pcl::PointXYZRGBNormal p_pcl;
				p_pcl.x = pt.x();
				p_pcl.y = pt.y();
				p_pcl.z = pt.z();
				p_pcl.normal_x = nvt.x();
				p_pcl.normal_y = nvt.y();
				p_pcl.normal_z = nvt.z();

				target_cloud->push_back(p_pcl);
			}

			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
			for(size_t j = 0; j < point_clouds[edges[i].index_from].points_local.size(); j++){
				Eigen::Vector3d p(point_clouds[edges[i].index_from].points_local[j].x(),
						point_clouds[edges[i].index_from].points_local[j].y(),
						point_clouds[edges[i].index_from].points_local[j].z());

				pcl::PointXYZRGBNormal p_pcl;
				Eigen::Vector3d nv(point_clouds[edges[i].index_from].normal_vectors_local[j].x(),
								   point_clouds[edges[i].index_from].normal_vectors_local[j].y(),
								   point_clouds[edges[i].index_from].normal_vectors_local[j].z());

				p_pcl.x = p.x();
				p_pcl.y = p.y();
				p_pcl.z = p.z();

				p_pcl.normal_x = nv.x();
				p_pcl.normal_y = nv.y();
				p_pcl.normal_z = nv.z();

				input_cloud->push_back(p_pcl);
			}

			pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
			icp.setInputSource(input_cloud);
			icp.setInputTarget(target_cloud);
			icp.setMaximumIterations (number_of_iterations_pair_wise_matching) ;
			icp.setMaxCorrespondenceDistance(search_radious) ;

			pcl::PointCloud<pcl::PointXYZRGBNormal> Final;
			icp.align(Final);

			std::cout << "has converged:" << icp.hasConverged() << " score: " <<
			icp.getFitnessScore() << std::endl;
			std::cout << "icp.getFinalTransformation()" << std::endl;
			std::cout << icp.getFinalTransformation() << std::endl;

			Eigen::Affine3d m_res;
			m_res.matrix() = icp.getFinalTransformation().cast<double> ();

			//---
#if 0
			std::ofstream outfile;
			outfile.open("tmp.txt");
			if (!outfile.good()) {
				return ;
			}

			for(size_t j = 0; j < point_clouds[edges[i].index_to].points_local.size(); j++){
				Eigen::Vector3d p(point_clouds[edges[i].index_to].points_local[j].x(),
									point_clouds[edges[i].index_to].points_local[j].y(),
									point_clouds[edges[i].index_to].points_local[j].z());

				Eigen::Vector3d pt = mt * p;
				outfile << pt.x() << ";" << pt.y() << ";" << pt.z() << std::endl;
			}

			for(size_t j = 0; j < point_clouds[edges[i].index_from].points_local.size(); j++){
				Eigen::Vector3d p(point_clouds[edges[i].index_from].points_local[j].x(),
						point_clouds[edges[i].index_from].points_local[j].y(),
						point_clouds[edges[i].index_from].points_local[j].z());
				Eigen::Vector3d pt = m_res * p;
				outfile << pt.x() << ";" << pt.y() << ";" << pt.z() << std::endl;
			}

			outfile.close();
			exit(1);
#endif

			edges[i].m_relative_pose = m_res.inverse() * mt;
		}
	}
#endif

}

bool PoseGraphSLAM::optimize_with_GTSAM(PointClouds& point_clouds_container)
{
#if WITH_GTSAM
	int number_of_unknowns = 6;
	if (is_quaternion)number_of_unknowns = 7;

	for (auto& pc : point_clouds_container.point_clouds) {
		pc.build_rgd();
		pc.cout_rgd();
		pc.compute_normal_vectors(0.5);
	}

	//get edges based on overlap
	for (int i = 0; i < point_clouds_container.point_clouds.size(); i++) {
		for (int j = i + 1; j < point_clouds_container.point_clouds.size(); j++) {
			std::vector<std::pair<int, int>> nns = point_clouds_container.point_clouds[i].nns(point_clouds_container.point_clouds[j], 0.5);

			float overlap = float(nns.size()) / float(point_clouds_container.point_clouds[i].points_local.size());
			std::cout << "overlap: " << overlap << " between " << i << "," << j << std::endl;
			if (overlap > overlap_threshold) {
				Edge edge;
				edge.index_from = i;
				edge.index_to = j;
				edges.push_back(edge);
			}
		}
	}

	calculate_edges(point_clouds_container.point_clouds);

	//graph slam
	bool is_ok = true;
	std::vector<Eigen::Affine3d> m_poses;
	for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++) {
		m_poses.push_back(point_clouds_container.point_clouds[i].m_pose);
	}

	using namespace std;
	using namespace gtsam;
	NonlinearFactorGraph graph;

	auto priorModel = noiseModel::Diagonal::Variances(
	        (Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());

	graph.add(PriorFactor<Pose3>(0, Pose3(Pose3(m_poses[0].matrix())), priorModel));

	for(size_t i = 0; i <  edges.size(); i++){
		auto odometryNoise = noiseModel::Diagonal::Variances(
			            (Vector(6) << 1, 1, 1, 1, 1, 1).finished());

		Eigen::Matrix4d update = edges[i].m_relative_pose.matrix();
		graph.emplace_shared<BetweenFactor<Pose3> >(edges[i].index_from, edges[i].index_to, Pose3(update), odometryNoise);
	}

	Values initial;
	for (int i = 0; i < m_poses.size(); i++){
	    initial.insert(i, Pose3(m_poses[i].matrix()));
	}
	Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();

	for (int i =0; i < m_poses.size(); i++){
		auto v =  result.at<Pose3>(i);
		m_poses[i] = v.matrix();
	}

	  if (is_ok) {
		  for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++) {
			  point_clouds_container.point_clouds[i].m_pose = m_poses[i];
			  point_clouds_container.point_clouds[i].pose = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);
			  point_clouds_container.point_clouds[i].gui_translation[0] = point_clouds_container.point_clouds[i].pose.px;
			  point_clouds_container.point_clouds[i].gui_translation[1] = point_clouds_container.point_clouds[i].pose.py;
			  point_clouds_container.point_clouds[i].gui_translation[2] = point_clouds_container.point_clouds[i].pose.pz;
			  point_clouds_container.point_clouds[i].gui_rotation[0] = rad2deg(point_clouds_container.point_clouds[i].pose.om);
			  point_clouds_container.point_clouds[i].gui_rotation[1] = rad2deg(point_clouds_container.point_clouds[i].pose.fi);
			  point_clouds_container.point_clouds[i].gui_rotation[2] = rad2deg(point_clouds_container.point_clouds[i].pose.ka);
		  }
	  }

//clean
	  for (auto& pc : point_clouds_container.point_clouds) {
		  pc.clean();
	  }
	  edges.clear();
	  std::cout << "optimize_with_GTSAM DONE" << std::endl;
#endif
	  return true;
}

bool PoseGraphSLAM::optimize_with_manif(PointClouds& point_clouds_container)
{
#if WITH_MANIF
	using manif::SE3d;
	using manif::SE3Tangentd;

	int number_of_unknowns = 6;
	if (is_quaternion)number_of_unknowns = 7;

	for (auto& pc : point_clouds_container.point_clouds) {
		pc.build_rgd();
		pc.cout_rgd();
		pc.compute_normal_vectors(0.5);
	}

	//get edges based on overlap
	for (int i = 0; i < point_clouds_container.point_clouds.size(); i++) {
		for (int j = i + 1; j < point_clouds_container.point_clouds.size(); j++) {
			std::vector<std::pair<int, int>> nns = point_clouds_container.point_clouds[i].nns(point_clouds_container.point_clouds[j], 0.5);

			float overlap = float(nns.size()) / float(point_clouds_container.point_clouds[i].points_local.size());
			std::cout << "overlap: " << overlap << " between " << i << "," << j << std::endl;
			if (overlap > overlap_threshold) {
				Edge edge;
				edge.index_from = i;
				edge.index_to = j;
				edges.push_back(edge);
			}
		}
	}

	calculate_edges(point_clouds_container.point_clouds);


	//graph slam
	bool is_ok = true;
	std::vector<Eigen::Affine3d> m_poses;
	for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++) {
		m_poses.push_back(point_clouds_container.point_clouds[i].m_pose);
	}
	
	std::vector<Eigen::Triplet<double>> tripletListA;
	std::vector<Eigen::Triplet<double>> tripletListP;
	std::vector<Eigen::Triplet<double>> tripletListB;

	std::vector<SE3d> X;

	for(size_t i = 0 ; i < m_poses.size(); i++){
		Eigen::Vector3d t(m_poses[i](0,3), m_poses[i](1,3), m_poses[i](2,3));
		Eigen::Quaterniond q(m_poses[i].rotation());
		X.push_back(SE3d(t,q));
	}

	for(size_t i = 0 ; i < edges.size(); i++){
		const int first = edges[i].index_from;
		const int second = edges[i].index_to;
		const Eigen::Affine3d& rel = edges[i].m_relative_pose;
		Eigen::Vector3d t(rel(0,3), rel(1,3), rel(2,3));
		Eigen::Quaterniond q(rel.rotation());
		SE3d U = SE3d(t,q);

		SE3Tangentd     d;
		SE3Tangentd     u;
		Eigen::Matrix<double, 6, 6>         J_d_xi, J_d_xj;

		SE3d         Xi,
					 Xj;

		Xi = X[first];
		Xj = X[second];

		d  = Xj.rminus(Xi, J_d_xj, J_d_xi);
		u = U.log();

		int ir = tripletListB.size();
		int ic_1 = first * 6;
		int ic_2 = second * 6;

		for(size_t row = 0 ; row < 6; row ++){
			tripletListA.emplace_back(ir + row, ic_1    , -J_d_xi(row,0));
			tripletListA.emplace_back(ir + row, ic_1 + 1, -J_d_xi(row,1));
			tripletListA.emplace_back(ir + row, ic_1 + 2, -J_d_xi(row,2));
			tripletListA.emplace_back(ir + row, ic_1 + 3, -J_d_xi(row,3));
			tripletListA.emplace_back(ir + row, ic_1 + 4, -J_d_xi(row,4));
			tripletListA.emplace_back(ir + row, ic_1 + 5, -J_d_xi(row,5));

			tripletListA.emplace_back(ir + row, ic_2    , -J_d_xj(row,0));
			tripletListA.emplace_back(ir + row, ic_2 + 1, -J_d_xj(row,1));
			tripletListA.emplace_back(ir + row, ic_2 + 2, -J_d_xj(row,2));
			tripletListA.emplace_back(ir + row, ic_2 + 3, -J_d_xj(row,3));
			tripletListA.emplace_back(ir + row, ic_2 + 4, -J_d_xj(row,4));
			tripletListA.emplace_back(ir + row, ic_2 + 5, -J_d_xj(row,5));
		}

		SE3Tangentd delta = d - u;

		tripletListB.emplace_back(ir,     0, delta.coeffs()(0));
		tripletListB.emplace_back(ir + 1, 0, delta.coeffs()(1));
		tripletListB.emplace_back(ir + 2, 0, delta.coeffs()(2));
		tripletListB.emplace_back(ir + 3, 0, delta.coeffs()(3));
		tripletListB.emplace_back(ir + 4, 0, delta.coeffs()(4));
		tripletListB.emplace_back(ir + 5, 0, delta.coeffs()(5));

		tripletListP.emplace_back(ir ,    ir,     1);
		tripletListP.emplace_back(ir + 1, ir + 1, 1);
		tripletListP.emplace_back(ir + 2, ir + 2, 1);
		tripletListP.emplace_back(ir + 3, ir + 3, 1);
		tripletListP.emplace_back(ir + 4, ir + 4, 1);
		tripletListP.emplace_back(ir + 5, ir + 5, 1);
	}

	int ir = tripletListB.size();
	tripletListA.emplace_back(ir     , 0, 1);
	tripletListA.emplace_back(ir + 1 , 1, 1);
	tripletListA.emplace_back(ir + 2 , 2, 1);
	tripletListA.emplace_back(ir + 3 , 3, 1);
	tripletListA.emplace_back(ir + 4 , 4, 1);
	tripletListA.emplace_back(ir + 5 , 5, 1);

	tripletListP.emplace_back(ir     , ir,     1000000);
	tripletListP.emplace_back(ir + 1 , ir + 1, 1000000);
	tripletListP.emplace_back(ir + 2 , ir + 2, 1000000);
	tripletListP.emplace_back(ir + 3 , ir + 3, 1000000);
	tripletListP.emplace_back(ir + 4 , ir + 4, 1000000);
	tripletListP.emplace_back(ir + 5 , ir + 5, 1000000);

	tripletListB.emplace_back(ir     , 0, 0);
	tripletListB.emplace_back(ir + 1 , 0, 0);
	tripletListB.emplace_back(ir + 2 , 0, 0);
	tripletListB.emplace_back(ir + 3 , 0, 0);
	tripletListB.emplace_back(ir + 4 , 0, 0);
	tripletListB.emplace_back(ir + 5 , 0, 0);

	Eigen::SparseMatrix<double> matA(tripletListB.size(), m_poses.size() * 6);
	Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
	Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

	matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
	matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
	matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

	Eigen::SparseMatrix<double> AtPA(m_poses.size() * 6 , m_poses.size() * 6);
	Eigen::SparseMatrix<double> AtPB(m_poses.size() * 6 , 1);

	{
	Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
	AtPA = (AtP) * matA;
	AtPB = (AtP) * matB;
	}

	tripletListA.clear();
	tripletListP.clear();
	tripletListB.clear();

	std::cout << "start solving AtPA=AtPB" << std::endl;
	Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);

	std::cout << "x = solver.solve(AtPB)" << std::endl;
	Eigen::SparseMatrix<double> x = solver.solve(AtPB);

	std::vector<double> h_x;

	for (int k=0; k<x.outerSize(); ++k){
		for (Eigen::SparseMatrix<double>::InnerIterator it(x,k); it; ++it){
			h_x.push_back(it.value());
		}
	}

	if(X.size() * 6 == h_x.size()){

		int counter = 0;
		for(size_t i = 0 ; i < X.size(); i++){
			SE3Tangentd     dx;
			dx.coeffs()(0) = h_x[counter++];
			dx.coeffs()(1) = h_x[counter++];
			dx.coeffs()(2) = h_x[counter++];
			dx.coeffs()(3) = h_x[counter++];
			dx.coeffs()(4) = h_x[counter++];
			dx.coeffs()(5) = h_x[counter++];

			X[i] = X[i] +  dx;
		}

		for (int i = 0 ; i < m_poses.size(); i++){
			m_poses[i](0,0) = X[i].rotation()(0,0);
			m_poses[i](1,0) = X[i].rotation()(1,0);
			m_poses[i](2,0) = X[i].rotation()(2,0);

			m_poses[i](0,1) = X[i].rotation()(0,1);
			m_poses[i](1,1) = X[i].rotation()(1,1);
			m_poses[i](2,1) = X[i].rotation()(2,1);

			m_poses[i](0,2) = X[i].rotation()(0,2);
			m_poses[i](1,2) = X[i].rotation()(1,2);
			m_poses[i](2,2) = X[i].rotation()(2,2);

			m_poses[i](0,3) = X[i].translation()(0);
			m_poses[i](1,3) = X[i].translation()(1);
			m_poses[i](2,3) = X[i].translation()(2);
		}
		std::cout << "AtPA=AtPB SOLVED" << std::endl;
	}else{
		std::cout << "h_x.size(): " << h_x.size() << std::endl;
		std::cout << "AtPA=AtPB FAILED" << std::endl;
	}

	if (is_ok) {
	  for (size_t i = 0; i < point_clouds_container.point_clouds.size(); i++) {
		  point_clouds_container.point_clouds[i].m_pose = m_poses[i];
		  point_clouds_container.point_clouds[i].pose = pose_tait_bryan_from_affine_matrix(point_clouds_container.point_clouds[i].m_pose);
		  point_clouds_container.point_clouds[i].gui_translation[0] = point_clouds_container.point_clouds[i].pose.px;
		  point_clouds_container.point_clouds[i].gui_translation[1] = point_clouds_container.point_clouds[i].pose.py;
		  point_clouds_container.point_clouds[i].gui_translation[2] = point_clouds_container.point_clouds[i].pose.pz;
		  point_clouds_container.point_clouds[i].gui_rotation[0] = rad2deg(point_clouds_container.point_clouds[i].pose.om);
		  point_clouds_container.point_clouds[i].gui_rotation[1] = rad2deg(point_clouds_container.point_clouds[i].pose.fi);
		  point_clouds_container.point_clouds[i].gui_rotation[2] = rad2deg(point_clouds_container.point_clouds[i].pose.ka);
	  }
	}

	//clean
	for (auto& pc : point_clouds_container.point_clouds) {
	  pc.clean();
	}
	edges.clear();
	std::cout << "optimize_with_manif DONE" << std::endl;
#endif
	return true;
}

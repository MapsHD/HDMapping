#ifndef _POSE_GRAPH_SLAM_H_
#define _POSE_GRAPH_SLAM_H_

#include <point_clouds.h>
#include <icp.h>

class PoseGraphSLAM {
public:
	enum PairWiseMatchingType{
		general,
		pcl_ndt,
		pcl_icp
	};

	struct Edge {
		int index_from;
		int index_to;
		Eigen::Affine3d m_relative_pose;
		Eigen::SparseMatrix<double> information_matrix;
	};

	PoseGraphSLAM() { ndt_bucket_size[0] = 1.0; ndt_bucket_size[1] = 1.0; ndt_bucket_size[2] = 1.0;/*icp.is_adaptive_robust_kernel = false;*/ };
	~PoseGraphSLAM() { ; };

	float overlap_threshold = 0.3;
	int iterations = 6;
	std::vector<Edge> edges;

	float search_radious = 0.1;
	int number_of_threads = 16;
	int number_of_iterations_pair_wise_matching = 6;

	//--
	bool is_adaptive_robust_kernel = false;
	bool is_fix_first_node = false;
	bool is_gauss_newton = true;
	bool is_levenberg_marguardt = false;
	bool is_cw = false;
	bool is_wc = true;
	bool is_tait_bryan_angles = true;
	bool is_quaternion = false;
	bool is_rodrigues = false;
	//--

	bool is_ndt = true;
	bool is_optimization_point_to_point_source_to_target = false;
	bool is_optimize_point_to_projection_onto_plane_source_to_target = false;
	bool is_optimize_point_to_plane_source_to_target = false;
	bool is_optimize_distance_point_to_plane_source_to_target = false;
	bool is_optimize_plane_to_plane_source_to_target = false;
	bool is_optimize_pcl_ndt = false;
	bool is_optimize_pcl_icp = false;
	bool is_ndt_lie_algebra_left_jacobian = false;
	bool is_ndt_lie_algebra_right_jacobian = false;
	bool is_optimize_point_to_point_source_to_target_lie_algebra_left_jacobian = false;
	bool is_optimize_point_to_point_source_to_target_lie_algebra_right_jacobian = false;
	bool is_optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_left_jacobian = false;
	bool is_optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_right_jacobian = false;
	bool is_lie_algebra_left_jacobian = false;
	bool is_lie_algebra_right_jacobian = false;


	float ndt_bucket_size[3];

	PairWiseMatchingType pair_wise_matching_type = PairWiseMatchingType::general;

	void add_random_noise(PointClouds& point_clouds_container);

	//bool optimize(PointClouds& point_clouds_container, double& rms_initial, double& rms_final, double& mui);
	bool optimize(PointClouds& point_clouds_container);
	std::vector<Eigen::SparseMatrix<double>> compute_covariance_matrices_and_rms(std::vector<PointCloud>& point_clouds, double& rms);

	void calculate_edges(std::vector<PointCloud>& point_clouds);

	bool optimize_with_GTSAM(PointClouds& point_clouds_container);
	bool optimize_with_manif(PointClouds& point_clouds_container);

	void set_all_to_false() {
		is_ndt_lie_algebra_left_jacobian = false;
		is_ndt_lie_algebra_right_jacobian = false;
		is_ndt = false;
		is_optimization_point_to_point_source_to_target = false;
		is_optimize_point_to_projection_onto_plane_source_to_target = false;
		is_optimize_point_to_plane_source_to_target = false;
		is_optimize_distance_point_to_plane_source_to_target = false;
		is_optimize_plane_to_plane_source_to_target = false;
		is_optimize_pcl_ndt = false;
		is_optimize_pcl_icp = false;
		is_optimize_point_to_point_source_to_target_lie_algebra_left_jacobian = false;
		is_optimize_point_to_point_source_to_target_lie_algebra_right_jacobian = false;
		is_optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_left_jacobian = false;
		is_optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_right_jacobian = false;
		is_lie_algebra_left_jacobian = false;
		is_lie_algebra_right_jacobian = false;
	}
};


#endif

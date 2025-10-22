#ifndef _REGISTRATION_PLANE_FEATURE_H_
#define _REGISTRATION_PLANE_FEATURE_H_

#include <point_clouds.h>

class RegistrationPlaneFeature {
public:
	enum PoseConvention {
		cw,
		wc
	};
	enum OptimizationAlgorithm {
		gauss_newton,
		levenberg_marguardt
	};
	enum RotationMatrixParametrization {
		tait_bryan_xyz,
		rodrigues,
		quaternion
	};

	struct Plane {
		Plane() {
			a = b = c = d = 0.0;
		}
		double a;
		double b;
		double c;
		double d;

		Plane(Eigen::Vector3d p, Eigen::Vector3d nv) {
			a = nv.x();
			b = nv.y();
			c = nv.z();
			d = -a * p.x() - b * p.y() - c * p.z();
		}
	};

	struct Job {
		long long unsigned int index_begin_inclusive;
		long long unsigned int index_end_exclusive;
	};
	std::vector<Job> get_jobs(long long unsigned int size, int num_threads = 8);

	RegistrationPlaneFeature() {
		search_radius = 0.1;
		number_of_threads = 16;
		number_of_iterations = 6;
		is_adaptive_robust_kernel = false;
		barron_c = 1.0;
	};
	~RegistrationPlaneFeature() {};
		
	//optimize_point_to_projection_onto_plane_source_to_target
	bool optimize_point_to_projection_onto_plane_source_to_target(PointClouds& point_clouds_container);
	bool optimize_point_to_projection_onto_plane_source_to_target(PointClouds& point_clouds_container, 
		PoseConvention pose_convention, OptimizationAlgorithm optimization_algorithm, RotationMatrixParametrization rotation_matrix_parametrization);
	std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> compute_covariance_matrices_tait_bryan_point_to_projection_onto_plane_source_to_target(
		PointClouds& point_clouds_container, PoseConvention pose_convention);
	std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> compute_covariance_matrices_rodrigues_point_to_projection_onto_plane_source_to_target(
		PointClouds& point_clouds_container, PoseConvention pose_convention);
	std::vector<Eigen::Matrix<double, 7, 7, Eigen::RowMajor>> compute_covariance_matrices_quaternion_point_to_projection_onto_plane_source_to_target(
		PointClouds& point_clouds_container, PoseConvention pose_convention);
	
	bool optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_left_jacobian(PointClouds& point_clouds_container);
	bool optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_right_jacobian(PointClouds& point_clouds_container);




	//optimize_point_to_plane_source_to_target
	bool optimize_point_to_plane_source_to_target(PointClouds& point_clouds_container);
	bool optimize_point_to_plane_source_to_target(PointClouds& point_clouds_container,
		PoseConvention pose_convention, OptimizationAlgorithm optimization_algorithm, RotationMatrixParametrization rotation_matrix_parametrization);
	std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> compute_covariance_matrices_tait_bryan_point_to_plane_source_to_target(
		PointClouds& point_clouds_container, PoseConvention pose_convention);
	std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> compute_covariance_matrices_rodrigues_point_to_plane_source_to_target(
		PointClouds& point_clouds_container, PoseConvention pose_convention);
	std::vector<Eigen::Matrix<double, 7, 7, Eigen::RowMajor>> compute_covariance_matrices_quaternion_point_to_plane_source_to_target(
		PointClouds& point_clouds_container, PoseConvention pose_convention);
	//optimize_point_to_plane_source_to_target

	//optimize_distance_point_to_plane_source_to_target
	bool optimize_distance_point_to_plane_source_to_target(PointClouds& point_clouds_container);
	bool optimize_distance_point_to_plane_source_to_target(PointClouds& point_clouds_container, 
		PoseConvention pose_convention, OptimizationAlgorithm optimization_algorithm, RotationMatrixParametrization rotation_matrix_parametrization);
	std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> compute_covariance_matrices_tait_bryan_distance_point_to_plane_source_to_target(
		PointClouds& point_clouds_container, PoseConvention pose_convention);
	std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> compute_covariance_matrices_rodrigues_distance_point_to_plane_source_to_target(
		PointClouds& point_clouds_container, PoseConvention pose_convention);
	std::vector<Eigen::Matrix<double, 7, 7, Eigen::RowMajor>> compute_covariance_matrices_quaternion_distance_point_to_plane_source_to_target(
		PointClouds& point_clouds_container, PoseConvention pose_convention);
	
	//optimize_plane_to_plane_source_to_target
	bool optimize_plane_to_plane_source_to_target(PointClouds& point_clouds_container);
	std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> compute_covariance_matrices_tait_bryan_plane_to_plane_source_to_target(
		PointClouds& point_clouds_container, PoseConvention pose_convention);
	std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> compute_covariance_matrices_rodrigues_plane_to_plane_source_to_target(
		PointClouds& point_clouds_container, PoseConvention pose_convention);
	std::vector<Eigen::Matrix<double, 7, 7, Eigen::RowMajor>> compute_covariance_matrices_quaternion_plane_to_plane_source_to_target(
		PointClouds& point_clouds_container, PoseConvention pose_convention);
	bool optimize_plane_to_plane_source_to_target(PointClouds& point_clouds_container,
		PoseConvention pose_convention, OptimizationAlgorithm optimization_algorithm, RotationMatrixParametrization rotation_matrix_parametrization);

	float search_radius = 0.1;
	int number_of_threads = 16;
	int number_of_iterations = 6;
	bool is_adaptive_robust_kernel = true;
	double barron_c = 1.0;

	//bool is_newton = false;
	bool is_gauss_newton = true;
	bool is_levenberg_marguardt = false;

	bool is_wc = true;
	bool is_cw = false;

	bool is_tait_bryan_angles = true;
	bool is_quaternion = false;
	bool is_rodrigues = false;

	bool is_fix_first_node = false;

	bool is_lie_algebra_left_jacobian = false;
	bool is_lie_algebra_right_jacobian = false;
};




#endif
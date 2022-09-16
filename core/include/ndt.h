#ifndef _NDT_H_
#define _NDT_H_

#include <point_cloud.h>
#include <Eigen/Eigen>

class NDT {
public:

	struct GridParameters {
		double bounding_box_min_X;
		double bounding_box_min_Y;
		double bounding_box_min_Z;
		double bounding_box_max_X;
		double bounding_box_max_Y;
		double bounding_box_max_Z;
		double bounding_box_extension;
		int number_of_buckets_X;
		int number_of_buckets_Y;
		int number_of_buckets_Z;
		long long unsigned int number_of_buckets;
		double resolution_X;
		double resolution_Y;
		double resolution_Z;
	};

	struct PointBucketIndexPair {
		int index_of_point;
		long long unsigned int index_of_bucket;
		int index_pose;
	};

	struct Bucket {
		long long unsigned int index_begin;
		long long unsigned int index_end;
		long long unsigned int number_of_points;
		Eigen::Vector3d mean;
		Eigen::Matrix3d cov;
	};

	struct Job {
		long long unsigned int index_begin_inclusive;
		long long unsigned int index_end_exclusive;
	};

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
		quaternion,
		lie_algebra_left_jacobian,
		lie_algebra_right_jacobian
	};

	NDT() {
		bucket_size[0] = 0.5;
		bucket_size[1] = 0.5;
		bucket_size[2] = 0.5;
		number_of_threads = 16;
		number_of_iterations = 6;
	};
	~NDT() { ; };

	
	void grid_calculate_params(const std::vector<Point3D>& point_cloud_global, GridParameters& in_out_params);
	void build_rgd(std::vector<Point3D>& points, std::vector<PointBucketIndexPair>& index_pair, std::vector<Bucket>& buckets, GridParameters& rgd_params, int num_threads = 8);
	std::vector<Job> get_jobs(long long unsigned int size, int num_threads = 8);
	void reindex(std::vector<Point3D>& points, std::vector<NDT::PointBucketIndexPair>& index_pair, NDT::GridParameters& rgd_params, int num_threads);

	bool optimize(std::vector<PointCloud>& point_clouds);
	std::vector<Eigen::SparseMatrix<double>> compute_covariance_matrices_and_rms(std::vector<PointCloud>& point_clouds, double& rms);
	
	bool optimize(std::vector<PointCloud>& point_clouds, double& rms_initial, double& rms_final, double& mui);
	
	bool optimize_lie_algebra_left_jacobian(std::vector<PointCloud>& point_clouds);
	bool optimize_lie_algebra_right_jacobian(std::vector<PointCloud>& point_clouds);
	//std::vector<Eigen::Matrix<double, 7, 7, Eigen::RowMajor>> compute_covariance_matrices7x7(PointClouds& point_clouds_container);
	
	float bucket_size[3];
	int number_of_threads;
	int number_of_iterations;

	bool is_fix_first_node = false;
	bool is_gauss_newton = true;
	bool is_levenberg_marguardt = false;
	bool is_wc = true;
	bool is_cw = false;
	bool is_tait_bryan_angles = true;
	bool is_quaternion = false;
	bool is_rodrigues = false;
	bool is_lie_algebra_left_jacobian = false;
	bool is_lie_algebra_right_jacobian = false;
};

#endif
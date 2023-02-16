#include <ndt.h>
#include <structures.h>
//#include <Eigen\Eigen>
#include <vector>
#include <thread>
#include <iostream>

#include <point_clouds.h>

#include <transformations.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_cw_jacobian.h>

#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_rodrigues_wc_jacobian.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_rodrigues_cw_jacobian.h>

#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_quaternion_wc_jacobian.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_quaternion_cw_jacobian.h>
#include <python-scripts/constraints/quaternion_constraint_jacobian.h>

void NDT::grid_calculate_params(const std::vector<Point3D>& point_cloud_global, GridParameters& in_out_params)
{
	double min_x = std::numeric_limits<double>::max();
	double max_x = std::numeric_limits<double>::lowest();

	double min_y = std::numeric_limits<double>::max();
	double max_y = std::numeric_limits<double>::lowest();

	double min_z = std::numeric_limits<double>::max();
	double max_z = std::numeric_limits<double>::lowest();

	for (size_t i = 0; i < point_cloud_global.size(); i++) {
		if (point_cloud_global[i].x < min_x) min_x = point_cloud_global[i].x;
		if (point_cloud_global[i].x > max_x) max_x = point_cloud_global[i].x;

		if (point_cloud_global[i].y < min_y) min_y = point_cloud_global[i].y;
		if (point_cloud_global[i].y > max_y) max_y = point_cloud_global[i].y;

		if (point_cloud_global[i].z < min_z) min_z = point_cloud_global[i].z;
		if (point_cloud_global[i].z > max_z) max_z = point_cloud_global[i].z;
	}

	if (min_x < -200) min_x = -500;
	if (max_x > 200) max_x = 500;
	
	if (min_y < -200) min_y = -500;
	if (max_y > 200) max_y = 500;

	if (min_z < -200) min_z = -100;
	if (max_z > 200) max_z = 100;

	long long unsigned int number_of_buckets_X = ((max_x - min_x) / in_out_params.resolution_X) + 1;
	long long unsigned int number_of_buckets_Y = ((max_y - min_y) / in_out_params.resolution_Y) + 1;
	long long unsigned int number_of_buckets_Z = ((max_z - min_z) / in_out_params.resolution_Z) + 1;

	in_out_params.number_of_buckets_X = number_of_buckets_X;
	in_out_params.number_of_buckets_Y = number_of_buckets_Y;
	in_out_params.number_of_buckets_Z = number_of_buckets_Z;
	in_out_params.number_of_buckets = static_cast<long long unsigned int>(number_of_buckets_X) *
		static_cast<long long unsigned int>(number_of_buckets_Y) * static_cast<long long unsigned int>(number_of_buckets_Z);

	in_out_params.bounding_box_max_X = max_x;
	in_out_params.bounding_box_min_X = min_x;
	in_out_params.bounding_box_max_Y = max_y;
	in_out_params.bounding_box_min_Y = min_y;
	in_out_params.bounding_box_max_Z = max_z;
	in_out_params.bounding_box_min_Z = min_z;
}

std::vector<NDT::Job> NDT::get_jobs(long long unsigned int size, int num_threads) {

	int hc = size / num_threads;
	if (hc < 1)hc = 1;

	std::vector<Job> jobs;
	for (long long unsigned int i = 0; i < size; i += hc) {
		long long unsigned int sequence_length = hc;
		if (i + hc >= size) {
			sequence_length = size - i;
		}
		if (sequence_length == 0)break;

		Job j;
		j.index_begin_inclusive = i;
		j.index_end_exclusive = i + sequence_length;
		jobs.push_back(j);
	}

	return jobs;
}

void reindex_job(int i, NDT::Job* job, std::vector<Point3D>* points, std::vector<NDT::PointBucketIndexPair>* pairs, NDT::GridParameters rgd_params) {
	for (size_t ii = job->index_begin_inclusive; ii < job->index_end_exclusive; ii++) {

		Point3D& p = (*points)[ii];

		(*pairs)[ii].index_of_point = ii;
		(*pairs)[ii].index_of_bucket = 0;
		(*pairs)[ii].index_pose = p.index_pose;

		if (p.x < rgd_params.bounding_box_min_X) {
			continue;
		}
		if (p.x > rgd_params.bounding_box_max_X) {
			continue;
		}
		if (p.y < rgd_params.bounding_box_min_Y) {
			continue;
		}
		if (p.y > rgd_params.bounding_box_max_Y) {
			continue;
		}
		if (p.z < rgd_params.bounding_box_min_Z) {
			continue;
		}
		if (p.z > rgd_params.bounding_box_max_Z) {
			continue;
		}

		long long unsigned int ix = (p.x - rgd_params.bounding_box_min_X) / rgd_params.resolution_X;
		long long unsigned int iy = (p.y - rgd_params.bounding_box_min_Y) / rgd_params.resolution_Y;
		long long unsigned int iz = (p.z - rgd_params.bounding_box_min_Z) / rgd_params.resolution_Z;

		(*pairs)[ii].index_of_bucket = ix * static_cast<long long unsigned int>(rgd_params.number_of_buckets_Y) *
			static_cast<long long unsigned int>(rgd_params.number_of_buckets_Z) + iy * static_cast<long long unsigned int>(rgd_params.number_of_buckets_Z) + iz;
	}
}


void NDT::reindex(std::vector<Point3D>& points, std::vector<NDT::PointBucketIndexPair>& index_pair, NDT::GridParameters& rgd_params, int num_threads)
{
	index_pair.resize(points.size());

	std::vector<NDT::Job> jobs = get_jobs(index_pair.size(), num_threads);

	std::vector<std::thread> threads;

	for (size_t i = 0; i < jobs.size(); i++) {
		threads.push_back(std::thread(reindex_job, i, &jobs[i], &points, &index_pair, rgd_params));
	}

	for (size_t j = 0; j < threads.size(); j++) {
		threads[j].join();
	}
	threads.clear();

	std::sort(index_pair.begin(), index_pair.end(), [](const NDT::PointBucketIndexPair& a, const NDT::PointBucketIndexPair& b) { return ((a.index_of_bucket == b.index_of_bucket) ? (a.index_pose < b.index_pose) : (a.index_of_bucket < b.index_of_bucket)); });
}

void build_rgd_init_job(int i, NDT::Job* job, std::vector<NDT::Bucket>* buckets) {

	for (size_t ii = job->index_begin_inclusive; ii < job->index_end_exclusive; ii++) {
		(*buckets)[ii].index_begin = -1;
		(*buckets)[ii].index_end = -1;
		(*buckets)[ii].number_of_points = 0;
	}
}

void build_rgd_job(int i, NDT::Job* job, std::vector<NDT::PointBucketIndexPair>* index_pair, std::vector<NDT::Bucket>* buckets) {
	for (size_t ii = job->index_begin_inclusive; ii < job->index_end_exclusive; ii++) {
		int ind = ii;
		if (ind == 0)
		{
			long long unsigned int index_of_bucket = (*index_pair)[ind].index_of_bucket;
			long long unsigned int index_of_bucket_1 = (*index_pair)[ind + 1].index_of_bucket;

			(*buckets)[index_of_bucket].index_begin = ind;
			if (index_of_bucket != index_of_bucket_1)
			{
				(*buckets)[index_of_bucket].index_end = ind + 1;
				(*buckets)[index_of_bucket_1].index_end = ind + 1;
			}
		}
		else if (ind == (*buckets).size() - 1)
		{
			if ((*index_pair)[ind].index_of_bucket < (*buckets).size()) {
				(*buckets)[(*index_pair)[ind].index_of_bucket].index_end = ind + 1;
			}
		}
		else if (ind + 1 < (*index_pair).size())
		{
			long long unsigned int index_of_bucket = (*index_pair)[ind].index_of_bucket;
			long long unsigned int index_of_bucket_1 = (*index_pair)[ind + 1].index_of_bucket;

			if (index_of_bucket != index_of_bucket_1)
			{
				(*buckets)[index_of_bucket].index_end = ind + 1;
				(*buckets)[index_of_bucket_1].index_begin = ind + 1;
			}
		}
	}
}

void build_rgd_final_job(int i, NDT::Job* job, std::vector<NDT::Bucket>* buckets) {
	for (size_t ii = job->index_begin_inclusive; ii < job->index_end_exclusive; ii++) {
		long long unsigned int index_begin = (*buckets)[ii].index_begin;
		long long unsigned int index_end = (*buckets)[ii].index_end;
		if (index_begin != -1 && index_end != -1)
		{
			(*buckets)[ii].number_of_points = index_end - index_begin;
		}
	}
}

void NDT::build_rgd(std::vector<Point3D>& points, std::vector<NDT::PointBucketIndexPair>& index_pair, std::vector<NDT::Bucket>& buckets, NDT::GridParameters& rgd_params, int num_threads)
{
	if (num_threads < 1)num_threads = 1;

	index_pair.resize(points.size());
	reindex(points, index_pair, rgd_params, num_threads);

	buckets.resize(rgd_params.number_of_buckets);

	std::vector<NDT::Job> jobs = get_jobs(buckets.size(), num_threads);
	std::vector<std::thread> threads;

	for (size_t i = 0; i < jobs.size(); i++) {
		threads.push_back(std::thread(build_rgd_init_job, i, &jobs[i], &buckets));
	}

	for (size_t j = 0; j < threads.size(); j++) {
		threads[j].join();
	}
	threads.clear();

	jobs = get_jobs(points.size(), num_threads);

	for (size_t i = 0; i < jobs.size(); i++) {
		threads.push_back(std::thread(build_rgd_job, i, &jobs[i], &index_pair, &buckets));
	}
	for (size_t j = 0; j < threads.size(); j++) {
		threads[j].join();
	}
	threads.clear();


	jobs = get_jobs(buckets.size(), num_threads);

	for (size_t i = 0; i < jobs.size(); i++) {
		threads.push_back(std::thread(build_rgd_final_job, i, &jobs[i], &buckets));
	}

	for (size_t j = 0; j < threads.size(); j++) {
		threads[j].join();
	}
	threads.clear();
}

void ndt_job(int i, NDT::Job* job, std::vector<NDT::Bucket>* buckets, Eigen::SparseMatrix<double >* AtPA,
	Eigen::SparseMatrix<double >* AtPB, std::vector<NDT::PointBucketIndexPair>* index_pair_internal, std::vector<Point3D>* pp,
	std::vector<Eigen::Affine3d>* mposes, std::vector<Eigen::Affine3d>* mposes_inv, size_t trajectory_size,
	NDT::PoseConvention pose_convention, NDT::RotationMatrixParametrization rotation_matrix_parametrization, int number_of_unknowns, double* sumssr, int* sums_obs) {

	
	std::vector<Eigen::Triplet<double>> tripletListA;
	std::vector<Eigen::Triplet<double>> tripletListP;
	std::vector<Eigen::Triplet<double>> tripletListB;

	double ssr = 0.0;
	int sum_obs = 0;

	for (size_t ii = job->index_begin_inclusive; ii < job->index_end_exclusive; ii++) {
		NDT::Bucket& b = (*buckets)[ii];
		if (b.number_of_points < 5)continue;

		Eigen::Vector3d mean(0, 0, 0);
		Eigen::Matrix3d cov;
		cov.setZero();

		for (int index = b.index_begin; index < b.index_end; index++) {
			const auto& p = (*pp)[(*index_pair_internal)[index].index_of_point];
			mean += Eigen::Vector3d(p.x, p.y, p.z);
		}
		mean /= b.number_of_points;

		for (int index = b.index_begin; index < b.index_end; index++) {
			const auto& p = (*pp)[(*index_pair_internal)[index].index_of_point];
			cov(0, 0) += (mean.x() - p.x) * (mean.x() - p.x);
			cov(0, 1) += (mean.x() - p.x) * (mean.y() - p.y);
			cov(0, 2) += (mean.x() - p.x) * (mean.z() - p.z);
			cov(1, 0) += (mean.y() - p.y) * (mean.x() - p.x);
			cov(1, 1) += (mean.y() - p.y) * (mean.y() - p.y);
			cov(1, 2) += (mean.y() - p.y) * (mean.z() - p.z);
			cov(2, 0) += (mean.z() - p.z) * (mean.x() - p.x);
			cov(2, 1) += (mean.z() - p.z) * (mean.y() - p.y);
			cov(2, 2) += (mean.z() - p.z) * (mean.z() - p.z);
		}
		cov /= b.number_of_points;

		(*buckets)[ii].mean = mean;
		(*buckets)[ii].cov = cov;

		Eigen::Matrix3d infm = cov.inverse();

		if (!(infm(0, 0) == infm(0, 0)))continue;
		if (!(infm(0, 1) == infm(0, 1)))continue;
		if (!(infm(0, 2) == infm(0, 2)))continue;

		if (!(infm(1, 0) == infm(1, 0)))continue;
		if (!(infm(1, 1) == infm(1, 1)))continue;
		if (!(infm(1, 2) == infm(1, 2)))continue;

		if (!(infm(2, 0) == infm(2, 0)))continue;
		if (!(infm(2, 1) == infm(2, 1)))continue;
		if (!(infm(2, 2) == infm(2, 2)))continue;

		double threshold = 10000.0;

		if (infm(0, 0) > threshold)infm(0, 0) = threshold;
		if (infm(0, 1) > threshold)infm(0, 1) = threshold;
		if (infm(0, 2) > threshold)infm(0, 2) = threshold;
		if (infm(1, 0) > threshold)infm(1, 0) = threshold;
		if (infm(1, 1) > threshold)infm(1, 1) = threshold;
		if (infm(1, 2) > threshold)infm(1, 2) = threshold;
		if (infm(2, 0) > threshold)infm(2, 0) = threshold;
		if (infm(2, 1) > threshold)infm(2, 1) = threshold;
		if (infm(2, 2) > threshold)infm(2, 2) = threshold;

		if (infm(0, 0) < -threshold)infm(0, 0) = -threshold;
		if (infm(0, 1) < -threshold)infm(0, 1) = -threshold;
		if (infm(0, 2) < -threshold)infm(0, 2) = -threshold;
		if (infm(1, 0) < -threshold)infm(1, 0) = -threshold;
		if (infm(1, 1) < -threshold)infm(1, 1) = -threshold;
		if (infm(1, 2) < -threshold)infm(1, 2) = -threshold;
		if (infm(2, 0) < -threshold)infm(2, 0) = -threshold;
		if (infm(2, 1) < -threshold)infm(2, 1) = -threshold;
		if (infm(2, 2) < -threshold)infm(2, 2) = -threshold;


		for (int index = b.index_begin; index < b.index_end; index++) {
					
			const auto& p = (*pp)[(*index_pair_internal)[index].index_of_point];
			Eigen::Vector3d point_local(p.x, p.y, p.z);
			point_local = (*mposes_inv)[p.index_pose] * point_local;
			int ir = tripletListB.size();
			double delta_x;
			double delta_y;
			double delta_z;

			if (rotation_matrix_parametrization == NDT::RotationMatrixParametrization::tait_bryan_xyz) {
				Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
				//-----------------------
				if(pose_convention == NDT::PoseConvention::wc) {
					TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix((*mposes)[p.index_pose]);
					
					point_to_point_source_to_target_tait_bryan_wc(delta_x, delta_y, delta_z,
						pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
						point_local.x(), point_local.y(), point_local.z(), mean.x(), mean.y(), mean.z());
					
					point_to_point_source_to_target_tait_bryan_wc_jacobian(jacobian,
						pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
						point_local.x(), point_local.y(), point_local.z());
				}
				else {
					TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix((*mposes)[p.index_pose].inverse());

					point_to_point_source_to_target_tait_bryan_cw(delta_x, delta_y, delta_z,
						pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
						point_local.x(), point_local.y(), point_local.z(), mean.x(), mean.y(), mean.z());

					point_to_point_source_to_target_tait_bryan_cw_jacobian(jacobian,
						pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
						point_local.x(), point_local.y(), point_local.z());
				}
				//-----------------------
				int c = p.index_pose * 6;
				for (int row = 0; row < 3; row++) {
					for (int col = 0; col < 6; col++) {
						if (jacobian(row, col) != 0.0) {
							tripletListA.emplace_back(ir + row, c + col, -jacobian(row, col));
						}
					}
				}
			}
			else if (rotation_matrix_parametrization == NDT::RotationMatrixParametrization::rodrigues) {
				Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
				//-----------------------
				if (pose_convention == NDT::PoseConvention::wc) {
					RodriguesPose pose_s = pose_rodrigues_from_affine_matrix((*mposes)[p.index_pose]);

					point_to_point_source_to_target_rodrigues_wc(delta_x, delta_y, delta_z,
						pose_s.px, pose_s.py, pose_s.pz, pose_s.sx, pose_s.sy, pose_s.sz,
						point_local.x(), point_local.y(), point_local.z(), mean.x(), mean.y(), mean.z());

					point_to_point_source_to_target_rodrigues_wc_jacobian(jacobian,
						pose_s.px, pose_s.py, pose_s.pz, pose_s.sx, pose_s.sy, pose_s.sz,
						point_local.x(), point_local.y(), point_local.z());
				}
				else {
					RodriguesPose pose_s = pose_rodrigues_from_affine_matrix((*mposes)[p.index_pose].inverse());

					point_to_point_source_to_target_rodrigues_cw(delta_x, delta_y, delta_z,
						pose_s.px, pose_s.py, pose_s.pz, pose_s.sx, pose_s.sy, pose_s.sz,
						point_local.x(), point_local.y(), point_local.z(), mean.x(), mean.y(), mean.z());

					point_to_point_source_to_target_rodrigues_cw_jacobian(jacobian,
						pose_s.px, pose_s.py, pose_s.pz, pose_s.sx, pose_s.sy, pose_s.sz,
						point_local.x(), point_local.y(), point_local.z());
				}
				//-----------------------
				int c = p.index_pose * 6;
				for (int row = 0; row < 3; row++) {
					for (int col = 0; col < 6; col++) {
						if (jacobian(row, col) != 0.0) {
							tripletListA.emplace_back(ir + row, c + col, -jacobian(row, col));
						}
					}
				}
			}
			else if (rotation_matrix_parametrization == NDT::RotationMatrixParametrization::quaternion) {
				Eigen::Matrix<double, 3, 7, Eigen::RowMajor> jacobian;
				//-----------------------
				if (pose_convention == NDT::PoseConvention::wc) {
					QuaternionPose pose_s = pose_quaternion_from_affine_matrix((*mposes)[p.index_pose]);

					point_to_point_source_to_target_quaternion_wc(delta_x, delta_y, delta_z,
						pose_s.px, pose_s.py, pose_s.pz, pose_s.q0, pose_s.q1, pose_s.q2, pose_s.q3,
						point_local.x(), point_local.y(), point_local.z(), mean.x(), mean.y(), mean.z());

					point_to_point_source_to_target_quaternion_wc_jacobian(jacobian,
						pose_s.px, pose_s.py, pose_s.pz, pose_s.q0, pose_s.q1, pose_s.q2, pose_s.q3,
						point_local.x(), point_local.y(), point_local.z());
				}
				else {
					QuaternionPose pose_s = pose_quaternion_from_affine_matrix((*mposes)[p.index_pose].inverse());

					point_to_point_source_to_target_quaternion_cw(delta_x, delta_y, delta_z,
						pose_s.px, pose_s.py, pose_s.pz, pose_s.q0, pose_s.q1, pose_s.q2, pose_s.q3,
						point_local.x(), point_local.y(), point_local.z(), mean.x(), mean.y(), mean.z());

					point_to_point_source_to_target_quaternion_cw_jacobian(jacobian,
						pose_s.px, pose_s.py, pose_s.pz, pose_s.q0, pose_s.q1, pose_s.q2, pose_s.q3,
						point_local.x(), point_local.y(), point_local.z());
				}
				//-----------------------
				int c = p.index_pose * 7;
				for (int row = 0; row < 3; row++) {
					for (int col = 0; col < 7; col++) {
						if (jacobian(row, col) != 0.0) {
							tripletListA.emplace_back(ir + row, c + col, -jacobian(row, col));
						}
					}
				}
			}
			else if (rotation_matrix_parametrization == NDT::RotationMatrixParametrization::lie_algebra_left_jacobian) {
				Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
				Eigen::Affine3d m_pose = (*mposes)[p.index_pose];
								
				Eigen::Vector3d p_s(point_local.x(), point_local.y(), point_local.z());
				Eigen::Vector3d p_t(mean.x(), mean.y(), mean.z());

				Eigen::Matrix3d R = m_pose.rotation();
				Eigen::Vector3d Rp = R * p_s;
				Eigen::Matrix3d Rpx;
				Rpx(0, 0) = 0;
				Rpx(0, 1) = -Rp.z();
				Rpx(0, 2) = Rp.y();
				Rpx(1, 0) = Rp.z();
				Rpx(1, 1) = 0;
				Rpx(1, 2) = -Rp.x();
				Rpx(2, 0) = -Rp.y();
				Rpx(2, 1) = Rp.x();
				Rpx(2, 2) = 0;

				int ic = p.index_pose * 6;

				tripletListA.emplace_back(ir, ic + 0, 1);
				//tripletListA.emplace_back(ir, ic + 1, 0);
				//tripletListA.emplace_back(ir, ic + 2, 0);
				tripletListA.emplace_back(ir, ic + 3, -Rpx(0, 0));
				tripletListA.emplace_back(ir, ic + 4, -Rpx(0, 1));
				tripletListA.emplace_back(ir, ic + 5, -Rpx(0, 2));

				//tripletListA.emplace_back(ir + 1, ic + 0, 0);
				tripletListA.emplace_back(ir + 1, ic + 1, 1);
				//tripletListA.emplace_back(ir + 1, ic + 2, 0);
				tripletListA.emplace_back(ir + 1, ic + 3, -Rpx(1, 0));
				tripletListA.emplace_back(ir + 1, ic + 4, -Rpx(1, 1));
				tripletListA.emplace_back(ir + 1, ic + 5, -Rpx(1, 2));

				//tripletListA.emplace_back(ir + 2, ic + 0, 0);
				//tripletListA.emplace_back(ir + 2, ic + 1, 0);
				tripletListA.emplace_back(ir + 2, ic + 2, 1);
				tripletListA.emplace_back(ir + 2, ic + 3, -Rpx(2, 0));
				tripletListA.emplace_back(ir + 2, ic + 4, -Rpx(2, 1));
				tripletListA.emplace_back(ir + 2, ic + 5, -Rpx(2, 2));

				Eigen::Vector3d target = p_t;
				Eigen::Vector3d source = m_pose * p_s;
				
				delta_x = target.x() - source.x();
				delta_y = target.y() - source.y();
				delta_z = target.z() - source.z();
			}
			else if (rotation_matrix_parametrization == NDT::RotationMatrixParametrization::lie_algebra_right_jacobian) {
				Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
				Eigen::Affine3d m_pose = (*mposes)[p.index_pose];

				Eigen::Vector3d p_s(point_local.x(), point_local.y(), point_local.z());
				Eigen::Vector3d p_t(mean.x(), mean.y(), mean.z());

				Eigen::Matrix3d px;
				px(0, 0) = 0;
				px(0, 1) = -p_s.z();
				px(0, 2) = p_s.y();
				px(1, 0) = p_s.z();
				px(1, 1) = 0;
				px(1, 2) = -p_s.x();
				px(2, 0) = -p_s.y();
				px(2, 1) = p_s.x();
				px(2, 2) = 0;
				Eigen::Matrix3d R = m_pose.rotation();
				Eigen::Matrix3d Rpx = R * px;

				int ic = p.index_pose * 6;

				tripletListA.emplace_back(ir, ic + 0, R(0, 0));
				tripletListA.emplace_back(ir, ic + 1, R(0, 1));
				tripletListA.emplace_back(ir, ic + 2, R(0, 2));
				tripletListA.emplace_back(ir, ic + 3, -Rpx(0, 0));
				tripletListA.emplace_back(ir, ic + 4, -Rpx(0, 1));
				tripletListA.emplace_back(ir, ic + 5, -Rpx(0, 2));

				tripletListA.emplace_back(ir + 1, ic + 0, R(1, 0));
				tripletListA.emplace_back(ir + 1, ic + 1, R(1, 1));
				tripletListA.emplace_back(ir + 1, ic + 2, R(1, 2));
				tripletListA.emplace_back(ir + 1, ic + 3, -Rpx(1, 0));
				tripletListA.emplace_back(ir + 1, ic + 4, -Rpx(1, 1));
				tripletListA.emplace_back(ir + 1, ic + 5, -Rpx(1, 2));

				tripletListA.emplace_back(ir + 2, ic + 0, R(2, 0));
				tripletListA.emplace_back(ir + 2, ic + 1, R(2, 1));
				tripletListA.emplace_back(ir + 2, ic + 2, R(2, 2));
				tripletListA.emplace_back(ir + 2, ic + 3, -Rpx(2, 0));
				tripletListA.emplace_back(ir + 2, ic + 4, -Rpx(2, 1));
				tripletListA.emplace_back(ir + 2, ic + 5, -Rpx(2, 2));

				Eigen::Vector3d target = p_t;
				Eigen::Vector3d source = m_pose * p_s;
				
				delta_x = target.x() - source.x();
				delta_y = target.y() - source.y();
				delta_z = target.z() - source.z();
			}

			tripletListB.emplace_back(ir, 0, delta_x);
			tripletListB.emplace_back(ir + 1, 0, delta_y);
			tripletListB.emplace_back(ir + 2, 0, delta_z);

			tripletListP.emplace_back(ir, ir, infm(0, 0));
			tripletListP.emplace_back(ir, ir + 1, infm(0, 1));
			tripletListP.emplace_back(ir, ir + 2, infm(0, 2));
			tripletListP.emplace_back(ir + 1, ir, infm(1, 0));
			tripletListP.emplace_back(ir + 1, ir + 1, infm(1, 1));
			tripletListP.emplace_back(ir + 1, ir + 2, infm(1, 2));
			tripletListP.emplace_back(ir + 2, ir, infm(2, 0));
			tripletListP.emplace_back(ir + 2, ir + 1, infm(2, 1));
			tripletListP.emplace_back(ir + 2, ir + 2, infm(2, 2));

			ssr += delta_x * delta_x;
			ssr += delta_y * delta_y;
			ssr += delta_z * delta_z;
			sum_obs += 3;
		}
	}
	(*sumssr) = ssr;
	(*sums_obs) = sum_obs;
	
	Eigen::SparseMatrix<double> matA(tripletListB.size(), trajectory_size * number_of_unknowns);
	Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
	Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

	//for (size_t i = 0; i < tripletListA.size(); i++) {
	//	std::cout << tripletListA[i].value() << " ";
	//}

	matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
	matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
	matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

	Eigen::SparseMatrix<double> AtPAt(trajectory_size * number_of_unknowns, trajectory_size * number_of_unknowns);
	Eigen::SparseMatrix<double> AtPBt(trajectory_size * number_of_unknowns, 1);

	{
		Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
		AtPAt = AtP * matA;
		AtPBt = AtP * matB;

		//std::cout << AtPAt << std::endl;

		(*AtPA) = AtPAt;
		(*AtPB) = AtPBt;
	}
}

bool NDT::optimize(std::vector<PointCloud>& point_clouds)
{
	OptimizationAlgorithm optimization_algorithm;
	if (is_gauss_newton) {
		optimization_algorithm = OptimizationAlgorithm::gauss_newton;
	}
	if (is_levenberg_marguardt) {
		optimization_algorithm = OptimizationAlgorithm::levenberg_marguardt;
	}

	PoseConvention pose_convention;
	if (is_wc) {
		pose_convention = PoseConvention::wc;
	}
	if (is_cw) {
		pose_convention = PoseConvention::cw;
	}

	RotationMatrixParametrization rotation_matrix_parametrization;
	if (is_tait_bryan_angles) {
		rotation_matrix_parametrization = RotationMatrixParametrization::tait_bryan_xyz;
	}
	else if (is_rodrigues) {
		rotation_matrix_parametrization = RotationMatrixParametrization::rodrigues;
	}
	else if (is_quaternion) {
		rotation_matrix_parametrization = RotationMatrixParametrization::quaternion;
	}
	else if (is_lie_algebra_left_jacobian) {
		rotation_matrix_parametrization = RotationMatrixParametrization::lie_algebra_left_jacobian;
	}
	else if (is_lie_algebra_right_jacobian) {
		rotation_matrix_parametrization = RotationMatrixParametrization::lie_algebra_right_jacobian;
	}

	if (is_rodrigues || is_quaternion || is_lie_algebra_left_jacobian || is_lie_algebra_right_jacobian) {
		for (size_t i = 0; i < point_clouds.size(); i++) {
			TaitBryanPose pose;
			pose.px = (((rand() % 1000000000) / 1000000000.0) - 0.5) * 2.0 * 0.000001;
			pose.py = (((rand() % 1000000000) / 1000000000.0) - 0.5) * 2.0 * 0.000001;
			pose.pz = (((rand() % 1000000000) / 1000000000.0) - 0.5) * 2.0 * 0.000001;
			pose.om = (((rand() % 1000000000) / 1000000000.0) - 0.5) * 2.0 * 0.000001;
			pose.fi = (((rand() % 1000000000) / 1000000000.0) - 0.5) * 2.0 * 0.000001;
			pose.ka = (((rand() % 1000000000) / 1000000000.0) - 0.5) * 2.0 * 0.000001;
			Eigen::Affine3d m = affine_matrix_from_pose_tait_bryan(pose);
			point_clouds[i].m_pose = point_clouds[i].m_pose * m;
		}
	}

	int number_of_unknowns;
	if (is_tait_bryan_angles || is_rodrigues || is_lie_algebra_left_jacobian || is_lie_algebra_right_jacobian) {
		number_of_unknowns = 6;
	}
	if (is_quaternion) {
		number_of_unknowns = 7;
	}

	double lm_lambda = 0.0001;
	double previous_rms = std::numeric_limits<double>::max();
	int number_of_lm_iterations = 0;

	std::vector<Eigen::Affine3d> m_poses_tmp;
	if (is_levenberg_marguardt) {
		m_poses_tmp.clear();
		for (size_t i = 0; i < point_clouds.size(); i++) {
			m_poses_tmp.push_back(point_clouds[i].m_pose);
		}
	}

	for (int iter = 0; iter < number_of_iterations; iter++) {
		GridParameters rgd_params;
		rgd_params.resolution_X = this->bucket_size[0];
		rgd_params.resolution_Y = this->bucket_size[1];
		rgd_params.resolution_Z = this->bucket_size[2];
		rgd_params.bounding_box_extension = 1.0;

		std::vector<Point3D> points_global;
		for (size_t i = 0; i < point_clouds.size(); i++) {
			for (size_t j = 0; j < point_clouds[i].points_local.size(); j++) {
				Eigen::Vector3d vt = point_clouds[i].m_pose * point_clouds[i].points_local[j];
				Point3D p;
				p.x = vt.x();
				p.y = vt.y();
				p.z = vt.z();
				p.index_pose = i;
				if(p.x > -250 && p.x < 250 && p.y > -250 && p.y < 250 && p.z > -50 && p.z < 50)
				points_global.push_back(p);
			}
		}

		std::vector<PointBucketIndexPair> index_pair;
		std::vector<Bucket> buckets;

		grid_calculate_params(points_global, rgd_params);
		build_rgd(points_global, index_pair, buckets, rgd_params, this->number_of_threads);

		std::vector<Job> jobs = get_jobs(buckets.size(), this->number_of_threads);

		std::vector<std::thread> threads;

		std::vector<Eigen::SparseMatrix<double>> AtPAtmp(jobs.size());
		std::vector<Eigen::SparseMatrix<double>> AtPBtmp(jobs.size());
		std::vector<double> sumrmss(jobs.size());
		std::vector<int> sums(jobs.size());


		for (size_t i = 0; i < jobs.size(); i++) {
			AtPAtmp[i] = Eigen::SparseMatrix<double>(point_clouds.size() * number_of_unknowns, point_clouds.size() * number_of_unknowns);
			AtPBtmp[i] = Eigen::SparseMatrix<double>(point_clouds.size() * number_of_unknowns, 1);
			sumrmss[i] = 0;
			sums[i] = 0;
		}

		std::vector<Eigen::Affine3d> mposes;
		std::vector<Eigen::Affine3d> mposes_inv;
		for (size_t i = 0; i < point_clouds.size(); i++) {
			mposes.push_back(point_clouds[i].m_pose);
			mposes_inv.push_back(point_clouds[i].m_pose.inverse());
		}

		for (size_t k = 0; k < jobs.size(); k++) {
			threads.push_back(std::thread(ndt_job, k, &jobs[k], &buckets, &(AtPAtmp[k]), &(AtPBtmp[k]),
				&index_pair, &points_global, &mposes, &mposes_inv, point_clouds.size(), pose_convention, rotation_matrix_parametrization, number_of_unknowns, &(sumrmss[k]), &(sums[k])));
		}
		
		for (size_t j = 0; j < threads.size(); j++) {
			threads[j].join();
		}
		
		double rms = 0.0;
		int sum = 0;
		for (size_t k = 0; k < jobs.size(); k++) {
			rms += sumrmss[k];
			sum += sums[k];
		}
		rms /= sum;
		std::cout << "rms " << rms << std::endl;

		bool init = false;
		Eigen::SparseMatrix<double> AtPA_ndt(point_clouds.size() * number_of_unknowns, point_clouds.size() * number_of_unknowns);
		Eigen::SparseMatrix<double> AtPB_ndt(point_clouds.size() * number_of_unknowns, 1);

		for (size_t k = 0; k < jobs.size(); k++) {
			if (!init) {
				if (AtPBtmp[k].size() > 0) {
					AtPA_ndt = AtPAtmp[k];
					AtPB_ndt = AtPBtmp[k];
					init = true;
				}
			}
			else {
				if (AtPBtmp[k].size() > 0) {

					AtPA_ndt += AtPAtmp[k];
					AtPB_ndt += AtPBtmp[k];
				}
			}
		}

		if (is_fix_first_node) {
			Eigen::SparseMatrix<double> I(point_clouds.size() * number_of_unknowns, point_clouds.size() * number_of_unknowns);
			for (int ii = 0; ii < number_of_unknowns; ii++) {
				I.coeffRef(ii, ii) = 1000000;
			}
			AtPA_ndt += I;
		}

		std::cout << "previous_rms: " << previous_rms << " rms: " << rms << std::endl;
		if (is_levenberg_marguardt) {

			if (rms < previous_rms) {
				if (lm_lambda < 1000000) {
					lm_lambda *= 10.0;
				}
				previous_rms = rms;
				std::cout << " lm_lambda: " << lm_lambda << std::endl;
			}
			else {
				lm_lambda /= 10.0;
				number_of_lm_iterations++;
				iter--;
				std::cout << " lm_lambda: " << lm_lambda << std::endl;
				for (size_t i = 0; i < point_clouds.size(); i++) {
					point_clouds[i].m_pose = m_poses_tmp[i];
				}
				previous_rms = std::numeric_limits<double>::max();
				continue;
			}
		}
		else {
			previous_rms = rms;
		}

		if (is_quaternion) {
			std::vector<Eigen::Triplet<double>> tripletListA;
			std::vector<Eigen::Triplet<double>> tripletListP;
			std::vector<Eigen::Triplet<double>> tripletListB;
			for (size_t i = 0; i < point_clouds.size(); i++) {
				int ic = i * 7;
				int ir = 0;
				QuaternionPose pose;
				if (is_wc) {
					pose = pose_quaternion_from_affine_matrix(point_clouds[i].m_pose);
				}
				else {
					pose = pose_quaternion_from_affine_matrix(point_clouds[i].m_pose.inverse());
				}
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
			Eigen::SparseMatrix<double> matA(tripletListB.size(), point_clouds.size()* 7);
			Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
			Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

			matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
			matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
			matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

			Eigen::SparseMatrix<double> AtPA(point_clouds.size() * 7, point_clouds.size()* 7);
			Eigen::SparseMatrix<double> AtPB(point_clouds.size() * 7, 1);

			Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
			AtPA = AtP * matA;
			AtPB = AtP * matB;

			AtPA_ndt += AtPA;
			AtPB_ndt += AtPB;
		}

		if (is_levenberg_marguardt) {
			Eigen::SparseMatrix<double> LM(point_clouds.size() * number_of_unknowns, point_clouds.size() * number_of_unknowns);
			LM.setIdentity();
			LM *= lm_lambda;
			AtPA_ndt += LM;
		}

		std::cout << "start solving AtPA=AtPB" << std::endl;
		Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA_ndt);

		std::cout << "x = solver.solve(AtPB)" << std::endl;
		Eigen::SparseMatrix<double> x = solver.solve(AtPB_ndt);

		std::vector<double> h_x;
		std::cout << "redult: row,col,value" << std::endl;
		for (int k = 0; k < x.outerSize(); ++k) {
			for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it) {
				if (it.value() == it.value()) {
					h_x.push_back(it.value());
					std::cout << it.row() << "," << it.col() << "," << it.value() << std::endl;
				}
			}
		}

		if (h_x.size() == point_clouds.size() * number_of_unknowns) {
			std::cout << "AtPA=AtPB SOLVED" << std::endl;
			int counter = 0;
			for (size_t i = 0; i < point_clouds.size(); i++) {
				Eigen::Affine3d m_pose;

				if (is_wc) {
					m_pose = point_clouds[i].m_pose;
				}
				else {
					m_pose = point_clouds[i].m_pose.inverse();
				}

				if (is_tait_bryan_angles) {
					TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(m_pose);
					pose.px += h_x[counter++];
					pose.py += h_x[counter++];
					pose.pz += h_x[counter++];
					pose.om += h_x[counter++];
					pose.fi += h_x[counter++];
					pose.ka += h_x[counter++];
					m_pose = affine_matrix_from_pose_tait_bryan(pose);
				}
				else if (is_rodrigues) {
					RodriguesPose pose = pose_rodrigues_from_affine_matrix(m_pose);
					pose.px += h_x[counter++];
					pose.py += h_x[counter++];
					pose.pz += h_x[counter++];
					pose.sx += h_x[counter++];
					pose.sy += h_x[counter++];
					pose.sz += h_x[counter++];
					m_pose = affine_matrix_from_pose_rodrigues(pose);
				}
				else if (is_quaternion) {
					QuaternionPose pose = pose_quaternion_from_affine_matrix(m_pose);
					
					QuaternionPose poseq;
					poseq.px = h_x[counter++];
					poseq.py = h_x[counter++];
					poseq.pz = h_x[counter++];
					poseq.q0 = h_x[counter++];
					poseq.q1 = h_x[counter++];
					poseq.q2 = h_x[counter++];
					poseq.q3 = h_x[counter++];
					
					if (fabs(poseq.px) < this->bucket_size[0] && fabs(poseq.py) < this->bucket_size[0] && fabs(poseq.pz) < this->bucket_size[0] &&
						fabs(poseq.q0) < 10 && fabs(poseq.q1) < 10 && fabs(poseq.q2) < 10 && fabs(poseq.q3) < 10) {
						pose.px += poseq.px;
						pose.py += poseq.py;
						pose.pz += poseq.pz;
						pose.q0 += poseq.q0;
						pose.q1 += poseq.q1;
						pose.q2 += poseq.q2;
						pose.q3 += poseq.q3;
						m_pose = affine_matrix_from_pose_quaternion(pose);
					}
				}
				else if (is_lie_algebra_left_jacobian) {
					RodriguesPose pose_update;
					pose_update.px = h_x[counter++];
					pose_update.py = h_x[counter++];
					pose_update.pz = h_x[counter++];
					pose_update.sx = h_x[counter++];
					pose_update.sy = h_x[counter++];
					pose_update.sz = h_x[counter++];
					m_pose = affine_matrix_from_pose_rodrigues(pose_update) * m_pose;
				}
				else if (is_lie_algebra_right_jacobian) {
					RodriguesPose pose_update;
					pose_update.px = h_x[counter++];
					pose_update.py = h_x[counter++];
					pose_update.pz = h_x[counter++];
					pose_update.sx = h_x[counter++];
					pose_update.sy = h_x[counter++];
					pose_update.sz = h_x[counter++];
					m_pose = m_pose * affine_matrix_from_pose_rodrigues(pose_update);
				}

				if (is_wc) {
					point_clouds[i].m_pose = m_pose;
				}
				else {
					point_clouds[i].m_pose = m_pose.inverse();
				}
				if (!point_clouds[i].fixed) {
					point_clouds[i].pose = pose_tait_bryan_from_affine_matrix(point_clouds[i].m_pose);
					point_clouds[i].gui_translation[0] = point_clouds[i].pose.px;
					point_clouds[i].gui_translation[1] = point_clouds[i].pose.py;
					point_clouds[i].gui_translation[2] = point_clouds[i].pose.pz;
					point_clouds[i].gui_rotation[0] = rad2deg(point_clouds[i].pose.om);
					point_clouds[i].gui_rotation[1] = rad2deg(point_clouds[i].pose.fi);
					point_clouds[i].gui_rotation[2] = rad2deg(point_clouds[i].pose.ka);
				}
			}
			if (is_levenberg_marguardt) {
				m_poses_tmp.clear();
				for (size_t i = 0; i < point_clouds.size(); i++) {
					m_poses_tmp.push_back(point_clouds[i].m_pose);
				}
			}

			std::cout << "iteration: " << iter + 1 << " of " << number_of_iterations << std::endl;
		}
		else {
			std::cout << "AtPA=AtPB FAILED" << std::endl;
			break;
		}
	}
	return true;
}

std::vector<Eigen::SparseMatrix<double>> NDT::compute_covariance_matrices_and_rms(std::vector<PointCloud>& point_clouds, double &rms)
{
	OptimizationAlgorithm optimization_algorithm;
	if (is_gauss_newton) {
		optimization_algorithm = OptimizationAlgorithm::gauss_newton;
	}
	if (is_levenberg_marguardt) {
		optimization_algorithm = OptimizationAlgorithm::levenberg_marguardt;
	}

	PoseConvention pose_convention;
	if (is_wc) {
		pose_convention = PoseConvention::wc;
	}
	if (is_cw) {
		pose_convention = PoseConvention::cw;
	}

	RotationMatrixParametrization rotation_matrix_parametrization;
	if (is_tait_bryan_angles) {
		rotation_matrix_parametrization = RotationMatrixParametrization::tait_bryan_xyz;
	}
	if (is_rodrigues) {
		rotation_matrix_parametrization = RotationMatrixParametrization::rodrigues;
	}
	if (is_quaternion) {
		rotation_matrix_parametrization = RotationMatrixParametrization::quaternion;
	}

	if (is_rodrigues || is_quaternion) {
		for (size_t i = 0; i < point_clouds.size(); i++) {
			TaitBryanPose pose;
			pose.px = (((rand() % 1000000000) / 1000000000.0) - 0.5) * 2.0 * 0.000001;
			pose.py = (((rand() % 1000000000) / 1000000000.0) - 0.5) * 2.0 * 0.000001;
			pose.pz = (((rand() % 1000000000) / 1000000000.0) - 0.5) * 2.0 * 0.000001;
			pose.om = (((rand() % 1000000000) / 1000000000.0) - 0.5) * 2.0 * 0.000001;
			pose.fi = (((rand() % 1000000000) / 1000000000.0) - 0.5) * 2.0 * 0.000001;
			pose.ka = (((rand() % 1000000000) / 1000000000.0) - 0.5) * 2.0 * 0.000001;
			Eigen::Affine3d m = affine_matrix_from_pose_tait_bryan(pose);
			point_clouds[i].m_pose = point_clouds[i].m_pose * m;
		}
	}

	int number_of_unknowns;
	if (is_tait_bryan_angles || is_rodrigues) {
		number_of_unknowns = 6;
	}
	if (is_quaternion) {
		number_of_unknowns = 7;
	}

	std::vector<Eigen::SparseMatrix<double>> covariance_matrices;

	for (size_t i = 0; i < point_clouds.size(); i++) {
		Eigen::SparseMatrix<double> covariance_matrix(number_of_unknowns, number_of_unknowns);
		covariance_matrices.push_back(covariance_matrix);
	}
		
	GridParameters rgd_params;
	rgd_params.resolution_X = this->bucket_size[0];
	rgd_params.resolution_Y = this->bucket_size[1];
	rgd_params.resolution_Z = this->bucket_size[2];
	rgd_params.bounding_box_extension = 1.0;

	std::vector<Point3D> points_global;
	for (size_t i = 0; i < point_clouds.size(); i++) {
		for (size_t j = 0; j < point_clouds[i].points_local.size(); j++) {
			Eigen::Vector3d vt = point_clouds[i].m_pose * point_clouds[i].points_local[j];
			Point3D p;
			p.x = vt.x();
			p.y = vt.y();
			p.z = vt.z();
			p.index_pose = i;
			points_global.push_back(p);
		}
	}

	std::vector<PointBucketIndexPair> index_pair;
	std::vector<Bucket> buckets;

	grid_calculate_params(points_global, rgd_params);
	build_rgd(points_global, index_pair, buckets, rgd_params, this->number_of_threads);

	std::vector<Job> jobs = get_jobs(buckets.size(), this->number_of_threads);

	std::vector<std::thread> threads;

	std::vector<Eigen::SparseMatrix<double>> AtPAtmp(jobs.size());
	std::vector<Eigen::SparseMatrix<double>> AtPBtmp(jobs.size());
	std::vector<double> sumrmss(jobs.size());
	std::vector<int> sums(jobs.size());


	for (size_t i = 0; i < jobs.size(); i++) {
		AtPAtmp[i] = Eigen::SparseMatrix<double>(point_clouds.size() * number_of_unknowns, point_clouds.size() * number_of_unknowns);
		AtPBtmp[i] = Eigen::SparseMatrix<double>(point_clouds.size() * number_of_unknowns, 1);
		sumrmss[i] = 0;
		sums[i] = 0;
	}

	std::vector<Eigen::Affine3d> mposes;
	std::vector<Eigen::Affine3d> mposes_inv;
	for (size_t i = 0; i < point_clouds.size(); i++) {
		//poses.push_back(pose_tait_bryan_from_affine_matrix(point_clouds[i].m_pose));
		mposes.push_back(point_clouds[i].m_pose);
		mposes_inv.push_back(point_clouds[i].m_pose.inverse());
	}
	//std::cout << "XXX" << std::endl;
	//std::cout << (int)pose_convention << " " << (int)rotation_matrix_parametrization << " " << (int)number_of_unknowns << std::endl;

	for (size_t k = 0; k < jobs.size(); k++) {
		threads.push_back(std::thread(ndt_job, k, &jobs[k], &buckets, &(AtPAtmp[k]), &(AtPBtmp[k]),
			&index_pair, &points_global, &mposes, &mposes_inv, point_clouds.size(), pose_convention, rotation_matrix_parametrization, number_of_unknowns, &(sumrmss[k]), &(sums[k])));
	}

	for (size_t j = 0; j < threads.size(); j++) {
		threads[j].join();
	}
	double ssr = 0.0;
	int num_obs = 0;
	for (size_t k = 0; k < jobs.size(); k++) {
		ssr += sumrmss[k];
		num_obs += sums[k];
	}
	
	double sq = ssr / ((double)num_obs - point_clouds.size() * number_of_unknowns);
	rms = ssr / (double)num_obs;

	//std::cout << "sq " << sq << " num_obs " << num_obs << std::endl;

	bool init = false;
	Eigen::SparseMatrix<double> AtPA_ndt(point_clouds.size() * number_of_unknowns, point_clouds.size() * number_of_unknowns);
	
	for (size_t k = 0; k < jobs.size(); k++) {
		if (!init) {
			if (AtPBtmp[k].size() > 0) {
				//
				AtPA_ndt = AtPAtmp[k];
				init = true;
			}
		}
		else {
			if (AtPBtmp[k].size() > 0) {
				AtPA_ndt += AtPAtmp[k];
			}
		}
	}
	
	AtPA_ndt = 0.5 * AtPA_ndt;

	Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA_ndt);
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
		//std::cout << "cm" << std::endl;
		//std::cout << cm << std::endl;
	}
	return covariance_matrices;
}

bool NDT::optimize(std::vector<PointCloud>& point_clouds, double& rms_initial, double& rms_final, double& mui)
{
	//double rms;
	//std::vector<Eigen::SparseMatrix<double>> cm_before = compute_covariance_matrices_and_rms(point_clouds, rms);
	//rms_initial = rms;
	//--
	bool res = optimize(point_clouds);
	//--
	//std::vector<Eigen::SparseMatrix<double>> cm_after = compute_covariance_matrices_and_rms(point_clouds, rms);
	//rms_final = rms;
	//mui = get_mean_uncertainty_xyz_impact(cm_before, cm_after);
	return res;
}

bool NDT::optimize_lie_algebra_left_jacobian(std::vector<PointCloud>& point_clouds)
{
	is_tait_bryan_angles = false;
	is_quaternion = false;
	is_rodrigues = false;
	is_lie_algebra_left_jacobian = true;
	is_lie_algebra_right_jacobian = false;

	bool res = optimize(point_clouds);

	is_tait_bryan_angles = true;
	is_quaternion = false;
	is_rodrigues = false;
	is_lie_algebra_left_jacobian = false;
	is_lie_algebra_right_jacobian = false;
	return true;
}


bool NDT::optimize_lie_algebra_right_jacobian(std::vector<PointCloud>& point_clouds)
{
	is_tait_bryan_angles = false;
	is_quaternion = false;
	is_rodrigues = false;
	is_lie_algebra_left_jacobian = false;
	is_lie_algebra_right_jacobian = true;

	bool res = optimize(point_clouds);

	is_tait_bryan_angles = true;
	is_quaternion = false;
	is_rodrigues = false;
	is_lie_algebra_left_jacobian = false;
	is_lie_algebra_right_jacobian = false;
	return true;
}
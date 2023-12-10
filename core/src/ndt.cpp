#include <ndt.h>
#include <structures.h>
// #include <Eigen\Eigen>
#include <vector>
#include <thread>
#include <iostream>
#include <random>

#include <point_clouds.h>

#include <transformations.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_cw_jacobian.h>

#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_rodrigues_wc_jacobian.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_rodrigues_cw_jacobian.h>

#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_quaternion_wc_jacobian.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_quaternion_cw_jacobian.h>
#include <python-scripts/constraints/quaternion_constraint_jacobian.h>

#include <python-scripts/elementary_error_theory_for_terrestrial_laser_scanner_jacobian.h>

void NDT::grid_calculate_params(const std::vector<Point3D> &point_cloud_global, GridParameters &in_out_params)
{
	double min_x = std::numeric_limits<double>::max();
	double max_x = std::numeric_limits<double>::lowest();

	double min_y = std::numeric_limits<double>::max();
	double max_y = std::numeric_limits<double>::lowest();

	double min_z = std::numeric_limits<double>::max();
	double max_z = std::numeric_limits<double>::lowest();

	for (size_t i = 0; i < point_cloud_global.size(); i++)
	{
		if (point_cloud_global[i].x < min_x)
			min_x = point_cloud_global[i].x;
		if (point_cloud_global[i].x > max_x)
			max_x = point_cloud_global[i].x;

		if (point_cloud_global[i].y < min_y)
			min_y = point_cloud_global[i].y;
		if (point_cloud_global[i].y > max_y)
			max_y = point_cloud_global[i].y;

		if (point_cloud_global[i].z < min_z)
			min_z = point_cloud_global[i].z;
		if (point_cloud_global[i].z > max_z)
			max_z = point_cloud_global[i].z;
	}

	max_x += in_out_params.bounding_box_extension;
	min_x -= in_out_params.bounding_box_extension;

	max_y += in_out_params.bounding_box_extension;
	min_y -= in_out_params.bounding_box_extension;

	max_z += in_out_params.bounding_box_extension;
	min_z -= in_out_params.bounding_box_extension;

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

void NDT::grid_calculate_params(const std::vector<Point3Di> &point_cloud_global, GridParameters &in_out_params)
{
	double min_x = std::numeric_limits<double>::max();
	double max_x = std::numeric_limits<double>::lowest();

	double min_y = std::numeric_limits<double>::max();
	double max_y = std::numeric_limits<double>::lowest();

	double min_z = std::numeric_limits<double>::max();
	double max_z = std::numeric_limits<double>::lowest();

	for (size_t i = 0; i < point_cloud_global.size(); i++)
	{
		if (point_cloud_global[i].point.x() < min_x)
			min_x = point_cloud_global[i].point.x();
		if (point_cloud_global[i].point.x() > max_x)
			max_x = point_cloud_global[i].point.x();

		if (point_cloud_global[i].point.y() < min_y)
			min_y = point_cloud_global[i].point.y();
		if (point_cloud_global[i].point.y() > max_y)
			max_y = point_cloud_global[i].point.y();

		if (point_cloud_global[i].point.z() < min_z)
			min_z = point_cloud_global[i].point.z();
		if (point_cloud_global[i].point.z() > max_z)
			max_z = point_cloud_global[i].point.z();
	}

	max_x += in_out_params.bounding_box_extension;
	min_x -= in_out_params.bounding_box_extension;

	max_y += in_out_params.bounding_box_extension;
	min_y -= in_out_params.bounding_box_extension;

	max_z += in_out_params.bounding_box_extension;
	min_z -= in_out_params.bounding_box_extension;

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

void NDT::grid_calculate_params(GridParameters &in_out_params, double min_x, double max_x, double min_y, double max_y, double min_z, double max_z)
{
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

std::vector<NDT::Job> NDT::get_jobs(long long unsigned int size, int num_threads)
{

	int hc = size / num_threads;
	if (hc < 1)
		hc = 1;

	std::vector<Job> jobs;
	for (long long unsigned int i = 0; i < size; i += hc)
	{
		long long unsigned int sequence_length = hc;
		if (i + hc >= size)
		{
			sequence_length = size - i;
		}
		if (sequence_length == 0)
			break;

		Job j;
		j.index_begin_inclusive = i;
		j.index_end_exclusive = i + sequence_length;
		jobs.push_back(j);
	}

	return jobs;
}

void reindex_job(int i, NDT::Job *job, std::vector<Point3D> *points, std::vector<NDT::PointBucketIndexPair> *pairs, NDT::GridParameters rgd_params)
{
	for (size_t ii = job->index_begin_inclusive; ii < job->index_end_exclusive; ii++)
	{

		Point3D &p = (*points)[ii];

		(*pairs)[ii].index_of_point = ii;
		(*pairs)[ii].index_of_bucket = 0;
		(*pairs)[ii].index_pose = p.index_pose;

		if (p.x < rgd_params.bounding_box_min_X)
		{
			continue;
		}
		if (p.x > rgd_params.bounding_box_max_X)
		{
			continue;
		}
		if (p.y < rgd_params.bounding_box_min_Y)
		{
			continue;
		}
		if (p.y > rgd_params.bounding_box_max_Y)
		{
			continue;
		}
		if (p.z < rgd_params.bounding_box_min_Z)
		{
			continue;
		}
		if (p.z > rgd_params.bounding_box_max_Z)
		{
			continue;
		}

		long long unsigned int ix = (p.x - rgd_params.bounding_box_min_X) / rgd_params.resolution_X;
		long long unsigned int iy = (p.y - rgd_params.bounding_box_min_Y) / rgd_params.resolution_Y;
		long long unsigned int iz = (p.z - rgd_params.bounding_box_min_Z) / rgd_params.resolution_Z;

		(*pairs)[ii].index_of_bucket = ix * static_cast<long long unsigned int>(rgd_params.number_of_buckets_Y) *
										   static_cast<long long unsigned int>(rgd_params.number_of_buckets_Z) +
									   iy * static_cast<long long unsigned int>(rgd_params.number_of_buckets_Z) + iz;
	}
}

void reindex_jobi(int i, NDT::Job *job, std::vector<Point3Di> *points, std::vector<NDT::PointBucketIndexPair> *pairs, NDT::GridParameters rgd_params)
{
	for (size_t ii = job->index_begin_inclusive; ii < job->index_end_exclusive; ii++)
	{

		Point3Di &p = (*points)[ii];

		(*pairs)[ii].index_of_point = ii;
		(*pairs)[ii].index_of_bucket = 0;
		(*pairs)[ii].index_pose = p.index_pose;

		if (p.point.x() < rgd_params.bounding_box_min_X)
		{
			continue;
		}
		if (p.point.x() > rgd_params.bounding_box_max_X)
		{
			continue;
		}
		if (p.point.y() < rgd_params.bounding_box_min_Y)
		{
			continue;
		}
		if (p.point.y() > rgd_params.bounding_box_max_Y)
		{
			continue;
		}
		if (p.point.z() < rgd_params.bounding_box_min_Z)
		{
			continue;
		}
		if (p.point.z() > rgd_params.bounding_box_max_Z)
		{
			continue;
		}

		long long unsigned int ix = (p.point.x() - rgd_params.bounding_box_min_X) / rgd_params.resolution_X;
		long long unsigned int iy = (p.point.y() - rgd_params.bounding_box_min_Y) / rgd_params.resolution_Y;
		long long unsigned int iz = (p.point.z() - rgd_params.bounding_box_min_Z) / rgd_params.resolution_Z;

		(*pairs)[ii].index_of_bucket = ix * static_cast<long long unsigned int>(rgd_params.number_of_buckets_Y) *
										   static_cast<long long unsigned int>(rgd_params.number_of_buckets_Z) +
									   iy * static_cast<long long unsigned int>(rgd_params.number_of_buckets_Z) + iz;
	}
}

void NDT::reindex(std::vector<Point3D> &points, std::vector<NDT::PointBucketIndexPair> &index_pair, NDT::GridParameters &rgd_params, int num_threads)
{
	index_pair.resize(points.size());

	std::vector<NDT::Job> jobs = get_jobs(index_pair.size(), num_threads);

	std::vector<std::thread> threads;

	for (size_t i = 0; i < jobs.size(); i++)
	{
		threads.push_back(std::thread(reindex_job, i, &jobs[i], &points, &index_pair, rgd_params));
	}

	for (size_t j = 0; j < threads.size(); j++)
	{
		threads[j].join();
	}
	threads.clear();

	std::sort(index_pair.begin(), index_pair.end(), [](const NDT::PointBucketIndexPair &a, const NDT::PointBucketIndexPair &b)
			  { return ((a.index_of_bucket == b.index_of_bucket) ? (a.index_pose < b.index_pose) : (a.index_of_bucket < b.index_of_bucket)); });
}

void NDT::reindex(std::vector<Point3Di> &points, std::vector<NDT::PointBucketIndexPair> &index_pair, NDT::GridParameters &rgd_params, int num_threads)
{
	index_pair.resize(points.size());

	std::vector<NDT::Job> jobs = get_jobs(index_pair.size(), num_threads);

	std::vector<std::thread> threads;

	for (size_t i = 0; i < jobs.size(); i++)
	{
		threads.push_back(std::thread(reindex_jobi, i, &jobs[i], &points, &index_pair, rgd_params));
	}

	for (size_t j = 0; j < threads.size(); j++)
	{
		threads[j].join();
	}
	threads.clear();

	std::sort(index_pair.begin(), index_pair.end(), [](const NDT::PointBucketIndexPair &a, const NDT::PointBucketIndexPair &b)
			  { return ((a.index_of_bucket == b.index_of_bucket) ? (a.index_pose < b.index_pose) : (a.index_of_bucket < b.index_of_bucket)); });
}

void build_rgd_init_job(int i, NDT::Job *job, std::vector<NDT::Bucket> *buckets)
{

	for (size_t ii = job->index_begin_inclusive; ii < job->index_end_exclusive; ii++)
	{
		(*buckets)[ii].index_begin = 0;
		(*buckets)[ii].index_end = 0;
		(*buckets)[ii].number_of_points = 0;
	}
}

void build_rgd_job(int i, NDT::Job *job, std::vector<NDT::PointBucketIndexPair> *index_pair, std::vector<NDT::Bucket> *buckets)
{
	for (size_t ii = job->index_begin_inclusive; ii < job->index_end_exclusive; ii++)
	{
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
		else if (ind == (*index_pair).size() - 1)
		{
			if ((*index_pair)[ind].index_of_bucket < (*buckets).size())
			{
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

void build_rgd_final_job(int i, NDT::Job *job, std::vector<NDT::Bucket> *buckets)
{
	for (size_t ii = job->index_begin_inclusive; ii < job->index_end_exclusive; ii++)
	{
		long long unsigned int index_begin = (*buckets)[ii].index_begin;
		long long unsigned int index_end = (*buckets)[ii].index_end;
		if (index_end > index_begin)
		{
			(*buckets)[ii].number_of_points = index_end - index_begin;
		}
	}
}

void NDT::build_rgd(std::vector<Point3D> &points, std::vector<NDT::PointBucketIndexPair> &index_pair, std::vector<NDT::Bucket> &buckets, NDT::GridParameters &rgd_params, int num_threads)
{
	if (num_threads < 1)
		num_threads = 1;

	std::cout << "points.size(): " << points.size() << std::endl;

	index_pair.resize(points.size());
	reindex(points, index_pair, rgd_params, num_threads);

	buckets.resize(rgd_params.number_of_buckets);

	std::vector<NDT::Job> jobs = get_jobs(buckets.size(), num_threads);
	std::vector<std::thread> threads;

	for (size_t i = 0; i < jobs.size(); i++)
	{
		threads.push_back(std::thread(build_rgd_init_job, i, &jobs[i], &buckets));
	}

	for (size_t j = 0; j < threads.size(); j++)
	{
		threads[j].join();
	}
	threads.clear();

	jobs = get_jobs(points.size(), num_threads);

	for (size_t i = 0; i < jobs.size(); i++)
	{
		threads.push_back(std::thread(build_rgd_job, i, &jobs[i], &index_pair, &buckets));
	}
	for (size_t j = 0; j < threads.size(); j++)
	{
		threads[j].join();
	}
	threads.clear();

	jobs = get_jobs(buckets.size(), num_threads);

	for (size_t i = 0; i < jobs.size(); i++)
	{
		threads.push_back(std::thread(build_rgd_final_job, i, &jobs[i], &buckets));
	}

	for (size_t j = 0; j < threads.size(); j++)
	{
		threads[j].join();
	}
	threads.clear();
}

void NDT::build_rgd(std::vector<Point3Di> &points, std::vector<NDT::PointBucketIndexPair> &index_pair, std::vector<NDT::Bucket> &buckets, NDT::GridParameters &rgd_params, int num_threads)
{
	if (num_threads < 1)
		num_threads = 1;

	index_pair.resize(points.size());
	std::cout << "reindex start" << std::endl;
	reindex(points, index_pair, rgd_params, num_threads);
	std::cout << "reindex finished" << std::endl;

	buckets.resize(rgd_params.number_of_buckets);

	std::vector<NDT::Job> jobs = get_jobs(buckets.size(), num_threads);
	std::vector<std::thread> threads;

	std::cout << "build_rgd_init_jobs start" << std::endl;
	for (size_t i = 0; i < jobs.size(); i++)
	{
		threads.push_back(std::thread(build_rgd_init_job, i, &jobs[i], &buckets));
	}

	for (size_t j = 0; j < threads.size(); j++)
	{
		threads[j].join();
	}
	threads.clear();
	std::cout << "build_rgd_init_jobs finished" << std::endl;

	jobs = get_jobs(points.size(), num_threads);

	std::cout << "build_rgd_jobs start jobs.size():" << jobs.size() << std::endl;
	std::cout << "points.size() " << points.size() << std::endl;
	std::cout << "index_pair.size() " << index_pair.size() << std::endl;
	std::cout << "buckets.size() " << buckets.size() << std::endl;

	for (size_t i = 0; i < jobs.size(); i++)
	{
		threads.push_back(std::thread(build_rgd_job, i, &jobs[i], &index_pair, &buckets));
	}
	for (size_t j = 0; j < threads.size(); j++)
	{
		threads[j].join();
	}
	threads.clear();
	std::cout << "build_rgd_jobs finished" << std::endl;

	jobs = get_jobs(buckets.size(), num_threads);

	std::cout << "build_rgd_final_jobs start" << std::endl;
	for (size_t i = 0; i < jobs.size(); i++)
	{
		threads.push_back(std::thread(build_rgd_final_job, i, &jobs[i], &buckets));
	}

	for (size_t j = 0; j < threads.size(); j++)
	{
		threads[j].join();
	}
	threads.clear();
	std::cout << "build_rgd_final_jobs finished" << std::endl;
}

std::vector<Eigen::Vector3d> get_points_normal_distribution(const Eigen::Matrix3d &covar, const Eigen::Vector3d &mean, const int num_points = 100)
{
	Eigen::LLT<Eigen::Matrix<double, 3, 3>> cholSolver(covar);
	Eigen::Matrix3d transform = cholSolver.matrixL();
	std::random_device rd{};
	std::mt19937 gen{rd()};
	std::vector<Eigen::Vector3d> res;
	for (int i = 0; i < num_points; i++)
	{
		std::normal_distribution<double> x{0.0, 1.0};
		std::normal_distribution<double> y{0.0, 1.0};
		std::normal_distribution<double> z{0.0, 1.0};
		Eigen::Vector3d xyz(x(gen), y(gen), z(gen));
		res.emplace_back(transform * xyz + mean);
	}
	return res;
}

void ndt_job(int i, NDT::Job *job, std::vector<NDT::Bucket> *buckets, Eigen::SparseMatrix<double> *AtPA,
			 Eigen::SparseMatrix<double> *AtPB, std::vector<NDT::PointBucketIndexPair> *index_pair_internal, std::vector<Point3D> *pp,
			 std::vector<Eigen::Affine3d> *mposes, std::vector<Eigen::Affine3d> *mposes_inv, size_t trajectory_size,
			 NDT::PoseConvention pose_convention, NDT::RotationMatrixParametrization rotation_matrix_parametrization, int number_of_unknowns, double *sumssr, int *sums_obs,
			 bool is_generalized, double sigma_r, double sigma_polar_angle, double sigma_azimuthal_angle, int num_extended_points, double *md_out, double *md_count_out,
			 bool compute_only_mean_and_cov)
{
	std::vector<Eigen::Triplet<double>> tripletListA;
	std::vector<Eigen::Triplet<double>> tripletListP;
	std::vector<Eigen::Triplet<double>> tripletListB;

	Eigen::SparseMatrix<double> AtPAt_tmp(trajectory_size * number_of_unknowns, trajectory_size * number_of_unknowns);
	Eigen::SparseMatrix<double> AtPBt_tmp(trajectory_size * number_of_unknowns, 1);

	double ssr = 0.0;
	int sum_obs = 0;
	bool init = true;
	double md = 0.0;
	double md_count = 0.0;

	for (size_t ii = job->index_begin_inclusive; ii < job->index_end_exclusive; ii++)
	{
		NDT::Bucket &b = (*buckets)[ii];
		if (b.number_of_points < 5)
			continue;

		Eigen::Vector3d mean(0, 0, 0);
		Eigen::Matrix3d cov;
		cov.setZero();

		int counter = 0;

		const auto &p_ = (*pp)[(*index_pair_internal)[b.index_begin].index_of_point];
		int index_first_pose = p_.index_pose;

		for (int index = b.index_begin; index < b.index_end; index++)
		{
			/*if (is_generalized)
			{
				const auto &p = (*pp)[(*index_pair_internal)[index].index_of_point];
				int index_pose = p.index_pose;

				if (index_pose == index_first_pose)
				{

					const auto &m = (*mposes)[p.index_pose];
					const auto &minv = (*mposes_inv)[p.index_pose];

					Eigen::Vector3d point_local(p.x, p.y, p.z);
					point_local = minv * point_local;

					double norm = point_local.norm();
					double r = norm;
					double polar_angle = acos(point_local.z() / norm);
					double azimuthal_angle = atan(point_local.y() / point_local.x());

					Eigen::Matrix<double, 3, 3> j;
					elementary_error_theory_for_terrestrial_laser_scanner_jacobian(j, r, polar_angle, azimuthal_angle);

					Eigen::Matrix<double, 3, 3> cov_r_alpha_theta;
					cov_r_alpha_theta(0, 0) = sigma_r * sigma_r;
					cov_r_alpha_theta(0, 1) = 0.0;
					cov_r_alpha_theta(0, 2) = 0.0;

					cov_r_alpha_theta(1, 0) = 0.0;
					cov_r_alpha_theta(1, 1) = sigma_polar_angle * sigma_polar_angle;
					cov_r_alpha_theta(1, 2) = 0.0;

					cov_r_alpha_theta(2, 0) = 0.0;
					cov_r_alpha_theta(2, 1) = 0.0;
					cov_r_alpha_theta(2, 2) = sigma_azimuthal_angle * sigma_azimuthal_angle;

					Eigen::Matrix<double, 3, 3> cov_xyz = j * cov_r_alpha_theta * j.transpose();
					std::vector<Eigen::Vector3d> points = get_points_normal_distribution(cov_xyz, point_local, num_extended_points);

					for (const auto &ppl : points)
					{
						mean += m * ppl;
						counter++;
					}
				}
			}
			else*/
			{
				const auto &p = (*pp)[(*index_pair_internal)[index].index_of_point];
				mean += Eigen::Vector3d(p.x, p.y, p.z);
				counter++;
			}
		}

		mean /= counter;

		counter = 0;
		for (int index = b.index_begin; index < b.index_end; index++)
		{
			const auto &p = (*pp)[(*index_pair_internal)[index].index_of_point];
			//const auto &m = (*mposes)[p.index_pose];
			//const auto &minv = (*mposes_inv)[p.index_pose];

			/*if (is_generalized)
			{
				int index_pose = p.index_pose;

				if (index_pose == index_first_pose)
				{

					Eigen::Vector3d point_local(p.x, p.y, p.z);
					point_local = minv * point_local;

					double norm = point_local.norm();
					double r = norm;
					double polar_angle = acos(point_local.z() / norm);
					double azimuthal_angle = atan(point_local.y() / point_local.x());

					Eigen::Matrix<double, 3, 3> j;
					elementary_error_theory_for_terrestrial_laser_scanner_jacobian(j, r, polar_angle, azimuthal_angle);

					Eigen::Matrix<double, 3, 3> cov_r_alpha_theta;
					cov_r_alpha_theta(0, 0) = sigma_r * sigma_r;
					cov_r_alpha_theta(0, 1) = 0.0;
					cov_r_alpha_theta(0, 2) = 0.0;

					cov_r_alpha_theta(1, 0) = 0.0;
					cov_r_alpha_theta(1, 1) = sigma_polar_angle * sigma_polar_angle;
					cov_r_alpha_theta(1, 2) = 0.0;

					cov_r_alpha_theta(2, 0) = 0.0;
					cov_r_alpha_theta(2, 1) = 0.0;
					cov_r_alpha_theta(2, 2) = sigma_azimuthal_angle * sigma_azimuthal_angle;

					Eigen::Matrix<double, 3, 3> cov_xyz = j * cov_r_alpha_theta * j.transpose();
					std::vector<Eigen::Vector3d> points = get_points_normal_distribution(cov_xyz, point_local, num_extended_points);


					//for (const auto &ppp : points)
					//{
					//	std::cout << "---" << std::endl;
					//	std::cout << ppp << std::endl;
					//}
					//exit(1);

					for (const auto &pp : points)
					{
						auto ppg = m * pp;

						cov(0, 0) += (mean.x() - ppg.x()) * (mean.x() - ppg.x());
						cov(0, 1) += (mean.x() - ppg.x()) * (mean.y() - ppg.y());
						cov(0, 2) += (mean.x() - ppg.x()) * (mean.z() - ppg.z());
						cov(1, 0) += (mean.y() - ppg.y()) * (mean.x() - ppg.x());
						cov(1, 1) += (mean.y() - ppg.y()) * (mean.y() - ppg.y());
						cov(1, 2) += (mean.y() - ppg.y()) * (mean.z() - ppg.z());
						cov(2, 0) += (mean.z() - ppg.z()) * (mean.x() - ppg.x());
						cov(2, 1) += (mean.z() - ppg.z()) * (mean.y() - ppg.y());
						cov(2, 2) += (mean.z() - ppg.z()) * (mean.z() - ppg.z());
						counter++;
					}
				}
			}
			else*/
			{
				cov(0, 0) += (mean.x() - p.x) * (mean.x() - p.x);
				cov(0, 1) += (mean.x() - p.x) * (mean.y() - p.y);
				cov(0, 2) += (mean.x() - p.x) * (mean.z() - p.z);
				cov(1, 0) += (mean.y() - p.y) * (mean.x() - p.x);
				cov(1, 1) += (mean.y() - p.y) * (mean.y() - p.y);
				cov(1, 2) += (mean.y() - p.y) * (mean.z() - p.z);
				cov(2, 0) += (mean.z() - p.z) * (mean.x() - p.x);
				cov(2, 1) += (mean.z() - p.z) * (mean.y() - p.y);
				cov(2, 2) += (mean.z() - p.z) * (mean.z() - p.z);
				counter++;
			}
		}

		if (counter < 5)
		{
			continue;
		}

		if (is_generalized)
		{
			Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(cov, Eigen::ComputeEigenvectors);
			// Eigen::Matrix3d eigenVectorsPCA = eigen_solver.eigenvectors();
			Eigen::Vector3d eigen_values = eigen_solver.eigenvalues();

			double min_eigen_value = eigen_values.x();
			if (eigen_values.y() < min_eigen_value)
			{
				min_eigen_value = eigen_values.y();
			}

			if (eigen_values.z() < min_eigen_value)
			{
				min_eigen_value = eigen_values.z();
			}

			if (min_eigen_value > 3 * sigma_r)
			{
				continue;
			} // else{
			  // std::cout << min_eigen_value << " ";
			  //}
			  // std::cout << min_eigen_value << " ";

			Eigen::Matrix3d eigenVectorsPCA = eigen_solver.eigenvectors();
			Eigen::Vector3d nv = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
			nv.normalize();

			const auto &p = (*pp)[(*index_pair_internal)[b.index_begin].index_of_point];
			const auto &m = (*mposes)[p.index_pose];

			//  flip towards viewport
			if (nv.dot(m.translation() - mean) < 0.0)
			{
				nv *= -1.0;
			}
			//this_bucket.normal_vector = nv;
			(*buckets)[ii].normal_vector = nv;
		}

		cov /= counter;

		(*buckets)[ii].mean = mean;
		(*buckets)[ii].cov = cov;

		if (compute_only_mean_and_cov)
		{
			continue;
		}

		Eigen::Matrix3d infm = cov.inverse();

		if (!(infm(0, 0) == infm(0, 0)))
			continue;
		if (!(infm(0, 1) == infm(0, 1)))
			continue;
		if (!(infm(0, 2) == infm(0, 2)))
			continue;

		if (!(infm(1, 0) == infm(1, 0)))
			continue;
		if (!(infm(1, 1) == infm(1, 1)))
			continue;
		if (!(infm(1, 2) == infm(1, 2)))
			continue;

		if (!(infm(2, 0) == infm(2, 0)))
			continue;
		if (!(infm(2, 1) == infm(2, 1)))
			continue;
		if (!(infm(2, 2) == infm(2, 2)))
			continue;

		double threshold = 1000000.0;

		if (infm(0, 0) > threshold)
			continue;
		if (infm(0, 1) > threshold)
			continue;
		if (infm(0, 2) > threshold)
			continue;
		if (infm(1, 0) > threshold)
			continue;
		if (infm(1, 1) > threshold)
			continue;
		if (infm(1, 2) > threshold)
			continue;
		if (infm(2, 0) > threshold)
			continue;
		if (infm(2, 1) > threshold)
			continue;
		if (infm(2, 2) > threshold)
			continue;

		if (infm(0, 0) < -threshold)
			continue;
		if (infm(0, 1) < -threshold)
			continue;
		if (infm(0, 2) < -threshold)
			continue;
		if (infm(1, 0) < -threshold)
			continue;
		if (infm(1, 1) < -threshold)
			continue;
		if (infm(1, 2) < -threshold)
			continue;
		if (infm(2, 0) < -threshold)
			continue;
		if (infm(2, 1) < -threshold)
			continue;
		if (infm(2, 2) < -threshold)
			continue;

		for (int index = b.index_begin; index < b.index_end; index++)
		{
			std::vector<Eigen::Vector3d> points_local;

			if (is_generalized)
			{
				const auto &p = (*pp)[(*index_pair_internal)[index].index_of_point];
				const auto &m = (*mposes)[p.index_pose];
				const auto &minv = (*mposes_inv)[p.index_pose];

				if ((*buckets)[ii].normal_vector.dot(m.translation() - (*buckets)[ii].mean) < 0.0)
				{
					continue;
				}

				Eigen::Vector3d point_local(p.x, p.y, p.z);
				point_local = minv * point_local;

				double norm = point_local.norm();
				double r = norm;
				double polar_angle = acos(point_local.z() / norm);
				double azimuthal_angle = atan(point_local.y() / point_local.x());

				Eigen::Matrix<double, 3, 3> j;
				elementary_error_theory_for_terrestrial_laser_scanner_jacobian(j, r, polar_angle, azimuthal_angle);

				Eigen::Matrix<double, 3, 3> cov_r_alpha_theta;
				cov_r_alpha_theta(0, 0) = sigma_r * sigma_r;
				cov_r_alpha_theta(0, 1) = 0.0;
				cov_r_alpha_theta(0, 2) = 0.0;

				cov_r_alpha_theta(1, 0) = 0.0;
				cov_r_alpha_theta(1, 1) = sigma_polar_angle * sigma_polar_angle;
				cov_r_alpha_theta(1, 2) = 0.0;

				cov_r_alpha_theta(2, 0) = 0.0;
				cov_r_alpha_theta(2, 1) = 0.0;
				cov_r_alpha_theta(2, 2) = sigma_azimuthal_angle * sigma_azimuthal_angle;

				Eigen::Matrix<double, 3, 3> cov_xyz = j * cov_r_alpha_theta * j.transpose();
				std::vector<Eigen::Vector3d> points = get_points_normal_distribution(cov_xyz, point_local, num_extended_points);

				for (const auto &ppl : points)
				{
					points_local.push_back(ppl);
				}
			}
			else
			{
				const auto &p = (*pp)[(*index_pair_internal)[index].index_of_point];
				Eigen::Vector3d point_local(p.x, p.y, p.z);
				point_local = (*mposes_inv)[p.index_pose] * point_local;
				points_local.push_back(point_local);
			}

			for (const auto &point_local : points_local)
			{
				const auto &p = (*pp)[(*index_pair_internal)[index].index_of_point];
				int ir = tripletListB.size();
				double delta_x;
				double delta_y;
				double delta_z;

				Eigen::Affine3d m_pose = (*mposes)[p.index_pose];
				Eigen::Vector3d p_s(point_local.x(), point_local.y(), point_local.z());
				Eigen::Vector3d p_t(mean.x(), mean.y(), mean.z());
				md += sqrt((p_s - p_t).transpose() * infm * (p_s - p_t));
				md_count += 1.0;

				if (rotation_matrix_parametrization == NDT::RotationMatrixParametrization::tait_bryan_xyz)
				{
					Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
					//-----------------------
					if (pose_convention == NDT::PoseConvention::wc)
					{
						TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix((*mposes)[p.index_pose]);

						point_to_point_source_to_target_tait_bryan_wc(delta_x, delta_y, delta_z,
																	  pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
																	  point_local.x(), point_local.y(), point_local.z(), mean.x(), mean.y(), mean.z());

						point_to_point_source_to_target_tait_bryan_wc_jacobian(jacobian,
																			   pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
																			   point_local.x(), point_local.y(), point_local.z());
					}
					else
					{
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
					for (int row = 0; row < 3; row++)
					{
						for (int col = 0; col < 6; col++)
						{
							if (jacobian(row, col) != 0.0)
							{
								tripletListA.emplace_back(ir + row, c + col, -jacobian(row, col));
							}
						}
					}
				}
				else if (rotation_matrix_parametrization == NDT::RotationMatrixParametrization::rodrigues)
				{
					Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
					//-----------------------
					if (pose_convention == NDT::PoseConvention::wc)
					{
						RodriguesPose pose_s = pose_rodrigues_from_affine_matrix((*mposes)[p.index_pose]);

						point_to_point_source_to_target_rodrigues_wc(delta_x, delta_y, delta_z,
																	 pose_s.px, pose_s.py, pose_s.pz, pose_s.sx, pose_s.sy, pose_s.sz,
																	 point_local.x(), point_local.y(), point_local.z(), mean.x(), mean.y(), mean.z());

						point_to_point_source_to_target_rodrigues_wc_jacobian(jacobian,
																			  pose_s.px, pose_s.py, pose_s.pz, pose_s.sx, pose_s.sy, pose_s.sz,
																			  point_local.x(), point_local.y(), point_local.z());
					}
					else
					{
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
					for (int row = 0; row < 3; row++)
					{
						for (int col = 0; col < 6; col++)
						{
							if (jacobian(row, col) != 0.0)
							{
								tripletListA.emplace_back(ir + row, c + col, -jacobian(row, col));
							}
						}
					}
				}
				else if (rotation_matrix_parametrization == NDT::RotationMatrixParametrization::quaternion)
				{
					Eigen::Matrix<double, 3, 7, Eigen::RowMajor> jacobian;
					//-----------------------
					if (pose_convention == NDT::PoseConvention::wc)
					{
						QuaternionPose pose_s = pose_quaternion_from_affine_matrix((*mposes)[p.index_pose]);

						point_to_point_source_to_target_quaternion_wc(delta_x, delta_y, delta_z,
																	  pose_s.px, pose_s.py, pose_s.pz, pose_s.q0, pose_s.q1, pose_s.q2, pose_s.q3,
																	  point_local.x(), point_local.y(), point_local.z(), mean.x(), mean.y(), mean.z());

						point_to_point_source_to_target_quaternion_wc_jacobian(jacobian,
																			   pose_s.px, pose_s.py, pose_s.pz, pose_s.q0, pose_s.q1, pose_s.q2, pose_s.q3,
																			   point_local.x(), point_local.y(), point_local.z());
					}
					else
					{
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
					for (int row = 0; row < 3; row++)
					{
						for (int col = 0; col < 7; col++)
						{
							if (jacobian(row, col) != 0.0)
							{
								tripletListA.emplace_back(ir + row, c + col, -jacobian(row, col));
							}
						}
					}
				}
				else if (rotation_matrix_parametrization == NDT::RotationMatrixParametrization::lie_algebra_left_jacobian)
				{
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
					// tripletListA.emplace_back(ir, ic + 1, 0);
					// tripletListA.emplace_back(ir, ic + 2, 0);
					tripletListA.emplace_back(ir, ic + 3, -Rpx(0, 0));
					tripletListA.emplace_back(ir, ic + 4, -Rpx(0, 1));
					tripletListA.emplace_back(ir, ic + 5, -Rpx(0, 2));

					// tripletListA.emplace_back(ir + 1, ic + 0, 0);
					tripletListA.emplace_back(ir + 1, ic + 1, 1);
					// tripletListA.emplace_back(ir + 1, ic + 2, 0);
					tripletListA.emplace_back(ir + 1, ic + 3, -Rpx(1, 0));
					tripletListA.emplace_back(ir + 1, ic + 4, -Rpx(1, 1));
					tripletListA.emplace_back(ir + 1, ic + 5, -Rpx(1, 2));

					// tripletListA.emplace_back(ir + 2, ic + 0, 0);
					// tripletListA.emplace_back(ir + 2, ic + 1, 0);
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
				else if (rotation_matrix_parametrization == NDT::RotationMatrixParametrization::lie_algebra_right_jacobian)
				{
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

			if (tripletListB.size() > 100000)
			{
				Eigen::SparseMatrix<double> matA(tripletListB.size(), trajectory_size * number_of_unknowns);
				Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
				Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

				matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
				matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
				matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

				Eigen::SparseMatrix<double> AtPAt(trajectory_size * number_of_unknowns, trajectory_size * number_of_unknowns);
				Eigen::SparseMatrix<double> AtPBt(trajectory_size * number_of_unknowns, 1);
				Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
				AtPAt = AtP * matA;
				AtPBt = AtP * matB;

				if (init)
				{
					AtPAt_tmp = AtPAt;
					AtPBt_tmp = AtPBt;
					init = false;
				}
				else
				{
					AtPAt_tmp += AtPAt;
					AtPBt_tmp += AtPBt;
				}

				tripletListA.clear();
				tripletListP.clear();
				tripletListB.clear();
			}
		}
	}
	(*sumssr) = ssr;
	(*sums_obs) = sum_obs;
	(*md_out) = md;
	(*md_count_out) = md_count;

	// std::cout << md << " ";

	if (tripletListB.size() > 0)
	{
		Eigen::SparseMatrix<double> matA(tripletListB.size(), trajectory_size * number_of_unknowns);
		Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
		Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

		matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
		matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
		matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

		Eigen::SparseMatrix<double> AtPAt(trajectory_size * number_of_unknowns, trajectory_size * number_of_unknowns);
		Eigen::SparseMatrix<double> AtPBt(trajectory_size * number_of_unknowns, 1);

		{
			Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
			AtPAt = AtP * matA;
			AtPBt = AtP * matB;

			if (init)
			{
				(*AtPA) = AtPAt;
				(*AtPB) = AtPBt;
			}
			else
			{
				AtPAt_tmp += AtPAt;
				AtPBt_tmp += AtPBt;

				(*AtPA) = AtPAt_tmp;
				(*AtPB) = AtPBt_tmp;
			}
			// std::cout << AtPAt << std::endl;
		}
	}
	else
	{
		(*AtPA) = AtPAt_tmp;
		(*AtPB) = AtPBt_tmp;
	}
}

void ndt_jobi(int i, NDT::Job *job, std::vector<NDT::Bucket> *buckets, Eigen::SparseMatrix<double> *AtPA,
			  Eigen::SparseMatrix<double> *AtPB, std::vector<NDT::PointBucketIndexPair> *index_pair_internal, std::vector<Point3Di> *pp,
			  std::vector<Eigen::Affine3d> *mposes, std::vector<Eigen::Affine3d> *mposes_inv, size_t trajectory_size,
			  NDT::PoseConvention pose_convention, NDT::RotationMatrixParametrization rotation_matrix_parametrization, int number_of_unknowns, double *sumssr, int *sums_obs,
			  bool is_generalized, double sigma_r, double sigma_polar_angle, double sigma_azimuthal_angle, int num_extended_points, double *md_out, double *md_count_out,
			  bool compute_only_mean_and_cov)
{
	std::vector<Eigen::Triplet<double>> tripletListA;
	std::vector<Eigen::Triplet<double>> tripletListP;
	std::vector<Eigen::Triplet<double>> tripletListB;

	Eigen::SparseMatrix<double> AtPAt_tmp(trajectory_size * number_of_unknowns, trajectory_size * number_of_unknowns);
	Eigen::SparseMatrix<double> AtPBt_tmp(trajectory_size * number_of_unknowns, 1);

	double ssr = 0.0;
	int sum_obs = 0;
	bool init = true;
	double md = 0.0;
	double md_count = 0.0;

	for (size_t ii = job->index_begin_inclusive; ii < job->index_end_exclusive; ii++)
	{
		// std::cout << ii << " ";
		NDT::Bucket &b = (*buckets)[ii];
		if (b.number_of_points < 5)
		{
			continue;
		}

		Eigen::Vector3d mean(0, 0, 0);
		Eigen::Matrix3d cov;
		cov.setZero();

		int counter = 0;
		for (int index = b.index_begin; index < b.index_end; index++)
		{
			if (is_generalized)
			{
				const auto &p = (*pp)[(*index_pair_internal)[index].index_of_point];
				const auto &m = (*mposes)[p.index_pose];
				const auto &minv = (*mposes_inv)[p.index_pose];

				// Eigen::Vector3d point_local(p.x, p.y, p.z);
				Eigen::Vector3d point_local = p.point;
				point_local = minv * point_local;

				double norm = point_local.norm();
				double r = norm;
				double polar_angle = acos(point_local.z() / norm);
				double azimuthal_angle = atan(point_local.y() / point_local.x());

				Eigen::Matrix<double, 3, 3> j;
				elementary_error_theory_for_terrestrial_laser_scanner_jacobian(j, r, polar_angle, azimuthal_angle);

				Eigen::Matrix<double, 3, 3> cov_r_alpha_theta;
				cov_r_alpha_theta(0, 0) = sigma_r * sigma_r;
				cov_r_alpha_theta(0, 1) = 0.0;
				cov_r_alpha_theta(0, 2) = 0.0;

				cov_r_alpha_theta(1, 0) = 0.0;
				cov_r_alpha_theta(1, 1) = sigma_polar_angle * sigma_polar_angle;
				cov_r_alpha_theta(1, 2) = 0.0;

				cov_r_alpha_theta(2, 0) = 0.0;
				cov_r_alpha_theta(2, 1) = 0.0;
				cov_r_alpha_theta(2, 2) = sigma_azimuthal_angle * sigma_azimuthal_angle;

				Eigen::Matrix<double, 3, 3> cov_xyz = j * cov_r_alpha_theta * j.transpose();
				std::vector<Eigen::Vector3d> points = get_points_normal_distribution(cov_xyz, point_local, num_extended_points);

				for (const auto &ppl : points)
				{
					mean += m * ppl;
					counter++;
				}
			}
			else
			{
				const auto &p = (*pp)[(*index_pair_internal)[index].index_of_point];
				mean += p.point; //::Vector3d(p.x, p.y, p.z);
				counter++;
				// std::cout << counter << " ";
			}
		}

		mean /= counter;

		counter = 0;
		for (int index = b.index_begin; index < b.index_end; index++)
		{
			const auto &p = (*pp)[(*index_pair_internal)[index].index_of_point];
			const auto &m = (*mposes)[p.index_pose];
			const auto &minv = (*mposes_inv)[p.index_pose];

			if (is_generalized)
			{
				Eigen::Vector3d point_local = p.point; //(p.x, p.y, p.z);
				point_local = minv * point_local;

				double norm = point_local.norm();
				double r = norm;
				double polar_angle = acos(point_local.z() / norm);
				double azimuthal_angle = atan(point_local.y() / point_local.x());

				Eigen::Matrix<double, 3, 3> j;
				elementary_error_theory_for_terrestrial_laser_scanner_jacobian(j, r, polar_angle, azimuthal_angle);

				Eigen::Matrix<double, 3, 3> cov_r_alpha_theta;
				cov_r_alpha_theta(0, 0) = sigma_r * sigma_r;
				cov_r_alpha_theta(0, 1) = 0.0;
				cov_r_alpha_theta(0, 2) = 0.0;

				cov_r_alpha_theta(1, 0) = 0.0;
				cov_r_alpha_theta(1, 1) = sigma_polar_angle * sigma_polar_angle;
				cov_r_alpha_theta(1, 2) = 0.0;

				cov_r_alpha_theta(2, 0) = 0.0;
				cov_r_alpha_theta(2, 1) = 0.0;
				cov_r_alpha_theta(2, 2) = sigma_azimuthal_angle * sigma_azimuthal_angle;

				Eigen::Matrix<double, 3, 3> cov_xyz = j * cov_r_alpha_theta * j.transpose();
				std::vector<Eigen::Vector3d> points = get_points_normal_distribution(cov_xyz, point_local, num_extended_points);

				for (const auto &pp : points)
				{
					auto ppg = m * pp;

					cov(0, 0) += (mean.x() - ppg.x()) * (mean.x() - ppg.x());
					cov(0, 1) += (mean.x() - ppg.x()) * (mean.y() - ppg.y());
					cov(0, 2) += (mean.x() - ppg.x()) * (mean.z() - ppg.z());
					cov(1, 0) += (mean.y() - ppg.y()) * (mean.x() - ppg.x());
					cov(1, 1) += (mean.y() - ppg.y()) * (mean.y() - ppg.y());
					cov(1, 2) += (mean.y() - ppg.y()) * (mean.z() - ppg.z());
					cov(2, 0) += (mean.z() - ppg.z()) * (mean.x() - ppg.x());
					cov(2, 1) += (mean.z() - ppg.z()) * (mean.y() - ppg.y());
					cov(2, 2) += (mean.z() - ppg.z()) * (mean.z() - ppg.z());
					counter++;
				}
			}
			else
			{
				cov(0, 0) += (mean.x() - p.point.x()) * (mean.x() - p.point.x());
				cov(0, 1) += (mean.x() - p.point.x()) * (mean.y() - p.point.y());
				cov(0, 2) += (mean.x() - p.point.x()) * (mean.z() - p.point.z());
				cov(1, 0) += (mean.y() - p.point.y()) * (mean.x() - p.point.x());
				cov(1, 1) += (mean.y() - p.point.y()) * (mean.y() - p.point.y());
				cov(1, 2) += (mean.y() - p.point.y()) * (mean.z() - p.point.z());
				cov(2, 0) += (mean.z() - p.point.z()) * (mean.x() - p.point.x());
				cov(2, 1) += (mean.z() - p.point.z()) * (mean.y() - p.point.y());
				cov(2, 2) += (mean.z() - p.point.z()) * (mean.z() - p.point.z());
				counter++;
			}
		}
		cov /= counter;

		(*buckets)[ii].mean = mean;
		(*buckets)[ii].cov = cov;

		// std::cout << mean << " ";
#if 0
		if(compute_only_mean_and_cov){
			continue;
		}

		Eigen::Matrix3d infm = cov.inverse();

		if (!(infm(0, 0) == infm(0, 0)))
			continue;
		if (!(infm(0, 1) == infm(0, 1)))
			continue;
		if (!(infm(0, 2) == infm(0, 2)))
			continue;

		if (!(infm(1, 0) == infm(1, 0)))
			continue;
		if (!(infm(1, 1) == infm(1, 1)))
			continue;
		if (!(infm(1, 2) == infm(1, 2)))
			continue;

		if (!(infm(2, 0) == infm(2, 0)))
			continue;
		if (!(infm(2, 1) == infm(2, 1)))
			continue;
		if (!(infm(2, 2) == infm(2, 2)))
			continue;

		double threshold = 10000.0;

		if (infm(0, 0) > threshold)
			continue;
		if (infm(0, 1) > threshold)
			continue;
		if (infm(0, 2) > threshold)
			continue;
		if (infm(1, 0) > threshold)
			continue;
		if (infm(1, 1) > threshold)
			continue;
		if (infm(1, 2) > threshold)
			continue;
		if (infm(2, 0) > threshold)
			continue;
		if (infm(2, 1) > threshold)
			continue;
		if (infm(2, 2) > threshold)
			continue;

		if (infm(0, 0) < -threshold)
			continue;
		if (infm(0, 1) < -threshold)
			continue;
		if (infm(0, 2) < -threshold)
			continue;
		if (infm(1, 0) < -threshold)
			continue;
		if (infm(1, 1) < -threshold)
			continue;
		if (infm(1, 2) < -threshold)
			continue;
		if (infm(2, 0) < -threshold)
			continue;
		if (infm(2, 1) < -threshold)
			continue;
		if (infm(2, 2) < -threshold)
			continue;

		for (int index = b.index_begin; index < b.index_end; index++)
		{
			std::vector<Eigen::Vector3d> points_local;

			if (is_generalized)
			{
				const auto &p = (*pp)[(*index_pair_internal)[index].index_of_point];
				const auto &m = (*mposes)[p.index_pose];
				const auto &minv = (*mposes_inv)[p.index_pose];

				Eigen::Vector3d point_local = p.point;//(p.x, p.y, p.z);
				point_local = minv * point_local;

				double norm = point_local.norm();
				double r = norm;
				double polar_angle = acos(point_local.z() / norm);
				double azimuthal_angle = atan(point_local.y() / point_local.x());

				Eigen::Matrix<double, 3, 3> j;
				elementary_error_theory_for_terrestrial_laser_scanner_jacobian(j, r, polar_angle, azimuthal_angle);

				Eigen::Matrix<double, 3, 3> cov_r_alpha_theta;
				cov_r_alpha_theta(0, 0) = sigma_r * sigma_r;
				cov_r_alpha_theta(0, 1) = 0.0;
				cov_r_alpha_theta(0, 2) = 0.0;

				cov_r_alpha_theta(1, 0) = 0.0;
				cov_r_alpha_theta(1, 1) = sigma_polar_angle * sigma_polar_angle;
				cov_r_alpha_theta(1, 2) = 0.0;

				cov_r_alpha_theta(2, 0) = 0.0;
				cov_r_alpha_theta(2, 1) = 0.0;
				cov_r_alpha_theta(2, 2) = sigma_azimuthal_angle * sigma_azimuthal_angle;

				Eigen::Matrix<double, 3, 3> cov_xyz = j * cov_r_alpha_theta * j.transpose();
				std::vector<Eigen::Vector3d> points = get_points_normal_distribution(cov_xyz, point_local, num_extended_points);

				for (const auto &ppl : points)
				{
					points_local.push_back(ppl);
				}
			}
			else
			{
				const auto &p = (*pp)[(*index_pair_internal)[index].index_of_point];
				Eigen::Vector3d point_local = p.point;//(p.x, p.y, p.z);
				point_local = (*mposes_inv)[p.index_pose] * point_local;
				points_local.push_back(point_local);
			}

			for (const auto &point_local : points_local)
			{
				const auto &p = (*pp)[(*index_pair_internal)[index].index_of_point];
				int ir = tripletListB.size();
				double delta_x;
				double delta_y;
				double delta_z;

				Eigen::Affine3d m_pose = (*mposes)[p.index_pose];
				Eigen::Vector3d p_s(point_local.x(), point_local.y(), point_local.z());
				Eigen::Vector3d p_t(mean.x(), mean.y(), mean.z());
				md += sqrt((p_s - p_t).transpose() * infm * (p_s - p_t));
				md_count += 1.0;

				if (rotation_matrix_parametrization == NDT::RotationMatrixParametrization::tait_bryan_xyz)
				{
					Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
					//-----------------------
					if (pose_convention == NDT::PoseConvention::wc)
					{
						TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix((*mposes)[p.index_pose]);

						point_to_point_source_to_target_tait_bryan_wc(delta_x, delta_y, delta_z,
																	  pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
																	  point_local.x(), point_local.y(), point_local.z(), mean.x(), mean.y(), mean.z());

						point_to_point_source_to_target_tait_bryan_wc_jacobian(jacobian,
																			   pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
																			   point_local.x(), point_local.y(), point_local.z());
					}
					else
					{
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
					for (int row = 0; row < 3; row++)
					{
						for (int col = 0; col < 6; col++)
						{
							if (jacobian(row, col) != 0.0)
							{
								tripletListA.emplace_back(ir + row, c + col, -jacobian(row, col));
							}
						}
					}
				}
				else if (rotation_matrix_parametrization == NDT::RotationMatrixParametrization::rodrigues)
				{
					Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
					//-----------------------
					if (pose_convention == NDT::PoseConvention::wc)
					{
						RodriguesPose pose_s = pose_rodrigues_from_affine_matrix((*mposes)[p.index_pose]);

						point_to_point_source_to_target_rodrigues_wc(delta_x, delta_y, delta_z,
																	 pose_s.px, pose_s.py, pose_s.pz, pose_s.sx, pose_s.sy, pose_s.sz,
																	 point_local.x(), point_local.y(), point_local.z(), mean.x(), mean.y(), mean.z());

						point_to_point_source_to_target_rodrigues_wc_jacobian(jacobian,
																			  pose_s.px, pose_s.py, pose_s.pz, pose_s.sx, pose_s.sy, pose_s.sz,
																			  point_local.x(), point_local.y(), point_local.z());
					}
					else
					{
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
					for (int row = 0; row < 3; row++)
					{
						for (int col = 0; col < 6; col++)
						{
							if (jacobian(row, col) != 0.0)
							{
								tripletListA.emplace_back(ir + row, c + col, -jacobian(row, col));
							}
						}
					}
				}
				else if (rotation_matrix_parametrization == NDT::RotationMatrixParametrization::quaternion)
				{
					Eigen::Matrix<double, 3, 7, Eigen::RowMajor> jacobian;
					//-----------------------
					if (pose_convention == NDT::PoseConvention::wc)
					{
						QuaternionPose pose_s = pose_quaternion_from_affine_matrix((*mposes)[p.index_pose]);

						point_to_point_source_to_target_quaternion_wc(delta_x, delta_y, delta_z,
																	  pose_s.px, pose_s.py, pose_s.pz, pose_s.q0, pose_s.q1, pose_s.q2, pose_s.q3,
																	  point_local.x(), point_local.y(), point_local.z(), mean.x(), mean.y(), mean.z());

						point_to_point_source_to_target_quaternion_wc_jacobian(jacobian,
																			   pose_s.px, pose_s.py, pose_s.pz, pose_s.q0, pose_s.q1, pose_s.q2, pose_s.q3,
																			   point_local.x(), point_local.y(), point_local.z());
					}
					else
					{
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
					for (int row = 0; row < 3; row++)
					{
						for (int col = 0; col < 7; col++)
						{
							if (jacobian(row, col) != 0.0)
							{
								tripletListA.emplace_back(ir + row, c + col, -jacobian(row, col));
							}
						}
					}
				}
				else if (rotation_matrix_parametrization == NDT::RotationMatrixParametrization::lie_algebra_left_jacobian)
				{
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
					// tripletListA.emplace_back(ir, ic + 1, 0);
					// tripletListA.emplace_back(ir, ic + 2, 0);
					tripletListA.emplace_back(ir, ic + 3, -Rpx(0, 0));
					tripletListA.emplace_back(ir, ic + 4, -Rpx(0, 1));
					tripletListA.emplace_back(ir, ic + 5, -Rpx(0, 2));

					// tripletListA.emplace_back(ir + 1, ic + 0, 0);
					tripletListA.emplace_back(ir + 1, ic + 1, 1);
					// tripletListA.emplace_back(ir + 1, ic + 2, 0);
					tripletListA.emplace_back(ir + 1, ic + 3, -Rpx(1, 0));
					tripletListA.emplace_back(ir + 1, ic + 4, -Rpx(1, 1));
					tripletListA.emplace_back(ir + 1, ic + 5, -Rpx(1, 2));

					// tripletListA.emplace_back(ir + 2, ic + 0, 0);
					// tripletListA.emplace_back(ir + 2, ic + 1, 0);
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
				else if (rotation_matrix_parametrization == NDT::RotationMatrixParametrization::lie_algebra_right_jacobian)
				{
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

			if (tripletListB.size() > 100000)
			{
				Eigen::SparseMatrix<double> matA(tripletListB.size(), trajectory_size * number_of_unknowns);
				Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
				Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

				matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
				matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
				matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

				Eigen::SparseMatrix<double> AtPAt(trajectory_size * number_of_unknowns, trajectory_size * number_of_unknowns);
				Eigen::SparseMatrix<double> AtPBt(trajectory_size * number_of_unknowns, 1);
				Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
				AtPAt = AtP * matA;
				AtPBt = AtP * matB;

				if (init)
				{
					AtPAt_tmp = AtPAt;
					AtPBt_tmp = AtPBt;
					init = false;
				}
				else
				{
					AtPAt_tmp += AtPAt;
					AtPBt_tmp += AtPBt;
				}

				tripletListA.clear();
				tripletListP.clear();
				tripletListB.clear();
			}
		}
#endif
	}

#if 0
	(*sumssr) = ssr;
	(*sums_obs) = sum_obs;
	(*md_out) = md;
	(*md_count_out) = md_count;

	// std::cout << md << " ";

	if (tripletListB.size() > 0)
	{
		Eigen::SparseMatrix<double> matA(tripletListB.size(), trajectory_size * number_of_unknowns);
		Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
		Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

		matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
		matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
		matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

		Eigen::SparseMatrix<double> AtPAt(trajectory_size * number_of_unknowns, trajectory_size * number_of_unknowns);
		Eigen::SparseMatrix<double> AtPBt(trajectory_size * number_of_unknowns, 1);

		{
			Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
			AtPAt = AtP * matA;
			AtPBt = AtP * matB;

			if (init)
			{
				(*AtPA) = AtPAt;
				(*AtPB) = AtPBt;
			}
			else
			{
				AtPAt_tmp += AtPAt;
				AtPBt_tmp += AtPBt;

				(*AtPA) = AtPAt_tmp;
				(*AtPB) = AtPBt_tmp;
			}
			// std::cout << AtPAt << std::endl;
		}
	}
	else
	{
		(*AtPA) = AtPAt_tmp;
		(*AtPB) = AtPBt_tmp;
	}
#endif
}

bool NDT::optimize(std::vector<PointCloud> &point_clouds, bool compute_only_mahalanobis_distance)
{
	auto start = std::chrono::system_clock::now();

	OptimizationAlgorithm optimization_algorithm;
	if (is_gauss_newton)
	{
		optimization_algorithm = OptimizationAlgorithm::gauss_newton;
	}
	if (is_levenberg_marguardt)
	{
		optimization_algorithm = OptimizationAlgorithm::levenberg_marguardt;
	}

	PoseConvention pose_convention;
	if (is_wc)
	{
		pose_convention = PoseConvention::wc;
	}
	if (is_cw)
	{
		pose_convention = PoseConvention::cw;
	}

	RotationMatrixParametrization rotation_matrix_parametrization;
	if (is_tait_bryan_angles)
	{
		rotation_matrix_parametrization = RotationMatrixParametrization::tait_bryan_xyz;
	}
	else if (is_rodrigues)
	{
		rotation_matrix_parametrization = RotationMatrixParametrization::rodrigues;
	}
	else if (is_quaternion)
	{
		rotation_matrix_parametrization = RotationMatrixParametrization::quaternion;
	}
	else if (is_lie_algebra_left_jacobian)
	{
		rotation_matrix_parametrization = RotationMatrixParametrization::lie_algebra_left_jacobian;
	}
	else if (is_lie_algebra_right_jacobian)
	{
		rotation_matrix_parametrization = RotationMatrixParametrization::lie_algebra_right_jacobian;
	}

	if (is_rodrigues || is_quaternion || is_lie_algebra_left_jacobian || is_lie_algebra_right_jacobian)
	{
		for (size_t i = 0; i < point_clouds.size(); i++)
		{
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
	if (is_tait_bryan_angles || is_rodrigues || is_lie_algebra_left_jacobian || is_lie_algebra_right_jacobian)
	{
		number_of_unknowns = 6;
	}
	if (is_quaternion)
	{
		number_of_unknowns = 7;
	}

	double lm_lambda = 0.0001;
	double previous_rms = std::numeric_limits<double>::max();
	int number_of_lm_iterations = 0;

	std::vector<Eigen::Affine3d> m_poses_tmp;
	if (is_levenberg_marguardt)
	{
		m_poses_tmp.clear();
		for (size_t i = 0; i < point_clouds.size(); i++)
		{
			m_poses_tmp.push_back(point_clouds[i].m_pose);
		}
	}

	for (int iter = 0; iter < number_of_iterations; iter++)
	{

#if 1
		//////rgd external///////
		// double min_x = std::numeric_limits<double>::max();
		// double max_x = std::numeric_limits<double>::lowest();

		// double min_y = std::numeric_limits<double>::max();
		// double max_y = std::numeric_limits<double>::lowest();

		// double min_z = std::numeric_limits<double>::max();
		// double max_z = std::numeric_limits<double>::lowest();

		std::cout << "building points_global_external begin" << std::endl;
		std::vector<Point3D> points_global_external;
		size_t num_total_points = 0;
		for (int i = 0; i < point_clouds.size(); i++)
		{
			num_total_points += point_clouds[i].points_local.size();
		}
		points_global_external.reserve(num_total_points);
		Eigen::Vector3d vt;
		Point3D p;
		for (int i = 0; i < point_clouds.size(); i++)
		{
			std::cout << "processing point_cloud [" << i + 1 << "] of " << point_clouds.size() << std::endl;

			for (int j = 0; j < point_clouds[i].points_local.size(); j++)
			{
				vt = point_clouds[i].m_pose * point_clouds[i].points_local[j];
				p.x = vt.x();
				p.y = vt.y();
				p.z = vt.z();
				p.index_pose = i;
				points_global_external.emplace_back(p);
			}
		}
		std::cout << "building points_global_external end" << std::endl;

		std::vector<PointBucketIndexPair> index_pair_external;
		std::vector<Bucket> buckets_external;

		GridParameters rgd_params_external;
		rgd_params_external.resolution_X = this->bucket_size_external[0];
		rgd_params_external.resolution_Y = this->bucket_size_external[1];
		rgd_params_external.resolution_Z = this->bucket_size_external[2];

		int bbext = this->bucket_size_external[0];
		if (this->bucket_size_external[1] > bbext)
			bbext = this->bucket_size_external[1];
		if (this->bucket_size_external[2] > bbext)
			bbext = this->bucket_size_external[2];
		rgd_params_external.bounding_box_extension = bbext;

		std::cout << "building external grid begin" << std::endl;
		grid_calculate_params(points_global_external, rgd_params_external);
		int num_threads = 1;
		if (buckets_external.size() > this->number_of_threads)
		{
			num_threads = this->number_of_threads;
		}
		build_rgd(points_global_external, index_pair_external, buckets_external, rgd_params_external, num_threads);
		std::vector<Bucket> buckets_external_reduced;

		for (const auto &b : buckets_external)
		{
			// std::cout << "number of points inside bucket: " << b.number_of_points << std::endl;
			if (b.number_of_points > 1000)
			{
				buckets_external_reduced.push_back(b);
			}
		}
		buckets_external = buckets_external_reduced;
		buckets_external_reduced.clear();

		std::sort(buckets_external.begin(), buckets_external.end(), [](const Bucket &a, const Bucket &b)
				  { return (a.number_of_points > b.number_of_points); });

		std::cout << "building external grid end" << std::endl;
		std::cout << "number active buckets external: " << buckets_external.size() << std::endl;

		bool init = false;
		Eigen::SparseMatrix<double> AtPA_ndt(point_clouds.size() * number_of_unknowns, point_clouds.size() * number_of_unknowns);
		Eigen::SparseMatrix<double> AtPB_ndt(point_clouds.size() * number_of_unknowns, 1);
		double rms = 0.0;
		int sum = 0;
		double md = 0.0;
		double md_sum = 0.0;

		for (int bi = 0; bi < buckets_external.size(); bi++)
		{
			if (compute_only_mahalanobis_distance)
			{
				std::cout << "bucket [" << bi + 1 << "] of " << buckets_external.size() << std::endl;
			}
			else
			{
				std::cout << "bucket [" << bi + 1 << "] of " << buckets_external.size() << " | iteration [" << iter + 1 << "] of " << number_of_iterations << " | number of points: " << buckets_external[bi].number_of_points << std::endl;
			}
			std::vector<Point3D> points_global;

			for (size_t index = buckets_external[bi].index_begin; index < buckets_external[bi].index_end; index++)
			{
				points_global.push_back(points_global_external[index_pair_external[index].index_of_point]);
			}

			GridParameters rgd_params;
			rgd_params.resolution_X = this->bucket_size[0];
			rgd_params.resolution_Y = this->bucket_size[1];
			rgd_params.resolution_Z = this->bucket_size[2];
			rgd_params.bounding_box_extension = 1.0;

			std::vector<PointBucketIndexPair> index_pair;
			std::vector<Bucket> buckets;

			std::cout << "building rgd begin" << std::endl;
			grid_calculate_params(points_global, rgd_params);
			build_rgd(points_global, index_pair, buckets, rgd_params, this->number_of_threads);
			std::cout << "building rgd end" << std::endl;

			std::vector<Job> jobs = get_jobs(buckets.size(), this->number_of_threads);

			std::vector<std::thread> threads;

			std::vector<Eigen::SparseMatrix<double>> AtPAtmp(jobs.size());
			std::vector<Eigen::SparseMatrix<double>> AtPBtmp(jobs.size());
			std::vector<double> sumrmss(jobs.size());
			std::vector<int> sums(jobs.size());

			std::vector<double> md_out(jobs.size());
			std::vector<double> md_count_out(jobs.size());
			// double *md_out, double *md_count_out

			for (size_t i = 0; i < jobs.size(); i++)
			{
				AtPAtmp[i] = Eigen::SparseMatrix<double>(point_clouds.size() * number_of_unknowns, point_clouds.size() * number_of_unknowns);
				AtPBtmp[i] = Eigen::SparseMatrix<double>(point_clouds.size() * number_of_unknowns, 1);
				sumrmss[i] = 0;
				sums[i] = 0;
				md_out[i] = 0.0;
				md_count_out[i] = 0.0;
			}

			std::vector<Eigen::Affine3d> mposes;
			std::vector<Eigen::Affine3d> mposes_inv;
			for (size_t i = 0; i < point_clouds.size(); i++)
			{
				mposes.push_back(point_clouds[i].m_pose);
				mposes_inv.push_back(point_clouds[i].m_pose.inverse());
			}

			std::cout << "computing AtPA AtPB start" << std::endl;
			for (size_t k = 0; k < jobs.size(); k++)
			{
				threads.push_back(std::thread(ndt_job, k, &jobs[k], &buckets, &(AtPAtmp[k]), &(AtPBtmp[k]),
											  &index_pair, &points_global, &mposes, &mposes_inv, point_clouds.size(), pose_convention, rotation_matrix_parametrization,
											  number_of_unknowns, &(sumrmss[k]), &(sums[k]),
											  is_generalized, sigma_r, sigma_polar_angle, sigma_azimuthal_angle, num_extended_points, &(md_out[k]), &(md_count_out[k]), false));
			}

			for (size_t j = 0; j < threads.size(); j++)
			{
				threads[j].join();
			}
			std::cout << "computing AtPA AtPB finished" << std::endl;

			for (size_t k = 0; k < jobs.size(); k++)
			{
				rms += sumrmss[k];
				sum += sums[k];
				md += md_out[k];
				md_sum += md_count_out[k];
			}

			for (size_t k = 0; k < jobs.size(); k++)
			{
				if (!init)
				{
					if (AtPBtmp[k].size() > 0)
					{
						AtPA_ndt = AtPAtmp[k];
						AtPB_ndt = AtPBtmp[k];
						init = true;
					}
				}
				else
				{
					if (AtPBtmp[k].size() > 0)
					{

						AtPA_ndt += AtPAtmp[k];
						AtPB_ndt += AtPBtmp[k];
					}
				}
			}
		}
		std::cout << "cleaning start" << std::endl;
		points_global_external.clear();
		index_pair_external.clear();
		buckets_external.clear();
		std::cout << "cleaning finished" << std::endl;

		rms /= sum;
		std::cout << "rms " << rms << std::endl;

		md /= md_sum;
		std::cout << "mean mahalanobis distance: " << md << std::endl;

		if (compute_only_mahalanobis_distance)
		{
			return true;
		}
#else

		GridParameters rgd_params;
		rgd_params.resolution_X = this->bucket_size[0];
		rgd_params.resolution_Y = this->bucket_size[1];
		rgd_params.resolution_Z = this->bucket_size[2];
		rgd_params.bounding_box_extension = 1.0;

		std::vector<Point3D> points_global;
		for (size_t i = 0; i < point_clouds.size(); i++)
		{
			for (size_t j = 0; j < point_clouds[i].points_local.size(); j++)
			{
				Eigen::Vector3d vt = point_clouds[i].m_pose * point_clouds[i].points_local[j];
				Point3D p;
				p.x = vt.x();
				p.y = vt.y();
				p.z = vt.z();
				p.index_pose = i;
				if (p.x > -250 && p.x < 250 && p.y > -250 && p.y < 250 && p.z > -50 && p.z < 50)
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

		for (size_t i = 0; i < jobs.size(); i++)
		{
			AtPAtmp[i] = Eigen::SparseMatrix<double>(point_clouds.size() * number_of_unknowns, point_clouds.size() * number_of_unknowns);
			AtPBtmp[i] = Eigen::SparseMatrix<double>(point_clouds.size() * number_of_unknowns, 1);
			sumrmss[i] = 0;
			sums[i] = 0;
		}

		std::vector<Eigen::Affine3d> mposes;
		std::vector<Eigen::Affine3d> mposes_inv;
		for (size_t i = 0; i < point_clouds.size(); i++)
		{
			mposes.push_back(point_clouds[i].m_pose);
			mposes_inv.push_back(point_clouds[i].m_pose.inverse());
		}

		for (size_t k = 0; k < jobs.size(); k++)
		{
			threads.push_back(std::thread(ndt_job, k, &jobs[k], &buckets, &(AtPAtmp[k]), &(AtPBtmp[k]),
										  &index_pair, &points_global, &mposes, &mposes_inv, point_clouds.size(), pose_convention, rotation_matrix_parametrization, number_of_unknowns, &(sumrmss[k]), &(sums[k]),
										  is_generalized, sigma_r, sigma_polar_angle, sigma_azimuthal_angle, num_extended_points));
		}

		for (size_t j = 0; j < threads.size(); j++)
		{
			threads[j].join();
		}

		double rms = 0.0;
		int sum = 0;
		for (size_t k = 0; k < jobs.size(); k++)
		{
			rms += sumrmss[k];
			sum += sums[k];
		}
		rms /= sum;
		std::cout << "rms " << rms << std::endl;

		bool init = false;
		Eigen::SparseMatrix<double> AtPA_ndt(point_clouds.size() * number_of_unknowns, point_clouds.size() * number_of_unknowns);
		Eigen::SparseMatrix<double> AtPB_ndt(point_clouds.size() * number_of_unknowns, 1);

		for (size_t k = 0; k < jobs.size(); k++)
		{
			if (!init)
			{
				if (AtPBtmp[k].size() > 0)
				{
					AtPA_ndt = AtPAtmp[k];
					AtPB_ndt = AtPBtmp[k];
					init = true;
				}
			}
			else
			{
				if (AtPBtmp[k].size() > 0)
				{

					AtPA_ndt += AtPAtmp[k];
					AtPB_ndt += AtPBtmp[k];
				}
			}
		}
#endif
		//////////////////////////////////////////////////////////////////

		if (is_fix_first_node)
		{
			Eigen::SparseMatrix<double> I(point_clouds.size() * number_of_unknowns, point_clouds.size() * number_of_unknowns);
			for (int ii = 0; ii < number_of_unknowns; ii++)
			{
				I.coeffRef(ii, ii) = 1000000;
			}
			AtPA_ndt += I;
		}

		std::cout << "previous_rms: " << previous_rms << " rms: " << rms << std::endl;
		if (is_levenberg_marguardt)
		{

			if (rms < previous_rms)
			{
				if (lm_lambda < 1000000)
				{
					lm_lambda *= 10.0;
				}
				previous_rms = rms;
				std::cout << " lm_lambda: " << lm_lambda << std::endl;
			}
			else
			{
				lm_lambda /= 10.0;
				number_of_lm_iterations++;
				iter--;
				std::cout << " lm_lambda: " << lm_lambda << std::endl;
				for (size_t i = 0; i < point_clouds.size(); i++)
				{
					point_clouds[i].m_pose = m_poses_tmp[i];
				}
				previous_rms = std::numeric_limits<double>::max();
				continue;
			}
		}
		else
		{
			previous_rms = rms;
		}

		if (is_quaternion)
		{
			std::vector<Eigen::Triplet<double>> tripletListA;
			std::vector<Eigen::Triplet<double>> tripletListP;
			std::vector<Eigen::Triplet<double>> tripletListB;
			for (size_t i = 0; i < point_clouds.size(); i++)
			{
				int ic = i * 7;
				int ir = 0;
				QuaternionPose pose;
				if (is_wc)
				{
					pose = pose_quaternion_from_affine_matrix(point_clouds[i].m_pose);
				}
				else
				{
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
			Eigen::SparseMatrix<double> matA(tripletListB.size(), point_clouds.size() * 7);
			Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
			Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

			matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
			matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
			matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

			Eigen::SparseMatrix<double> AtPA(point_clouds.size() * 7, point_clouds.size() * 7);
			Eigen::SparseMatrix<double> AtPB(point_clouds.size() * 7, 1);

			Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
			AtPA = AtP * matA;
			AtPB = AtP * matB;

			AtPA_ndt += AtPA;
			AtPB_ndt += AtPB;
		}

		if (is_levenberg_marguardt)
		{
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
		for (int k = 0; k < x.outerSize(); ++k)
		{
			for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
			{
				if (it.value() == it.value())
				{
					h_x.push_back(it.value());
					std::cout << it.row() << "," << it.col() << "," << it.value() << std::endl;
				}
			}
		}

		if (h_x.size() == point_clouds.size() * number_of_unknowns)
		{
			std::cout << "AtPA=AtPB SOLVED" << std::endl;
			int counter = 0;
			for (size_t i = 0; i < point_clouds.size(); i++)
			{
				Eigen::Affine3d m_pose;

				if (is_wc)
				{
					m_pose = point_clouds[i].m_pose;
				}
				else
				{
					m_pose = point_clouds[i].m_pose.inverse();
				}

				if (is_tait_bryan_angles)
				{
					TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(m_pose);
					pose.px += h_x[counter++];
					pose.py += h_x[counter++];
					pose.pz += h_x[counter++];
					pose.om += h_x[counter++];
					pose.fi += h_x[counter++];
					pose.ka += h_x[counter++];
					m_pose = affine_matrix_from_pose_tait_bryan(pose);
				}
				else if (is_rodrigues)
				{
					RodriguesPose pose = pose_rodrigues_from_affine_matrix(m_pose);
					pose.px += h_x[counter++];
					pose.py += h_x[counter++];
					pose.pz += h_x[counter++];
					pose.sx += h_x[counter++];
					pose.sy += h_x[counter++];
					pose.sz += h_x[counter++];
					m_pose = affine_matrix_from_pose_rodrigues(pose);
				}
				else if (is_quaternion)
				{
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
						fabs(poseq.q0) < 10 && fabs(poseq.q1) < 10 && fabs(poseq.q2) < 10 && fabs(poseq.q3) < 10)
					{
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
				else if (is_lie_algebra_left_jacobian)
				{
					RodriguesPose pose_update;
					pose_update.px = h_x[counter++];
					pose_update.py = h_x[counter++];
					pose_update.pz = h_x[counter++];
					pose_update.sx = h_x[counter++];
					pose_update.sy = h_x[counter++];
					pose_update.sz = h_x[counter++];
					m_pose = affine_matrix_from_pose_rodrigues(pose_update) * m_pose;
				}
				else if (is_lie_algebra_right_jacobian)
				{
					RodriguesPose pose_update;
					pose_update.px = h_x[counter++];
					pose_update.py = h_x[counter++];
					pose_update.pz = h_x[counter++];
					pose_update.sx = h_x[counter++];
					pose_update.sy = h_x[counter++];
					pose_update.sz = h_x[counter++];
					m_pose = m_pose * affine_matrix_from_pose_rodrigues(pose_update);
				}

				if (is_wc)
				{
					point_clouds[i].m_pose = m_pose;
				}
				else
				{
					point_clouds[i].m_pose = m_pose.inverse();
				}
				if (!point_clouds[i].fixed)
				{
					point_clouds[i].pose = pose_tait_bryan_from_affine_matrix(point_clouds[i].m_pose);
					point_clouds[i].gui_translation[0] = point_clouds[i].pose.px;
					point_clouds[i].gui_translation[1] = point_clouds[i].pose.py;
					point_clouds[i].gui_translation[2] = point_clouds[i].pose.pz;
					point_clouds[i].gui_rotation[0] = rad2deg(point_clouds[i].pose.om);
					point_clouds[i].gui_rotation[1] = rad2deg(point_clouds[i].pose.fi);
					point_clouds[i].gui_rotation[2] = rad2deg(point_clouds[i].pose.ka);
				}
			}
			if (is_levenberg_marguardt)
			{
				m_poses_tmp.clear();
				for (size_t i = 0; i < point_clouds.size(); i++)
				{
					m_poses_tmp.push_back(point_clouds[i].m_pose);
				}
			}

			std::cout << "iteration: " << iter + 1 << " of " << number_of_iterations << std::endl;
		}
		else
		{
			std::cout << "AtPA=AtPB FAILED" << std::endl;
			break;
		}
	}

	auto end = std::chrono::system_clock::now();
	auto elapsed =
		std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

	std::cout << "ndt execution time [ms]: " << elapsed.count() << std::endl;

	return true;
}

std::vector<Eigen::SparseMatrix<double>> NDT::compute_covariance_matrices_and_rms(std::vector<PointCloud> &point_clouds, double &rms)
{
	OptimizationAlgorithm optimization_algorithm;
	if (is_gauss_newton)
	{
		optimization_algorithm = OptimizationAlgorithm::gauss_newton;
	}
	if (is_levenberg_marguardt)
	{
		optimization_algorithm = OptimizationAlgorithm::levenberg_marguardt;
	}

	PoseConvention pose_convention;
	if (is_wc)
	{
		pose_convention = PoseConvention::wc;
	}
	if (is_cw)
	{
		pose_convention = PoseConvention::cw;
	}

	RotationMatrixParametrization rotation_matrix_parametrization;
	if (is_tait_bryan_angles)
	{
		rotation_matrix_parametrization = RotationMatrixParametrization::tait_bryan_xyz;
	}
	if (is_rodrigues)
	{
		rotation_matrix_parametrization = RotationMatrixParametrization::rodrigues;
	}
	if (is_quaternion)
	{
		rotation_matrix_parametrization = RotationMatrixParametrization::quaternion;
	}

	if (is_rodrigues || is_quaternion)
	{
		for (size_t i = 0; i < point_clouds.size(); i++)
		{
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
	if (is_tait_bryan_angles || is_rodrigues)
	{
		number_of_unknowns = 6;
	}
	if (is_quaternion)
	{
		number_of_unknowns = 7;
	}

	std::vector<Eigen::SparseMatrix<double>> covariance_matrices;

	for (size_t i = 0; i < point_clouds.size(); i++)
	{
		Eigen::SparseMatrix<double> covariance_matrix(number_of_unknowns, number_of_unknowns);
		covariance_matrices.push_back(covariance_matrix);
	}

	GridParameters rgd_params;
	rgd_params.resolution_X = this->bucket_size[0];
	rgd_params.resolution_Y = this->bucket_size[1];
	rgd_params.resolution_Z = this->bucket_size[2];
	rgd_params.bounding_box_extension = 1.0;

	std::vector<Point3D> points_global;
	for (size_t i = 0; i < point_clouds.size(); i++)
	{
		for (size_t j = 0; j < point_clouds[i].points_local.size(); j++)
		{
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
	std::vector<double> md_out(jobs.size());
	std::vector<double> md_count_out(jobs.size());

	for (size_t i = 0; i < jobs.size(); i++)
	{
		AtPAtmp[i] = Eigen::SparseMatrix<double>(point_clouds.size() * number_of_unknowns, point_clouds.size() * number_of_unknowns);
		AtPBtmp[i] = Eigen::SparseMatrix<double>(point_clouds.size() * number_of_unknowns, 1);
		sumrmss[i] = 0;
		sums[i] = 0;
		md_out[i] = 0.0;
		md_count_out[i] = 0.0;
	}

	std::vector<Eigen::Affine3d> mposes;
	std::vector<Eigen::Affine3d> mposes_inv;
	for (size_t i = 0; i < point_clouds.size(); i++)
	{
		// poses.push_back(pose_tait_bryan_from_affine_matrix(point_clouds[i].m_pose));
		mposes.push_back(point_clouds[i].m_pose);
		mposes_inv.push_back(point_clouds[i].m_pose.inverse());
	}
	// std::cout << "XXX" << std::endl;
	// std::cout << (int)pose_convention << " " << (int)rotation_matrix_parametrization << " " << (int)number_of_unknowns << std::endl;

	for (size_t k = 0; k < jobs.size(); k++)
	{
		threads.push_back(std::thread(ndt_job, k, &jobs[k], &buckets, &(AtPAtmp[k]), &(AtPBtmp[k]),
									  &index_pair, &points_global, &mposes, &mposes_inv, point_clouds.size(), pose_convention, rotation_matrix_parametrization,
									  number_of_unknowns, &(sumrmss[k]), &(sums[k]),
									  is_generalized, sigma_r, sigma_polar_angle, sigma_azimuthal_angle, num_extended_points, &(md_out[k]), &(md_count_out[k]), false));
	}

	for (size_t j = 0; j < threads.size(); j++)
	{
		threads[j].join();
	}
	double ssr = 0.0;
	int num_obs = 0;
	for (size_t k = 0; k < jobs.size(); k++)
	{
		ssr += sumrmss[k];
		num_obs += sums[k];
	}

	double sq = ssr / ((double)num_obs - point_clouds.size() * number_of_unknowns);
	rms = ssr / (double)num_obs;

	// std::cout << "sq " << sq << " num_obs " << num_obs << std::endl;

	bool init = false;
	Eigen::SparseMatrix<double> AtPA_ndt(point_clouds.size() * number_of_unknowns, point_clouds.size() * number_of_unknowns);

	for (size_t k = 0; k < jobs.size(); k++)
	{
		if (!init)
		{
			if (AtPBtmp[k].size() > 0)
			{
				//
				AtPA_ndt = AtPAtmp[k];
				init = true;
			}
		}
		else
		{
			if (AtPBtmp[k].size() > 0)
			{
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

	for (int i = 0; i < point_clouds.size(); i++)
	{
		Eigen::SparseMatrix<double> cm(number_of_unknowns, number_of_unknowns);
		for (int r = 0; r < number_of_unknowns; r++)
		{
			for (int c = 0; c < number_of_unknowns; c++)
			{
				cm.coeffRef(r, c) = AtAinv.coeff(i * number_of_unknowns + r, i * number_of_unknowns + c);
			}
		}
		covariance_matrices.push_back(cm);
		// std::cout << "cm" << std::endl;
		// std::cout << cm << std::endl;
	}
	return covariance_matrices;
}

bool NDT::optimize(std::vector<PointCloud> &point_clouds, double &rms_initial, double &rms_final, double &mui)
{
	// double rms;
	// std::vector<Eigen::SparseMatrix<double>> cm_before = compute_covariance_matrices_and_rms(point_clouds, rms);
	// rms_initial = rms;
	//--
	bool res = optimize(point_clouds, false);
	//--
	// std::vector<Eigen::SparseMatrix<double>> cm_after = compute_covariance_matrices_and_rms(point_clouds, rms);
	// rms_final = rms;
	// mui = get_mean_uncertainty_xyz_impact(cm_before, cm_after);
	return res;
}

bool NDT::optimize_lie_algebra_left_jacobian(std::vector<PointCloud> &point_clouds)
{
	is_tait_bryan_angles = false;
	is_quaternion = false;
	is_rodrigues = false;
	is_lie_algebra_left_jacobian = true;
	is_lie_algebra_right_jacobian = false;

	bool res = optimize(point_clouds, false);

	is_tait_bryan_angles = true;
	is_quaternion = false;
	is_rodrigues = false;
	is_lie_algebra_left_jacobian = false;
	is_lie_algebra_right_jacobian = false;
	return true;
}

bool NDT::optimize_lie_algebra_right_jacobian(std::vector<PointCloud> &point_clouds)
{
	is_tait_bryan_angles = false;
	is_quaternion = false;
	is_rodrigues = false;
	is_lie_algebra_left_jacobian = false;
	is_lie_algebra_right_jacobian = true;

	bool res = optimize(point_clouds, false);

	is_tait_bryan_angles = true;
	is_quaternion = false;
	is_rodrigues = false;
	is_lie_algebra_left_jacobian = false;
	is_lie_algebra_right_jacobian = false;
	return true;
}

bool NDT::compute_cov_mean(std::vector<Point3D> &points, std::vector<PointBucketIndexPair> &index_pair, std::vector<Bucket> &buckets, GridParameters &rgd_params,
						   double min_x, double max_x, double min_y, double max_y, double min_z, double max_z, int num_threads)
{
	int number_of_unknowns = 6;

	grid_calculate_params(rgd_params, min_x, max_x, min_y, max_y, min_z, max_z);
	build_rgd(points, index_pair, buckets, rgd_params);
	std::cout << "buckets.size() " << buckets.size() << std::endl;

	std::vector<Job> jobs = get_jobs(buckets.size(), this->number_of_threads);

	std::vector<std::thread> threads;

	std::vector<Eigen::SparseMatrix<double>> AtPAtmp(jobs.size());
	std::vector<Eigen::SparseMatrix<double>> AtPBtmp(jobs.size());
	std::vector<double> sumrmss(jobs.size());
	std::vector<int> sums(jobs.size());

	std::vector<double> md_out(jobs.size());
	std::vector<double> md_count_out(jobs.size());
	// double *md_out, double *md_count_out

	for (size_t i = 0; i < jobs.size(); i++)
	{
		AtPAtmp[i] = Eigen::SparseMatrix<double>(points.size() * number_of_unknowns, points.size() * number_of_unknowns);
		AtPBtmp[i] = Eigen::SparseMatrix<double>(points.size() * number_of_unknowns, 1);
		sumrmss[i] = 0;
		sums[i] = 0;
		md_out[i] = 0.0;
		md_count_out[i] = 0.0;
	}

	std::vector<Eigen::Affine3d> mposes;
	std::vector<Eigen::Affine3d> mposes_inv;
	mposes.push_back(Eigen::Affine3d::Identity());
	mposes_inv.push_back(Eigen::Affine3d::Identity());

	// for (size_t i = 0; i < points.size(); i++)
	//{
	//	mposes.push_back(points[i].m_pose);
	//	mposes_inv.push_back(points[i].m_pose.inverse());
	// }

	std::cout << "computing cov mean start" << std::endl;
	for (size_t k = 0; k < jobs.size(); k++)
	{
		threads.push_back(std::thread(ndt_job, k, &jobs[k], &buckets, &(AtPAtmp[k]), &(AtPBtmp[k]),
									  &index_pair, &points, &mposes, &mposes_inv, points.size(),
									  PoseConvention::wc, RotationMatrixParametrization::tait_bryan_xyz,
									  number_of_unknowns, &(sumrmss[k]), &(sums[k]),
									  is_generalized, sigma_r, sigma_polar_angle, sigma_azimuthal_angle,
									  num_extended_points, &(md_out[k]), &(md_count_out[k]), true));
	}

	for (size_t j = 0; j < threads.size(); j++)
	{
		threads[j].join();
	}
	std::cout << "computing cov mean finished" << std::endl;

	// for(int i = 0; i < buckets.size(); i++){
	//	if(buckets[i].number_of_points > 5){
	//		std::cout << i << " " << buckets[i].cov << std::endl;
	//	}
	// }
	buckets[0].number_of_points = 0;
	for (auto &b : buckets)
	{
		if (b.number_of_points < 5)
		{
			b.number_of_points = 0;
		}
	}
	return true;
}

bool NDT::compute_cov_mean(std::vector<Point3Di> &points,
						   std::vector<PointBucketIndexPair> &index_pair,
						   std::vector<Bucket> &buckets, GridParameters &rgd_params,
						   int num_threads)
{
	int number_of_unknowns = 6;

	std::cout << "grid_calculate_params start" << std::endl;
	grid_calculate_params(points, rgd_params);
	// for(auto &p:points){
	//	std::cout << p.point;
	// }
	std::cout << "grid_calculate_params finished" << std::endl;

	// grid_calculate_params(rgd_params, min_x, max_x, min_y, max_y, min_z, max_z);
	std::cout << "build_rgd start" << std::endl;
	build_rgd(points, index_pair, buckets, rgd_params);
	std::cout << "build_rgd finished" << std::endl;

	std::cout << "check" << std::endl;
	int counter_active_buckets = 0;
	for (int i = 0; i < buckets.size(); i++)
	{
		if (buckets[i].number_of_points > 5)
		{
			counter_active_buckets++;
		}
	}
	std::cout << "check: counter_active_buckets: " << counter_active_buckets << std::endl;

	std::cout << "buckets.size() " << buckets.size() << std::endl;

	std::vector<Job> jobs = get_jobs(buckets.size(), this->number_of_threads);

	std::vector<std::thread> threads;

	std::vector<Eigen::SparseMatrix<double>> AtPAtmp(jobs.size());
	std::vector<Eigen::SparseMatrix<double>> AtPBtmp(jobs.size());
	std::vector<double> sumrmss(jobs.size());
	std::vector<int> sums(jobs.size());

	std::vector<double> md_out(jobs.size());
	std::vector<double> md_count_out(jobs.size());
	// double *md_out, double *md_count_out

	for (size_t i = 0; i < jobs.size(); i++)
	{
		AtPAtmp[i] = Eigen::SparseMatrix<double>(points.size() * number_of_unknowns, points.size() * number_of_unknowns);
		AtPBtmp[i] = Eigen::SparseMatrix<double>(points.size() * number_of_unknowns, 1);
		sumrmss[i] = 0;
		sums[i] = 0;
		md_out[i] = 0.0;
		md_count_out[i] = 0.0;
	}

	std::vector<Eigen::Affine3d> mposes;
	std::vector<Eigen::Affine3d> mposes_inv;
	mposes.push_back(Eigen::Affine3d::Identity());
	mposes_inv.push_back(Eigen::Affine3d::Identity());

	// for (size_t i = 0; i < points.size(); i++)
	//{
	//	mposes.push_back(points[i].m_pose);
	//	mposes_inv.push_back(points[i].m_pose.inverse());
	// }

	std::cout << "computing cov mean ndt_jobi start" << std::endl;
	for (size_t k = 0; k < jobs.size(); k++)
	{
		threads.push_back(std::thread(ndt_jobi, k, &jobs[k], &buckets, &(AtPAtmp[k]), &(AtPBtmp[k]),
									  &index_pair, &points, &mposes, &mposes_inv, points.size(),
									  PoseConvention::wc, RotationMatrixParametrization::tait_bryan_xyz,
									  number_of_unknowns, &(sumrmss[k]), &(sums[k]),
									  is_generalized, sigma_r, sigma_polar_angle, sigma_azimuthal_angle,
									  num_extended_points, &(md_out[k]), &(md_count_out[k]), true));
	}

	for (size_t j = 0; j < threads.size(); j++)
	{
		threads[j].join();
	}
	std::cout << "computing cov mean ndt_jobi finished" << std::endl;

	// for(int i = 0; i < buckets.size(); i++){
	//	if(buckets[i].number_of_points > 5){
	//		std::cout << i << " " << buckets[i].cov << std::endl;
	//	}
	// }
	buckets[0].number_of_points = 0;

	// int counter_active_buckets = 0;
	for (auto &b : buckets)
	{
		if (b.number_of_points < 5)
		{
			b.number_of_points = 0;
		}
		else
		{
			// counter_active_buckets ++;
		}
	}
	// std::cout << "counter_active_buckets: " << counter_active_buckets << std::endl;
	return true;
}

bool NDT::compute_cov_mean(std::vector<Point3Di> &points,
						   std::vector<PointBucketIndexPair> &index_pair,
						   std::map<unsigned long long int, NDT::Bucket> &buckets, GridParameters &rgd_params,
						   int num_threads)
{
#if 0
	int number_of_unknowns = 6;

	std::cout << "grid_calculate_params start" << std::endl;	
	grid_calculate_params(points, rgd_params);
	//for(auto &p:points){
	//	std::cout << p.point;
	//}
	std::cout << "grid_calculate_params finished" << std::endl;	

	//grid_calculate_params(rgd_params, min_x, max_x, min_y, max_y, min_z, max_z);
	std::cout << "build_rgd start" << std::endl;	
	build_rgd(points, index_pair, buckets, rgd_params);
	std::cout << "build_rgd finished" << std::endl;

	std::cout << "check" << std::endl;
	int counter_active_buckets = 0; 
	//for(int i = 0; i < buckets.size(); i++){
	//	if(buckets[i].number_of_points > 5){
	//		counter_active_buckets ++;
	//	}
	//}
	for (const auto& [key, value] : buckets){
		if(value.number_of_points > 5){
			counter_active_buckets ++;
		}
	}

	std::cout << "check: counter_active_buckets: " << counter_active_buckets << std::endl;


	std::cout << "buckets.size() " << buckets.size() << std::endl;

	std::vector<Job> jobs = get_jobs(buckets.size(), this->number_of_threads);

	std::vector<std::thread> threads;

	std::vector<Eigen::SparseMatrix<double>> AtPAtmp(jobs.size());
	std::vector<Eigen::SparseMatrix<double>> AtPBtmp(jobs.size());
	std::vector<double> sumrmss(jobs.size());
	std::vector<int> sums(jobs.size());

	std::vector<double> md_out(jobs.size());
	std::vector<double> md_count_out(jobs.size());
	// double *md_out, double *md_count_out

	for (size_t i = 0; i < jobs.size(); i++)
	{
		AtPAtmp[i] = Eigen::SparseMatrix<double>(points.size() * number_of_unknowns, points.size() * number_of_unknowns);
		AtPBtmp[i] = Eigen::SparseMatrix<double>(points.size() * number_of_unknowns, 1);
		sumrmss[i] = 0;
		sums[i] = 0;
		md_out[i] = 0.0;
		md_count_out[i] = 0.0;
	}

	std::vector<Eigen::Affine3d> mposes;
	std::vector<Eigen::Affine3d> mposes_inv;
	mposes.push_back(Eigen::Affine3d::Identity());
	mposes_inv.push_back(Eigen::Affine3d::Identity());

	//for (size_t i = 0; i < points.size(); i++)
	//{
	//	mposes.push_back(points[i].m_pose);
	//	mposes_inv.push_back(points[i].m_pose.inverse());
	//}

	std::cout << "computing cov mean ndt_jobi start" << std::endl;
	for (size_t k = 0; k < jobs.size(); k++)
	{
		threads.push_back(std::thread(ndt_jobi, k, &jobs[k], &buckets, &(AtPAtmp[k]), &(AtPBtmp[k]),
										&index_pair, &points, &mposes, &mposes_inv, points.size(), 
										PoseConvention::wc, RotationMatrixParametrization::tait_bryan_xyz,
										number_of_unknowns, &(sumrmss[k]), &(sums[k]),
										is_generalized, sigma_r, sigma_polar_angle, sigma_azimuthal_angle, 
										num_extended_points, &(md_out[k]), &(md_count_out[k]), true));
	}

	for (size_t j = 0; j < threads.size(); j++)
	{
		threads[j].join();
	}
	std::cout << "computing cov mean ndt_jobi finished" << std::endl;

	//for(int i = 0; i < buckets.size(); i++){
	//	if(buckets[i].number_of_points > 5){
	//		std::cout << i << " " << buckets[i].cov << std::endl;
	//	}
	//}
	buckets[0].number_of_points = 0;

	//int counter_active_buckets = 0;
	/*for(auto &b:buckets){
		if(b.number_of_points < 5){
			b.number_of_points = 0;
		}else{
			//counter_active_buckets ++;
		}
	}*/

	for (auto& [key, value] : buckets){
		if(value.number_of_points < 5){
			value.number_of_points = 0;
		}
	}

	//std::cout << "counter_active_buckets: " << counter_active_buckets << std::endl;
#endif
	return true;
}

void NDT::build_rgd(std::vector<Point3Di> &points, std::vector<NDT::PointBucketIndexPair> &index_pair, std::map<unsigned long long int, NDT::Bucket> &buckets, NDT::GridParameters &rgd_params, int num_threads)
{
	if (num_threads < 1)
		num_threads = 1;

	index_pair.resize(points.size());
	std::cout << "reindex start" << std::endl;
	reindex(points, index_pair, rgd_params, num_threads);
	std::cout << "reindex finished" << std::endl;

	// buckets.resize(rgd_params.number_of_buckets);

	// std::vector<NDT::Job> jobs = get_jobs(buckets.size(), num_threads);
	// std::vector<std::thread> threads;

	// std::cout << "build_rgd_init_jobs start" << std::endl;
	// for (size_t i = 0; i < jobs.size(); i++)
	//{
	//	threads.push_back(std::thread(build_rgd_init_job, i, &jobs[i], &buckets));
	// }

	// for (size_t j = 0; j < threads.size(); j++)
	//{
	//	threads[j].join();
	// }
	// threads.clear();
	// std::cout << "build_rgd_init_jobs finished" << std::endl;

	// jobs = get_jobs(points.size(), num_threads);

	// std::cout << "build_rgd_jobs start jobs.size():" << jobs.size() << std::endl;
	// std::cout << "points.size() " << points.size() << std::endl;
	// std::cout << "index_pair.size() " << index_pair.size() << std::endl;
	// std::cout << "buckets.size() " << buckets.size() << std::endl;

	// for (size_t i = 0; i < jobs.size(); i++)
	//{
	//	threads.push_back(std::thread(build_rgd_job, i, &jobs[i], &index_pair, &buckets));
	// }
	// for (size_t j = 0; j < threads.size(); j++)
	//{
	//	threads[j].join();
	// }
	// threads.clear();
	// std::cout << "build_rgd_jobs finished" << std::endl;

	// jobs = get_jobs(buckets.size(), num_threads);

	// std::cout << "build_rgd_final_jobs start" << std::endl;
	// for (size_t i = 0; i < jobs.size(); i++)
	//{
	//	threads.push_back(std::thread(build_rgd_final_job, i, &jobs[i], &buckets));
	// }

	// for (size_t j = 0; j < threads.size(); j++)
	//{
	//	threads[j].join();
	// }
	// threads.clear();
	// std::cout << "build_rgd_final_jobs finished" << std::endl;
}
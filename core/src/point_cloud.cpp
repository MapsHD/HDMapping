#include <point_cloud.h>
#include <transformations.h>

#include <GL/freeglut.h>
#include <iostream>
#include <cstdlib>
#include <sstream>
#include <string>
#include <fstream>
#include <vector>
#include <algorithm>
#include <thread>

#include <plycpp.h>

bool PointCloud::load(const std::string& file_name) {
	points_local.clear();
	
	//try
	//{
		std::cout << "Loading PLY data..." << std::endl;
		plycpp::PLYData data;

		plycpp::load(file_name, data);

		// Listing PLY content
		{
			std::cout << "List of elements and properties:\n"
				<< "===========================" << std::endl;
			for (const auto& element : data)
			{
				std::cout << "* " << element.key << " -- size: " << element.data->size() << std::endl;
				for (const auto& prop : element.data->properties)
				{
					std::cout << "    - " << prop.key
						<< " -- type: "
						<< (prop.data->isList ? "list of " : "")
						<< prop.data->type.name()
						<< " -- size: " << prop.data->size() << std::endl;
				}
			}
			std::cout << "\n";
		}

		// Example of direct access
		{
			auto xData = data["vertex"]->properties["x"];
			//std::cout << "x value of the first vertex element:\n" << xData->at<float>(0) << std::endl;
			//std::cout << "\n";
			auto yData = data["vertex"]->properties["y"];
			auto zData = data["vertex"]->properties["z"];

			for (size_t i = 0; i < xData->size(); i++) {
				Eigen::Vector3d point(xData->at<float>(i), yData->at<float>(i), zData->at<float>(i));
				points_local.push_back(point);
			}
		}
	//}
	//catch (const plycpp::Exception& e)
	//{
	//	std::cout << "An exception happened:\n" << e.what() << std::endl;
	//}

#if 0
	std::ifstream infile(file_name);
	if (!infile.good()) {
		return false;
	}

	std::string line;
	std::getline(infile, line);

	while (std::getline(infile, line))
	{
		std::replace(line.begin(), line.end(), ';', ' ');
		std::istringstream iss(line);

		double x, y, z;// , intensity;

		iss >> x >> y >> z;// >> intensity;

		Eigen::Vector3d point(x, y, z);
		points_local.push_back(point);
		//intensities.push_back(intensity);
	}

#endif
	std::cout << "File: '" << file_name << "' loaded" << std::endl;
	return true;
}

void PointCloud::render(bool show_with_initial_pose, const ObservationPicking& observation_picking, int viewer_decmiate_point_cloud) {
	glPointSize(observation_picking.point_size);
	
	if (this->visible) {
		glColor3f(render_color[0], render_color[1], render_color[2]);
		glPointSize(point_size);
		glBegin(GL_POINTS);
		//for (const auto& p : this->points_local) {
		for(int i = 0 ; i < this->points_local.size(); i+= viewer_decmiate_point_cloud){
			const auto& p = this->points_local[i];
			Eigen::Vector3d vp;
			if (show_with_initial_pose) {
				vp = this->m_initial_pose * p;
			}
			else {
				vp = this->m_pose * p;
			}
			if (observation_picking.is_observation_picking_mode) {
				if (fabs(vp.z() - observation_picking.picking_plane_height) <= observation_picking.picking_plane_threshold) {
					glVertex3d(vp.x(), vp.y(), vp.z());
				}
			}
			else {
				glVertex3d(vp.x(), vp.y(), vp.z());
			}
		}
		glEnd();
		glPointSize(1);
	}
}

void PointCloud::update_from_gui()
{
	pose.px = gui_translation[0];
	pose.py = gui_translation[1];
	pose.pz = gui_translation[2];

	pose.om = deg2rad(gui_rotation[0]);
	pose.fi = deg2rad(gui_rotation[1]);
	pose.ka = deg2rad(gui_rotation[2]);

	m_pose = affine_matrix_from_pose_tait_bryan(pose);
}

bool PointCloud::save_as_global(std::string file_name)
{
	std::ofstream outfile;
	outfile.open(file_name);
	if (!outfile.good()) {
		std::cout << "Problem with saving to file: '" << file_name << "'" << std::endl;
		return false;
	}

	for (size_t i = 0; i < this->points_local.size(); i++) {
		Eigen::Vector3d vt = this->m_pose * this->points_local[i];
		outfile << vt.x() << ";" << vt.y() << ";" << vt.z() << std::endl;
	}

	std::cout << "File: '" << file_name << "' saved" << std::endl;
	outfile.close();
	return true;
}

void build_rgd_init_job(int i, PointCloud::Job* job, std::vector < PointCloud::Bucket>* buckets) {
	for (size_t ii = job->index_begin_inclusive; ii < job->index_end_exclusive; ii++) {
		(*buckets)[ii].index_begin = -1;
		(*buckets)[ii].index_end = -1;
		(*buckets)[ii].number_of_points = 0;
	}
}

void build_rgd_job(int i, PointCloud::Job* job, std::vector<PointCloud::PointBucketIndexPair>* index_pair, std::vector < PointCloud::Bucket>* buckets) {
	//std::cout << "build_rgd_job:[" << i << "]" << std::endl;

	for (long long unsigned int ii = job->index_begin_inclusive; ii < job->index_end_exclusive; ii++) {
		long long unsigned int ind = ii;
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

void build_rgd_final_job(int i, PointCloud::Job* job, std::vector < PointCloud::Bucket>* buckets) {
	// std::cout << "build_rgd_init_job:[" << i << "]" << std::endl;

	for (size_t ii = job->index_begin_inclusive; ii < job->index_end_exclusive; ii++) {

		//std::cout << job->index_begin_inclusive << " " << job->index_end_exclusive << " " << (*buckets).size() << std::endl;

		long long unsigned int index_begin = (*buckets)[ii].index_begin;
		long long unsigned int index_end = (*buckets)[ii].index_end;
		if (index_begin != -1 && index_end != -1)
		{
			(*buckets)[ii].number_of_points = index_end - index_begin;
		}
	}
}

bool PointCloud::build_rgd()
{
	//std::cout << "build_rgd()" << std::endl;

	grid_calculate_params(this->points_local, this->rgd_params);
	cout_rgd();

	//std::cout << "grid_calculate_params done" << std::endl;
	
	reindex(this->index_pairs, this->points_local, this->rgd_params);
	//std::cout << "reindex done" << std::endl;

	//std::cout << "rgd_params.number_of_buckets: " << rgd_params.number_of_buckets << std::endl;

	buckets.resize(rgd_params.number_of_buckets);


	std::vector<Job> jobs = get_jobs(buckets.size(), num_threads);

	std::vector<std::thread> threads;

	for (size_t i = 0; i < jobs.size(); i++) {
		threads.push_back(std::thread(build_rgd_init_job, i, &jobs[i], &buckets));
	}

	for (size_t j = 0; j < threads.size(); j++) {
		threads[j].join();
	}
	threads.clear();


	jobs = get_jobs(points_local.size(), num_threads);

	for (size_t i = 0; i < jobs.size(); i++) {
		threads.push_back(std::thread(build_rgd_job, i, &jobs[i], &index_pairs, &buckets));
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

	return true;
}

void PointCloud::grid_calculate_params(std::vector<Eigen::Vector3d> &points, GridParameters& params)
{
	double min_x = std::numeric_limits<double>::max();
	double max_x = std::numeric_limits<double>::min();

	double min_y = std::numeric_limits<double>::max();
	double max_y = std::numeric_limits<double>::min();

	double min_z = std::numeric_limits<double>::max();
	double max_z = std::numeric_limits<double>::min();

	for (size_t i = 0; i < points.size(); i++) {
		if (points[i].x() < min_x) min_x = points[i].x();
		if (points[i].x() > max_x) max_x = points[i].x();

		if (points[i].y() < min_y) min_y = points[i].y();
		if (points[i].y() > max_y) max_y = points[i].y();

		if (points[i].z() < min_z) min_z = points[i].z();
		if (points[i].z() > max_z) max_z = points[i].z();
	}

	min_x -= params.bounding_box_extension;
	max_x += params.bounding_box_extension;
	min_y -= params.bounding_box_extension;
	max_y += params.bounding_box_extension;
	min_z -= params.bounding_box_extension;
	max_z += params.bounding_box_extension;

	long long unsigned int number_of_buckets_X = ((max_x - min_x) / params.resolution_X) + 1;
	long long unsigned int number_of_buckets_Y = ((max_y - min_y) / params.resolution_Y) + 1;
	long long unsigned int number_of_buckets_Z = ((max_z - min_z) / params.resolution_Z) + 1;

	params.number_of_buckets_X = number_of_buckets_X;
	params.number_of_buckets_Y = number_of_buckets_Y;
	params.number_of_buckets_Z = number_of_buckets_Z;
	params.number_of_buckets = static_cast<long long unsigned int>(number_of_buckets_X) *
		static_cast<long long unsigned int>(number_of_buckets_Y) * static_cast<long long unsigned int>(number_of_buckets_Z);

	params.bounding_box_max_X = max_x;
	params.bounding_box_min_X = min_x;
	params.bounding_box_max_Y = max_y;
	params.bounding_box_min_Y = min_y;
	params.bounding_box_max_Z = max_z;
	params.bounding_box_min_Z = min_z;
}

void reindex_job(int i, PointCloud::Job* job, std::vector<Eigen::Vector3d>* points, std::vector<PointCloud::PointBucketIndexPair>* pairs, 
	PointCloud::GridParameters rgd_params) {

	// std::cout << "reindex_job:[" << i << "]" << std::endl;

	for (size_t ii = job->index_begin_inclusive; ii < job->index_end_exclusive; ii++) {


		Eigen::Vector3d& p = (*points)[ii];

		(*pairs)[ii].index_of_point = ii;
		(*pairs)[ii].index_of_bucket = 0;
		//(*pairs)[ii].index_pose = p.index_pose;


		long long unsigned int ix = (p.x() - rgd_params.bounding_box_min_X) / rgd_params.resolution_X;
		long long unsigned int iy = (p.y() - rgd_params.bounding_box_min_Y) / rgd_params.resolution_Y;
		long long unsigned int iz = (p.z() - rgd_params.bounding_box_min_Z) / rgd_params.resolution_Z;

		(*pairs)[ii].index_of_bucket = ix * static_cast<long long unsigned int>(rgd_params.number_of_buckets_Y) *
			static_cast<long long unsigned int>(rgd_params.number_of_buckets_Z) + iy * static_cast<long long unsigned int>(rgd_params.number_of_buckets_Z) + iz;
	}
}

void PointCloud::reindex(std::vector<PointBucketIndexPair>& ip, std::vector<Eigen::Vector3d>& points, GridParameters params)
{
	ip.resize(points.size());

	std::vector<Job> jobs = get_jobs(ip.size(), num_threads);

	std::vector<std::thread> threads;

	for (size_t i = 0; i < jobs.size(); i++) {
		threads.push_back(std::thread(reindex_job, i, &jobs[i], &points, &ip, params));
	}

	for (size_t j = 0; j < threads.size(); j++) {
		threads[j].join();
	}
	threads.clear();

	//std::sort(index_pairs.begin(), index_pairs.end(), [](const PointBucketIndexPair& a, const PointBucketIndexPair& b) { return ((a.index_of_bucket == b.index_of_bucket) ? (a.index_pose < b.index_pose) : (a.index_of_bucket < b.index_of_bucket)); });
	std::sort(ip.begin(), ip.end(), [](const PointBucketIndexPair& a, const PointBucketIndexPair& b) { return a.index_of_bucket < b.index_of_bucket; });
}

std::vector<PointCloud::Job> PointCloud::get_jobs(long long unsigned int size, int num_threads) {

	int hc = size / num_threads;
	if (hc < 1)hc = 1;

	std::vector<PointCloud::Job> jobs;
	for (long long unsigned int i = 0; i < size; i += hc) {
		long long unsigned int sequence_length = hc;
		if (i + hc >= size) {
			sequence_length = size - i;
		}
		if (sequence_length == 0)break;

		PointCloud::Job j;
		j.index_begin_inclusive = i;
		j.index_end_exclusive = i + sequence_length;
		jobs.push_back(j);
	}

	//std::cout << jobs.size() << " jobs; chunks: ";
	//for(size_t i = 0; i < jobs.size(); i++){
	//	std::cout << "("<<jobs[i].index_begin_inclusive << ", " << jobs[i].index_end_exclusive <<") ";
	//}
	//std::cout << "\n";
	return jobs;
}

void PointCloud::cout_rgd()
{
	std::cout << "Number_of_buckets X,Y,Z: " << rgd_params.number_of_buckets_X << ", " << rgd_params.number_of_buckets_Y << ", " << rgd_params.number_of_buckets_Z << std::endl;
	std::cout << "bounding box: x(" << rgd_params.bounding_box_min_X << "," << rgd_params.bounding_box_max_X << "), y(" << rgd_params.bounding_box_min_Y << "," << rgd_params.bounding_box_max_Y << ") , z(" << rgd_params.bounding_box_min_Z << "," << rgd_params.bounding_box_max_Z << ")" << std::endl;
}

void nearest_neighbours_job(
	//int i, 
	PointCloud::Job* job, 
	std::vector<std::pair<int, int>>* nn_segments,
	std::vector<Eigen::Vector3d>* points_local_source, 
	std::vector<Eigen::Vector3d>* points_local_target,
	std::vector <int>* points_type_source,
	std::vector <int>* points_type_target,
	PointCloud::GridParameters rgd_params,
	std::vector<PointCloud::Bucket>* buckets,
	std::vector<PointCloud::PointBucketIndexPair>* index_pairs,
	Eigen::Affine3d m_pose_source,
	Eigen::Affine3d m_pose_target,
	double search_radious){
	
	int threshold_inner = 100;
	int threshold_outer = 100;

	Eigen::Affine3d m_pose_target_inv = m_pose_target.inverse();
	for (size_t ii = job->index_begin_inclusive; ii < job->index_end_exclusive; ii++) {
		Eigen::Vector3d point_source_local = (*points_local_source)[ii];
		Eigen::Vector3d p = m_pose_target_inv * (m_pose_source * point_source_local);
		int point_source_type = (*points_type_source)[ii];

		if (p.x() < rgd_params.bounding_box_min_X || p.x() > rgd_params.bounding_box_max_X)continue;
		if (p.y() < rgd_params.bounding_box_min_Y || p.y() > rgd_params.bounding_box_max_Y)continue;
		if (p.z() < rgd_params.bounding_box_min_Z || p.z() > rgd_params.bounding_box_max_Z)continue;

		long long unsigned int ix = (p.x() - rgd_params.bounding_box_min_X) / rgd_params.resolution_X;
		long long unsigned int iy = (p.y() - rgd_params.bounding_box_min_Y) / rgd_params.resolution_Y;
		long long unsigned int iz = (p.z() - rgd_params.bounding_box_min_Z) / rgd_params.resolution_Z;

		long long unsigned int index_bucket = ix * static_cast<long long unsigned int>(rgd_params.number_of_buckets_Y) *
			static_cast<long long unsigned int>(rgd_params.number_of_buckets_Z) + iy * static_cast<long long unsigned int>(rgd_params.number_of_buckets_Z) + iz;

		if (index_bucket >= 0 && index_bucket < rgd_params.number_of_buckets) {
			int sx, sy, sz, stx, sty, stz;
			if (ix == 0)sx = 0; else sx = -1;
			if (iy == 0)sy = 0; else sy = -1;
			if (iz == 0)sz = 0; else sz = -1;

			if (ix == rgd_params.number_of_buckets_X - 1)stx = 1; else stx = 2;
			if (iy == rgd_params.number_of_buckets_Y - 1)sty = 1; else sty = 2;
			if (iz == rgd_params.number_of_buckets_Z - 1)stz = 1; else stz = 2;


			long long unsigned int index_next_bucket;
			int iter;
			int number_of_points_in_bucket;
			int l_begin;
			int l_end;
			float dist = 0.0f;


			///////////////////////////////////////////////////////////////////////////////////////////
			double min_dist = std::numeric_limits<double>::max();
			PointCloud::NearestNeighbour nn;
			nn.found = false;

			//Point pTargetGlobal;
			//Point pTargetGlobalOut;

			for (int i = sx; i < stx; i++) {
				for (int j = sy; j < sty; j++) {
					for (int k = sz; k < stz; k++) {
						index_next_bucket = index_bucket +
							i * rgd_params.number_of_buckets_Y * rgd_params.number_of_buckets_Z +
							j * rgd_params.number_of_buckets_Z + k;

						if (index_next_bucket >= 0 && index_next_bucket < (*buckets).size()) {

							PointCloud::Bucket b = (*buckets)[index_next_bucket];

							number_of_points_in_bucket = b.number_of_points;

							if (number_of_points_in_bucket <= 0)continue;

							int max_number_considered_in_bucket;
							if (index_next_bucket == index_bucket) {
								max_number_considered_in_bucket = threshold_inner;
							}
							else {
								max_number_considered_in_bucket = threshold_outer;
							}
							if (max_number_considered_in_bucket <= 0)continue;

							if (max_number_considered_in_bucket >= number_of_points_in_bucket) {
								iter = 1;
							}
							else {
								iter = number_of_points_in_bucket / max_number_considered_in_bucket;
								if (iter <= 0)iter = 1;
							}

							l_begin = b.index_begin;
							l_end = b.index_end;

							for (int l = l_begin; l < l_end; l += iter) {
								if (l >= 0 && l < (*index_pairs).size()) {
									int hashed_index_of_point = (*index_pairs)[l].index_of_point;

									if (hashed_index_of_point >= 0 && hashed_index_of_point < (*points_local_target).size()) {
										const auto& p_target = (*points_local_target)[hashed_index_of_point];
										int point_target_type = (*points_type_target)[hashed_index_of_point];

										
										double dist = (p_target - p).norm();

										if (dist <= search_radious && (point_source_type == point_target_type)) {
											if (dist < min_dist) {
												min_dist = dist;
												nn.index_source = ii;
												nn.index_target = hashed_index_of_point;
												nn.found = true;
											}
										}
									}
								}
							}
						}
					}
				}
			}
			if (nn.found) {
				(*nn_segments).emplace_back(nn.index_source, nn.index_target);
			}
		}
	}
}

std::vector<std::pair<int, int>> PointCloud::nns(PointCloud& pc_target, double search_radious)
{
	std::vector<std::pair<int, int>> nerest_neighbours;

	std::vector<Job> jobs = get_jobs(points_local.size(), num_threads);

	std::vector<std::vector<std::pair<int, int>>> nn_segments;
	nn_segments.resize(jobs.size());

	std::vector<std::thread> threads;

	for (size_t i = 0; i < jobs.size(); i++) {
		threads.push_back(
			std::thread(
				nearest_neighbours_job, 
				//i, 
				&jobs[i], 
				&nn_segments[i], 
				&this->points_local, 
				&pc_target.points_local,
				&this->points_type,
				&pc_target.points_type,
				pc_target.rgd_params,
				&pc_target.buckets,
				&pc_target.index_pairs,
				this->m_pose,
				pc_target.m_pose,
				search_radious));
	}

	for (size_t j = 0; j < threads.size(); j++) {
		threads[j].join();
	}
	threads.clear();

	for (size_t i = 0; i < nn_segments.size(); i++) {
		if (nn_segments[i].size() > 0) {
			nerest_neighbours.insert(nerest_neighbours.end(), nn_segments[i].begin(), nn_segments[i].end());
		}
	};

	return nerest_neighbours;
}

void PointCloud::clean()
{
	index_pairs.clear();
	buckets.clear();
}


inline void	gpuata3(double* AA, double* A)
{
	AA[3 * 0 + 0] = A[3 * 0 + 0] * A[3 * 0 + 0] + A[3 * 0 + 1] * A[3 * 0 + 1] + A[3 * 0 + 2] * A[3 * 0 + 2];
	AA[3 * 1 + 0] = A[3 * 0 + 0] * A[3 * 1 + 0] + A[3 * 0 + 1] * A[3 * 1 + 1] + A[3 * 0 + 2] * A[3 * 1 + 2];
	AA[3 * 2 + 0] = A[3 * 0 + 0] * A[3 * 2 + 0] + A[3 * 0 + 1] * A[3 * 2 + 1] + A[3 * 0 + 2] * A[3 * 2 + 2];

	AA[3 * 0 + 1] = AA[3 * 1 + 0];
	AA[3 * 1 + 1] = A[3 * 1 + 0] * A[3 * 1 + 0] + A[3 * 1 + 1] * A[3 * 1 + 1] + A[3 * 1 + 2] * A[3 * 1 + 2];
	AA[3 * 2 + 1] = A[3 * 1 + 0] * A[3 * 2 + 0] + A[3 * 1 + 1] * A[3 * 2 + 1] + A[3 * 1 + 2] * A[3 * 2 + 2];

	AA[3 * 0 + 2] = AA[3 * 2 + 0];
	AA[3 * 1 + 2] = AA[3 * 2 + 1];
	AA[3 * 2 + 2] = A[3 * 2 + 0] * A[3 * 2 + 0] + A[3 * 2 + 1] * A[3 * 2 + 1] + A[3 * 2 + 2] * A[3 * 2 + 2];
}

inline void gpusolvecubic(double* c)
{
	double sq3d2 = 0.86602540378443864676, c2d3 = c[2] / 3,
		c2sq = c[2] * c[2], Q = (3 * c[1] - c2sq) / 9,
		R = (c[2] * (9 * c[1] - 2 * c2sq) - 27 * c[0]) / 54;
	double tmp, t, sint, cost;

	if (Q < 0) {
		/*
		* Instead of computing
		* c_0 = A cos(t) - B
		* c_1 = A cos(t + 2 pi/3) - B
		* c_2 = A cos(t + 4 pi/3) - B
		* Use cos(a+b) = cos(a) cos(b) - sin(a) sin(b)
		* Keeps t small and eliminates 1 function call.
		* cos(2 pi/3) = cos(4 pi/3) = -0.5
		* sin(2 pi/3) = sqrtf(3.0f)/2
		* sin(4 pi/3) = -sqrtf(3.0f)/2
		*/

		tmp = 2 * sqrt(-Q);
		t = acos(R / sqrt(-Q * Q * Q)) / 3;
		cost = tmp * cos(t);
		sint = tmp * sin(t);

		c[0] = cost - c2d3;

		cost = -0.5 * cost - c2d3;
		sint = sq3d2 * sint;

		c[1] = cost - sint;
		c[2] = cost + sint;
	}
	else {
		tmp = cbrt(R);
		c[0] = -c2d3 + 2 * tmp;
		c[1] = c[2] = -c2d3 - tmp;
	}
}

inline void gpusort3(double* x)
{
	double tmp;

	if (x[0] < x[1]) {
		tmp = x[0];
		x[0] = x[1];
		x[1] = tmp;
	}
	if (x[1] < x[2]) {
		if (x[0] < x[2]) {
			tmp = x[2];
			x[2] = x[1];
			x[1] = x[0];
			x[0] = tmp;
		}
		else {
			tmp = x[1];
			x[1] = x[2];
			x[2] = tmp;
		}
	}
}

inline void gpuldu3(double* A, int* P)
{
	int tmp;

	P[1] = 1;
	P[2] = 2;

	P[0] = abs(A[3 * 1 + 0]) > abs(A[3 * 0 + 0]) ?
		(abs(A[3 * 2 + 0]) > abs(A[3 * 1 + 0]) ? 2 : 1) :
		(abs(A[3 * 2 + 0]) > abs(A[3 * 0 + 0]) ? 2 : 0);
	P[P[0]] = 0;

	if (abs(A[3 * P[2] + 1]) > abs(A[3 * P[1] + 1])) {
		tmp = P[1];
		P[1] = P[2];
		P[2] = tmp;
	}

	if (A[3 * P[0] + 0] != 0) {
		A[3 * P[1] + 0] = A[3 * P[1] + 0] / A[3 * P[0] + 0];
		A[3 * P[2] + 0] = A[3 * P[2] + 0] / A[3 * P[0] + 0];
		A[3 * P[0] + 1] = A[3 * P[0] + 1] / A[3 * P[0] + 0];
		A[3 * P[0] + 2] = A[3 * P[0] + 2] / A[3 * P[0] + 0];
	}

	A[3 * P[1] + 1] = A[3 * P[1] + 1] - A[3 * P[0] + 1] * A[3 * P[1] + 0] * A[3 * P[0] + 0];

	if (A[3 * P[1] + 1] != 0) {
		A[3 * P[2] + 1] = (A[3 * P[2] + 1] - A[3 * P[0] + 1] * A[3 * P[2] + 0] * A[3 * P[0] + 0]) / A[3 * P[1] + 1];
		A[3 * P[1] + 2] = (A[3 * P[1] + 2] - A[3 * P[0] + 2] * A[3 * P[1] + 0] * A[3 * P[0] + 0]) / A[3 * P[1] + 1];
	}

	A[3 * P[2] + 2] = A[3 * P[2] + 2] - A[3 * P[0] + 2] * A[3 * P[2] + 0] * A[3 * P[0] + 0] - A[3 * P[1] + 2] * A[3 * P[2] + 1] * A[3 * P[1] + 1];
}

inline void gpuldubsolve3(double* x, double* y, double* LDU, const int* P)
{
	x[P[2]] = y[2];
	x[P[1]] = y[1] - LDU[3 * P[2] + 1] * x[P[2]];
	x[P[0]] = y[0] - LDU[3 * P[2] + 0] * x[P[2]] - LDU[3 * P[1] + 0] * x[P[1]];
}

inline void gpucross(double* z, double* x, double* y)
{
	z[0] = x[1] * y[2] - x[2] * y[1];
	z[1] = -(x[0] * y[2] - x[2] * y[0]);
	z[2] = x[0] * y[1] - x[1] * y[0];
}

inline void gpumatvec3(double* y, double* A, double* x)
{
	y[0] = A[3 * 0 + 0] * x[0] + A[3 * 1 + 0] * x[1] + A[3 * 2 + 0] * x[2];
	y[1] = A[3 * 0 + 1] * x[0] + A[3 * 1 + 1] * x[1] + A[3 * 2 + 1] * x[2];
	y[2] = A[3 * 0 + 2] * x[0] + A[3 * 1 + 2] * x[1] + A[3 * 2 + 2] * x[2];
}

inline void gpumatmul3(double* C, double* A, double* B)
{
	C[3 * 0 + 0] = A[3 * 0 + 0] * B[3 * 0 + 0] + A[3 * 1 + 0] * B[3 * 0 + 1] + A[3 * 2 + 0] * B[3 * 0 + 2];
	C[3 * 1 + 0] = A[3 * 0 + 0] * B[3 * 1 + 0] + A[3 * 1 + 0] * B[3 * 1 + 1] + A[3 * 2 + 0] * B[3 * 1 + 2];
	C[3 * 2 + 0] = A[3 * 0 + 0] * B[3 * 2 + 0] + A[3 * 1 + 0] * B[3 * 2 + 1] + A[3 * 2 + 0] * B[3 * 2 + 2];

	C[3 * 0 + 1] = A[3 * 0 + 1] * B[3 * 0 + 0] + A[3 * 1 + 1] * B[3 * 0 + 1] + A[3 * 2 + 1] * B[3 * 0 + 2];
	C[3 * 1 + 1] = A[3 * 0 + 1] * B[3 * 1 + 0] + A[3 * 1 + 1] * B[3 * 1 + 1] + A[3 * 2 + 1] * B[3 * 1 + 2];
	C[3 * 2 + 1] = A[3 * 0 + 1] * B[3 * 2 + 0] + A[3 * 1 + 1] * B[3 * 2 + 1] + A[3 * 2 + 1] * B[3 * 2 + 2];

	C[3 * 0 + 2] = A[3 * 0 + 2] * B[3 * 0 + 0] + A[3 * 1 + 2] * B[3 * 0 + 1] + A[3 * 2 + 2] * B[3 * 0 + 2];
	C[3 * 1 + 2] = A[3 * 0 + 2] * B[3 * 1 + 0] + A[3 * 1 + 2] * B[3 * 1 + 1] + A[3 * 2 + 2] * B[3 * 1 + 2];
	C[3 * 2 + 2] = A[3 * 0 + 2] * B[3 * 2 + 0] + A[3 * 1 + 2] * B[3 * 2 + 1] + A[3 * 2 + 2] * B[3 * 2 + 2];
}

inline void gpuunit3(double* x)
{
	double tmp = sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
	x[0] /= tmp;
	x[1] /= tmp;
	x[2] /= tmp;
}

inline void gpuSVD(double* _A, double* U, double* _S, double* V)
{
	double A[9];
	for (int i = 0; i < 9; i++)A[i] = (double)_A[i];

	double thr = 1e-10;
	int P[3], k;
	double y[3], AA[3][3], LDU[3][3];

	double S[3];
	/*
	* Steps:
	* 1) Use eigendecomposition on A^T A to compute V.
	* Since A = U S V^T then A^T A = V S^T S V^T with D = S^T S and V the
	* eigenvalues and eigenvectors respectively (V is orthogonal).
	* 2) Compute U from A and V.
	* 3) Normalize columns of U and V and root the eigenvalues to obtain
	* the singular values.
	*/

	/* Compute AA = A^T A */
	gpuata3((double*)AA, A);

	/* Form the monic characteristic polynomial */
	S[2] = -AA[0][0] - AA[1][1] - AA[2][2];
	S[1] = AA[0][0] * AA[1][1] + AA[2][2] * AA[0][0] + AA[2][2] * AA[1][1] -
		AA[2][1] * AA[1][2] - AA[2][0] * AA[0][2] - AA[1][0] * AA[0][1];
	S[0] = AA[2][1] * AA[1][2] * AA[0][0] + AA[2][0] * AA[0][2] * AA[1][1] + AA[1][0] * AA[0][1] * AA[2][2] -
		AA[0][0] * AA[1][1] * AA[2][2] - AA[1][0] * AA[2][1] * AA[0][2] - AA[2][0] * AA[0][1] * AA[1][2];

	/* Solve the cubic equation. */
	gpusolvecubic(S);

	/* All roots should be positive */
	if (S[0] < 0)
		S[0] = 0;
	if (S[1] < 0)
		S[1] = 0;
	if (S[2] < 0)
		S[2] = 0;

	/* Sort from greatest to least */
	gpusort3(S);

	/* Form the eigenvector system for the first (largest) eigenvalue */
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		{
			LDU[i][j] = AA[i][j];
		}

	LDU[0][0] -= S[0];
	LDU[1][1] -= S[0];
	LDU[2][2] -= S[0];

	/* Perform LDUP decomposition */
	gpuldu3((double*)LDU, P);

	/*
	* Write LDU = AA-I*lambda.  Then an eigenvector can be
	* found by solving LDU x = LD y = L z = 0
	* L is invertible, so L z = 0 implies z = 0
	* D is singular since det(AA-I*lambda) = 0 and so
	* D y = z = 0 has a non-unique solution.
	* Pick k so that D_kk = 0 and set y = e_k, the k'th column
	* of the identity matrix.
	* U is invertible so U x = y has a unique solution for a given y.
	* The solution for U x = y is an eigenvector.
	*/

	/* Pick the component of D nearest to 0 */
	y[0] = y[1] = y[2] = 0;
	k = abs(LDU[P[1]][1]) < abs(LDU[P[0]][0]) ?
		(abs(LDU[P[2]][2]) < abs(LDU[P[1]][1]) ? 2 : 1) :
		(abs(LDU[P[2]][2]) < abs(LDU[P[0]][0]) ? 2 : 0);
	y[k] = 1;

	/* Do a backward solve for the eigenvector */
	gpuldubsolve3(V + (3 * 0 + 0), y, (double*)LDU, P);

	/* Form the eigenvector system for the last (smallest) eigenvalue */
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		{
			LDU[i][j] = AA[i][j];
		}

	LDU[0][0] -= S[2];
	LDU[1][1] -= S[2];
	LDU[2][2] -= S[2];

	/* Perform LDUP decomposition */
	gpuldu3((double*)LDU, P);

	/*
	* NOTE: The arrangement of the ternary operator output is IMPORTANT!
	* It ensures a different system is solved if there are 3 repeat eigenvalues.
	*/

	/* Pick the component of D nearest to 0 */
	y[0] = y[1] = y[2] = 0;
	k = abs(LDU[P[0]][0]) < abs(LDU[P[2]][2]) ?
		(abs(LDU[P[0]][0]) < abs(LDU[P[1]][1]) ? 0 : 1) :
		(abs(LDU[P[1]][1]) < abs(LDU[P[2]][2]) ? 1 : 2);
	y[k] = 1;

	/* Do a backward solve for the eigenvector */
	gpuldubsolve3(V + (3 * 2 + 0), y, (double*)LDU, P);

	/* The remaining column must be orthogonal (AA is symmetric) */
	gpucross(V + (3 * 1 + 0), V + (3 * 2 + 0), V + (3 * 0 + 0));

	/* Count the rank */
	k = (S[0] > thr) + (S[1] > thr) + (S[2] > thr);

	switch (k) {
	case 0:
		/*
		* Zero matrix.
		* Since V is already orthogonal, just copy it into U.
		*/
		for (int i = 0; i < 9; i++)U[i] = V[i];

		break;
	case 1:
		/*
		* The first singular value is non-zero.
		* Since A = U S V^T, then A V = U S.
		* A V_1 = S_11 U_1 is non-zero. Here V_1 and U_1 are
		* column vectors. Since V_1 is known, we may compute
		* U_1 = A V_1.  The S_11 factor is not important as
		* U_1 will be normalized later.
		*/
		gpumatvec3(U + (3 * 0 + 0), A, V + (3 * 0 + 0));

		/*
		* The other columns of U do not contribute to the expansion
		* and we may arbitrarily choose them (but they do need to be
		* orthogonal). To ensure the first cross product does not fail,
		* pick k so that U_k1 is nearest 0 and then cross with e_k to
		* obtain an orthogonal vector to U_1.
		*/
		y[0] = y[1] = y[2] = 0;
		k = abs(U[3 * 0 + 0]) < abs(U[3 * 0 + 2]) ?
			(abs(U[3 * 0 + 0]) < abs(U[3 * 0 + 1]) ? 0 : 1) :
			(abs(U[3 * 0 + 1]) < abs(U[3 * 0 + 2]) ? 1 : 2);
		y[k] = 1;

		gpucross(U + (3 * 1 + 0), y, U + (3 * 0 + 0));

		/* Cross the first two to obtain the remaining column */
		gpucross(U + (3 * 2 + 0), U + (3 * 0 + 0), U + (3 * 1 + 0));
		break;
	case 2:
		/*
		* The first two singular values are non-zero.
		* Compute U_1 = A V_1 and U_2 = A V_2. See case 1
		* for more information.
		*/
		gpumatvec3(U + (3 * 0 + 0), A, V + (3 * 0 + 0));
		gpumatvec3(U + (3 * 1 + 0), A, V + (3 * 1 + 0));

		/* Cross the first two to obtain the remaining column */
		gpucross(U + (3 * 2 + 0), U + (3 * 0 + 0), U + (3 * 1 + 0));
		break;
	case 3:
		/*
		* All singular values are non-zero.
		* We may compute U = A V. See case 1 for more information.
		*/
		gpumatmul3(U, A, V);
		break;
	}

	/* Normalize the columns of U and V */
	gpuunit3(V + (3 * 0 + 0));
	gpuunit3(V + (3 * 1 + 0));
	gpuunit3(V + (3 * 2 + 0));

	gpuunit3(U + (3 * 0 + 0));
	gpuunit3(U + (3 * 1 + 0));
	gpuunit3(U + (3 * 2 + 0));

	/* S was initially the eigenvalues of A^T A = V S^T S V^T which are squared. */
	S[0] = sqrt(S[0]);
	S[1] = sqrt(S[1]);
	S[2] = sqrt(S[2]);

	for (int i = 0; i < 9; i++)_S[i] = 0.0;

	_S[0] = S[0];
	_S[4] = S[1];
	_S[8] = S[2];
}

void compute_normal_vectors_job(
	int i, 
	PointCloud::Job* job, 
	std::vector<Eigen::Vector3d>* points, 
	PointCloud::GridParameters rgd_params, 
	std::vector<PointCloud::Bucket>* buckets,
	std::vector<PointCloud::PointBucketIndexPair>* index_pairs, 
	std::vector<Eigen::Vector3d>* normal_vectors,
	double search_radious
) {
	for (size_t ii = job->index_begin_inclusive; ii < job->index_end_exclusive; ii++) {
		Eigen::Vector3d& point = (*points)[ii];
		Eigen::Vector3d& normal_vector = (*normal_vectors)[ii];

		if (point.x() < rgd_params.bounding_box_min_X || point.x() > rgd_params.bounding_box_max_X)continue;
		if (point.y() < rgd_params.bounding_box_min_Y || point.y() > rgd_params.bounding_box_max_Y)continue;
		if (point.z() < rgd_params.bounding_box_min_Z || point.z() > rgd_params.bounding_box_max_Z)continue;

		normal_vector.x() = 0;
		normal_vector.y() = 0;
		normal_vector.z() = 0;
		//point.valid_nv = false;

		Eigen::Vector3d mean(0.0, 0.0, 0.0);
		int number_of_points_nn = 0;

		long long unsigned int ix = (point.x() - rgd_params.bounding_box_min_X) / rgd_params.resolution_X;
		long long unsigned int iy = (point.y() - rgd_params.bounding_box_min_Y) / rgd_params.resolution_Y;
		long long unsigned int iz = (point.z() - rgd_params.bounding_box_min_Z) / rgd_params.resolution_Z;


		long long unsigned int index_bucket = ix * static_cast<long long unsigned int>(rgd_params.number_of_buckets_Y) *
			static_cast<long long unsigned int>(rgd_params.number_of_buckets_Z) + iy * static_cast<long long unsigned int>(rgd_params.number_of_buckets_Z) + iz;

		///////////////////////////////////
		if (index_bucket >= 0 && index_bucket < rgd_params.number_of_buckets)
		{
			int sx, sy, sz, stx, sty, stz;
			if (ix == 0)sx = 0; else sx = -1;
			if (iy == 0)sy = 0; else sy = -1;
			if (iz == 0)sz = 0; else sz = -1;

			if (ix == rgd_params.number_of_buckets_X - 1)stx = 1; else stx = 2;
			if (iy == rgd_params.number_of_buckets_Y - 1)sty = 1; else sty = 2;
			if (iz == rgd_params.number_of_buckets_Z - 1)stz = 1; else stz = 2;

			long long unsigned int index_next_bucket;
			int iter;
			int number_of_points_in_bucket;
			int l_begin;
			int l_end;
			float dist = 0.0f;

			///////////////////////////////////////////////////////////////////////////////////////////
			for (int i = sx; i < stx; i++) {
				for (int j = sy; j < sty; j++) {
					for (int k = sz; k < stz; k++) {
						index_next_bucket = index_bucket +
							i * rgd_params.number_of_buckets_Y * rgd_params.number_of_buckets_Z +
							j * rgd_params.number_of_buckets_Z + k;

						if (index_next_bucket >= 0 && index_next_bucket < (*buckets).size()) {

							PointCloud::Bucket& b = (*buckets)[index_next_bucket];

							number_of_points_in_bucket = b.number_of_points;

							if (number_of_points_in_bucket <= 0)continue;

							int max_number_considered_in_bucket;
							if (index_next_bucket == index_bucket) {
								max_number_considered_in_bucket = 100;
							}
							else {
								max_number_considered_in_bucket = 100;
							}
							if (max_number_considered_in_bucket <= 0)continue;

							if (max_number_considered_in_bucket >= number_of_points_in_bucket) {
								iter = 1;
							}
							else {
								iter = number_of_points_in_bucket / max_number_considered_in_bucket;
								if (iter <= 0)iter = 1;
							}

							l_begin = b.index_begin;
							l_end = b.index_end;

							for (int l = l_begin; l < l_end; l += iter) {
								if (l >= 0 && l < (*index_pairs).size())
								{
									int hashed_index_of_point = (*index_pairs)[l].index_of_point;

									if (hashed_index_of_point >= 0 && hashed_index_of_point < (*points).size()) {
										Eigen::Vector3d& point_nn = (*points)[hashed_index_of_point];

										double dist = (point_nn - point).norm();


										if (dist < search_radious) {
											mean += point_nn;
											number_of_points_nn++;
										}
									}
								}
							}
						}
					}
				}
			}
		}

		if (number_of_points_nn >= 3) {
			mean /= number_of_points_nn;


			double cov[3][3];
			cov[0][0] = cov[0][1] = cov[0][2] = cov[1][0] = cov[1][1] = cov[1][2] = cov[2][0] = cov[2][1] = cov[2][2] = 0;
			number_of_points_nn = 0;

			if (index_bucket >= 0 && index_bucket < rgd_params.number_of_buckets)
			{
				int sx, sy, sz, stx, sty, stz;
				if (ix == 0)sx = 0; else sx = -1;
				if (iy == 0)sy = 0; else sy = -1;
				if (iz == 0)sz = 0; else sz = -1;

				if (ix == rgd_params.number_of_buckets_X - 1)stx = 1; else stx = 2;
				if (iy == rgd_params.number_of_buckets_Y - 1)sty = 1; else sty = 2;
				if (iz == rgd_params.number_of_buckets_Z - 1)stz = 1; else stz = 2;


				long long unsigned int index_next_bucket;
				int iter;
				int number_of_points_in_bucket;
				int l_begin;
				int l_end;
				float dist = 0.0f;


				///////////////////////////////////////////////////////////////////////////////////////////
				for (int i = sx; i < stx; i++) {
					for (int j = sy; j < sty; j++) {
						for (int k = sz; k < stz; k++) {
							index_next_bucket = index_bucket +
								i * rgd_params.number_of_buckets_Y * rgd_params.number_of_buckets_Z +
								j * rgd_params.number_of_buckets_Z + k;

							if (index_next_bucket >= 0 && index_next_bucket < (*buckets).size()) {

								PointCloud::Bucket& b = (*buckets)[index_next_bucket];

								number_of_points_in_bucket = b.number_of_points;

								if (number_of_points_in_bucket <= 0)continue;

								int max_number_considered_in_bucket;
								if (index_next_bucket == index_bucket) {
									max_number_considered_in_bucket = 100;
								}
								else {
									max_number_considered_in_bucket = 100;
								}
								if (max_number_considered_in_bucket <= 0)continue;

								if (max_number_considered_in_bucket >= number_of_points_in_bucket) {
									iter = 1;
								}
								else {
									iter = number_of_points_in_bucket / max_number_considered_in_bucket;
									if (iter <= 0)iter = 1;
								}

								l_begin = b.index_begin;
								l_end = b.index_end;

								for (int l = l_begin; l < l_end; l += iter) {
									if (l >= 0 && l < (*index_pairs).size())
									{
										int hashed_index_of_point = (*index_pairs)[l].index_of_point;

										if (hashed_index_of_point >= 0 && hashed_index_of_point < (*points).size()) {
											Eigen::Vector3d& point_nn = (*points)[hashed_index_of_point];

											double dist = (point_nn - point).norm();


											if (dist < search_radious) {
												cov[0][0] += (mean.x() - point_nn.x()) * (mean.x() - point_nn.x());
												cov[0][1] += (mean.x() - point_nn.x()) * (mean.y() - point_nn.y());
												cov[0][2] += (mean.x() - point_nn.x()) * (mean.z() - point_nn.z());
												cov[1][0] += (mean.y() - point_nn.y()) * (mean.x() - point_nn.x());
												cov[1][1] += (mean.y() - point_nn.y()) * (mean.y() - point_nn.y());
												cov[1][2] += (mean.y() - point_nn.y()) * (mean.z() - point_nn.z());
												cov[2][0] += (mean.z() - point_nn.z()) * (mean.x() - point_nn.x());
												cov[2][1] += (mean.z() - point_nn.z()) * (mean.y() - point_nn.y());
												cov[2][2] += (mean.z() - point_nn.z()) * (mean.z() - point_nn.z());
												number_of_points_nn++;
											}
										}
									}
								}//for (int l = l_begin; l < l_end; l += iter){
							}
						}
					}
				}//for (int i = sx; i < stx; i++)
			}//if (index_bucket >= 0 && index_bucket < rgd_params.number_of_buckets)

			if (number_of_points_nn >= 3) {
				cov[0][0] /= number_of_points_nn;
				cov[0][1] /= number_of_points_nn;
				cov[0][2] /= number_of_points_nn;
				cov[1][0] /= number_of_points_nn;
				cov[1][1] /= number_of_points_nn;
				cov[1][2] /= number_of_points_nn;
				cov[2][0] /= number_of_points_nn;
				cov[2][1] /= number_of_points_nn;
				cov[2][2] /= number_of_points_nn;

				double U[3][3], V[3][3];
				double SS[9];
				gpuSVD((double*)cov, (double*)U, (double*)SS, (double*)V);
				double _nx = (float)(U[0][1] * U[1][2] - U[0][2] * U[1][1]);
				double _ny = (float)(-(U[0][0] * U[1][2] - U[0][2] * U[1][0]));
				double _nz = (float)(U[0][0] * U[1][1] - U[0][1] * U[1][0]);

				double lenght = sqrt(_nx * _nx + _ny * _ny + _nz * _nz);
				if (lenght == 0)
				{
					normal_vector.x() = 0.0f;
					normal_vector.y() = 0.0f;
					normal_vector.z() = 0.0f;
				}
				else {
					normal_vector.x() = _nx / lenght;
					normal_vector.y() = _ny / lenght;
					normal_vector.z() = _nz / lenght;
				}
			}
		}//if(number_of_points_nn >= 3)
	}
}

void PointCloud::compute_normal_vectors(double search_radious)
{
	number_points_vertical = 0;
	number_points_horizontal = 0;

	normal_vectors_local.clear();
	normal_vectors_local.resize(points_local.size());
	points_type.resize(points_local.size());
	std::vector<std::thread> threads;
	std::vector<Job> jobs = get_jobs(points_local.size(), num_threads);

	for (size_t i = 0; i < jobs.size(); i++) {
		threads.push_back(std::thread(compute_normal_vectors_job, 
			i, 
			&jobs[i], 
			&points_local, 
			rgd_params, 
			&buckets, 
			&index_pairs, 
			&normal_vectors_local,
			search_radious
		));
	}

	for (size_t j = 0; j < threads.size(); j++) {
		threads[j].join();
	}
	threads.clear();
		
	for (size_t i = 0; i < normal_vectors_local.size(); i++) {
		auto nv = normal_vectors_local[i];
		nv = this->m_pose.rotation() * nv;

		if (fabs(nv.z()) > 0.8) {
			number_points_horizontal++;
			points_type[i] = 0;
		}
		else {
			number_points_vertical++;
			points_type[i] = 1;
		}
	}
	std::cout << "number_points_horizontal: " << number_points_horizontal << " number_points_vertical: " << number_points_vertical << std::endl;
}

void PointCloud::decimate(double bucket_x, double bucket_y, double bucket_z)
{
	auto params = rgd_params;

	params.resolution_X = bucket_x;
	params.resolution_Y = bucket_y;
	params.resolution_Z = bucket_z;
	params.bounding_box_extension = 1.0;

	grid_calculate_params(this->points_local, params);
	cout_rgd();

	std::vector<PointBucketIndexPair> ip;
	reindex(ip, this->points_local, params);

	std::vector<Eigen::Vector3d> n_points_local;
	std::vector<Eigen::Vector3d> n_normal_vectors_local;
	std::vector <int> n_points_type;
	std::vector <float> n_intensities;

	for (int i = 1; i < ip.size(); i++) {
		if (ip[i - 1].index_of_bucket != ip[i].index_of_bucket) {
			//std::cout << index_pairs[i].index_of_bucket << std::endl;
			n_points_local.emplace_back(points_local[ip[i].index_of_point]);
			if (normal_vectors_local.size() == points_local.size())n_normal_vectors_local.emplace_back(normal_vectors_local[ip[i].index_of_point]);
			if (points_type.size() == points_local.size())n_points_type.emplace_back(n_points_type[ip[i].index_of_point]);
			if (intensities.size() == points_local.size())n_intensities.emplace_back(n_intensities[ip[i].index_of_point]);
		}
	}

	points_local = n_points_local;
	normal_vectors_local = n_normal_vectors_local;
	points_type = n_points_type;
	intensities = n_intensities;
}
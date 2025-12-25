#include <point_clouds.h>
#include <transformations.h>

#include <iostream>
#include <cstdlib>
#include <sstream>
#include <string>
#include <fstream>
#include <vector>
#include <algorithm>
#include <set>
#include <filesystem>
#include <laszip/laszip_api.h>
#include <execution>

// #include <liblas/liblas.hpp>
// #include <laszip/laszip_api.h>
inline void split(std::string &str, char delim, std::vector<std::string> &out)
{
	size_t start;
	size_t end = 0;

	while ((start = str.find_first_not_of(delim, end)) != std::string::npos)
	{
		end = str.find(delim, start);
		out.push_back(str.substr(start, end - start));
	}
}

bool PointClouds::load(const std::string &folder_with_point_clouds, const std::string &poses_file_name, bool decimation,
					   double bucket_x, double bucket_y, double bucket_z, bool load_cache_mode)
{
	point_clouds.clear();

	std::ifstream infile(poses_file_name);
	if (!infile.good())
	{
		std::cout << "problem with file: '" << poses_file_name << "'" << std::endl;
		return false;
	}
	std::string line;
	std::getline(infile, line);
	std::istringstream iss(line);

	int num_scans;
	iss >> num_scans;

	std::cout << "number of scans: " << num_scans << std::endl;
	size_t sum_points_before_decimation = 0;
	size_t sum_points_after_decimation = 0;

	for (size_t i = 0; i < num_scans; i++)
	{
		std::getline(infile, line);
		std::istringstream iss(line);
		std::string point_cloud_file_name;
		iss >> point_cloud_file_name;

		double r11, r12, r13, r21, r22, r23, r31, r32, r33;
		double t14, t24, t34;

		std::getline(infile, line);
		std::istringstream iss1(line);
		iss1 >> r11 >> r12 >> r13 >> t14;

		std::getline(infile, line);
		std::istringstream iss2(line);
		iss2 >> r21 >> r22 >> r23 >> t24;

		std::getline(infile, line);
		std::istringstream iss3(line);
		iss3 >> r31 >> r32 >> r33 >> t34;

		std::getline(infile, line);

		PointCloud pc;
		pc.file_name = point_cloud_file_name;
		pc.m_pose = Eigen::Affine3d::Identity();
		pc.m_pose(0, 0) = r11;
		pc.m_pose(0, 1) = r12;
		pc.m_pose(0, 2) = r13;
		pc.m_pose(1, 0) = r21;
		pc.m_pose(1, 1) = r22;
		pc.m_pose(1, 2) = r23;
		pc.m_pose(2, 0) = r31;
		pc.m_pose(2, 1) = r32;
		pc.m_pose(2, 2) = r33;
		pc.m_pose(0, 3) = t14;
		pc.m_pose(1, 3) = t24;
		pc.m_pose(2, 3) = t34;

		pc.m_initial_pose = pc.m_pose;

		pc.pose = pose_tait_bryan_from_affine_matrix(pc.m_pose);
		pc.gui_translation[0] = pc.pose.px;
		pc.gui_translation[1] = pc.pose.py;
		pc.gui_translation[2] = pc.pose.pz;
		pc.gui_rotation[0] = rad2deg(pc.pose.om);
		pc.gui_rotation[1] = rad2deg(pc.pose.fi);
		pc.gui_rotation[2] = rad2deg(pc.pose.ka);

		if (!load_cache_mode)
		{
			if (!pc.load(folder_with_point_clouds + "/" + pc.file_name))
			{
				point_clouds.clear();
				std::cout << "problem with file '" << folder_with_point_clouds + "/" + pc.file_name << "'" << std::endl;
				return false;
			}
			if (decimation)
			{
				sum_points_before_decimation += pc.points_local.size();
				pc.decimate(bucket_x, bucket_y, bucket_z);
				sum_points_after_decimation += pc.points_local.size();
			}
		}

		point_clouds.push_back(pc);
	}
	infile.close();
	folder_name = folder_with_point_clouds;

	std::cout << "all scans, sum_points_before_decimation: " << sum_points_before_decimation << std::endl;
	std::cout << "all scans, sum_points_after_decimation: " << sum_points_after_decimation << std::endl;
	print_point_cloud_dimension();
	return true;
}

bool PointClouds::update_poses_from_RESSO(const std::string &folder_with_point_clouds, const std::string &poses_file_name)
{
	std::ifstream infile(poses_file_name);
	if (!infile.good())
	{
		std::cout << "problem with file: '" << poses_file_name << "' (!infile.good())" << std::endl;
		//	return false;
	}
	std::string line;
	std::getline(infile, line);
	std::istringstream iss(line);

	int num_scans;
	iss >> num_scans;

	std::cout << "number of scans: " << num_scans << std::endl;

	std::vector<PointCloud> pcs;

	for (size_t i = 0; i < num_scans; i++)
	{
		std::getline(infile, line);
		std::istringstream iss(line);
		std::string point_cloud_file_name;
		iss >> point_cloud_file_name;

		double r11, r12, r13, r21, r22, r23, r31, r32, r33;
		double t14, t24, t34;

		std::getline(infile, line);
		std::istringstream iss1(line);
		iss1 >> r11 >> r12 >> r13 >> t14;

		std::getline(infile, line);
		std::istringstream iss2(line);
		iss2 >> r21 >> r22 >> r23 >> t24;

		std::getline(infile, line);
		std::istringstream iss3(line);
		iss3 >> r31 >> r32 >> r33 >> t34;

		std::getline(infile, line);

		PointCloud pc;
		pc.file_name = point_cloud_file_name;
		pc.m_pose = Eigen::Affine3d::Identity();
		pc.m_pose(0, 0) = r11;
		pc.m_pose(0, 1) = r12;
		pc.m_pose(0, 2) = r13;
		pc.m_pose(1, 0) = r21;
		pc.m_pose(1, 1) = r22;
		pc.m_pose(1, 2) = r23;
		pc.m_pose(2, 0) = r31;
		pc.m_pose(2, 1) = r32;
		pc.m_pose(2, 2) = r33;
		pc.m_pose(0, 3) = t14;
		pc.m_pose(1, 3) = t24;
		pc.m_pose(2, 3) = t34;
		// pc.m_initial_pose = pc.m_pose;

		// std::cout << "update pose: " << std::endl;
		// std::cout << pc.m_pose.matrix() << std::endl;

		pcs.push_back(pc);
	}
	infile.close();

	if (point_clouds.size() == pcs.size())
	{
		for (int i = 0; i < point_clouds.size(); i++)
		{
			for (int j = 0; j < pcs.size(); j++)
			{
				if (std::filesystem::path(point_clouds[i].file_name).filename().string() == pcs[j].file_name)
				{
					// std::cout << "-------------------------" << std::endl;
					// std::cout << "update pose: " << i << std::endl;
					// std::cout << "previous pose: " << std::endl
					//		  << point_clouds[i].m_pose.matrix() << std::endl;
					// std::cout << "current pose: " << std::endl
					//		  << pcs[j].m_pose.matrix() << std::endl;

					point_clouds[i].m_pose = pcs[j].m_pose;
					point_clouds[i].pose = pose_tait_bryan_from_affine_matrix(point_clouds[i].m_pose);
					point_clouds[i].gui_translation[0] = point_clouds[i].pose.px;
					point_clouds[i].gui_translation[1] = point_clouds[i].pose.py;
					point_clouds[i].gui_translation[2] = point_clouds[i].pose.pz;
					point_clouds[i].gui_rotation[0] = rad2deg(point_clouds[i].pose.om);
					point_clouds[i].gui_rotation[1] = rad2deg(point_clouds[i].pose.fi);
					point_clouds[i].gui_rotation[2] = rad2deg(point_clouds[i].pose.ka);
				} // else{
				  //	std::cout << "std::filesystem::path(point_clouds[i].file_name).filename().string() != pcs[j].file_name" << std::endl;
				  //	std::cout << "std::filesystem::path(point_clouds[i].file_name).filename().string(): "<< std::filesystem::path(point_clouds[i].file_name).filename().string() << std::endl;
				  //	std::cout << "pcs[j].file_name: "<< pcs[j].file_name << std::endl;
				  //	std::cout << "j: " << j << std::endl;
				  // return false;
				  //}
			}

			/**/

			// point_clouds[i].m_initial_pose = point_clouds[i].m_pose;
		}
	}
	else
	{
		std::cout << "PROBLEM point_clouds.size() != pcs.size()" << std::endl;
		std::cout << "point_clouds.size() " << point_clouds.size() << std::endl;
		std::cout << "poses.size() " << pcs.size() << std::endl;
		return false;
	}
	return true;
}

bool PointClouds::update_poses_from_RESSO_inverse(const std::string &folder_with_point_clouds, const std::string &poses_file_name)
{
	std::ifstream infile(poses_file_name);
	if (!infile.good())
	{
		std::cout << "problem with file: '" << poses_file_name << "' (!infile.good())" << std::endl;
		//	return false;
	}
	std::string line;
	std::getline(infile, line);
	std::istringstream iss(line);

	int num_scans;
	iss >> num_scans;

	std::cout << "number of scans: " << num_scans << std::endl;

	std::vector<PointCloud> pcs;

	for (size_t i = 0; i < num_scans; i++)
	{
		std::getline(infile, line);
		std::istringstream iss(line);
		std::string point_cloud_file_name;
		iss >> point_cloud_file_name;

		double r11, r12, r13, r21, r22, r23, r31, r32, r33;
		double t14, t24, t34;

		std::getline(infile, line);
		std::istringstream iss1(line);
		iss1 >> r11 >> r12 >> r13 >> t14;

		std::getline(infile, line);
		std::istringstream iss2(line);
		iss2 >> r21 >> r22 >> r23 >> t24;

		std::getline(infile, line);
		std::istringstream iss3(line);
		iss3 >> r31 >> r32 >> r33 >> t34;

		std::getline(infile, line);

		PointCloud pc;
		pc.file_name = point_cloud_file_name;
		pc.m_pose = Eigen::Affine3d::Identity();
		pc.m_pose(0, 0) = r11;
		pc.m_pose(0, 1) = r12;
		pc.m_pose(0, 2) = r13;
		pc.m_pose(1, 0) = r21;
		pc.m_pose(1, 1) = r22;
		pc.m_pose(1, 2) = r23;
		pc.m_pose(2, 0) = r31;
		pc.m_pose(2, 1) = r32;
		pc.m_pose(2, 2) = r33;
		pc.m_pose(0, 3) = t14;
		pc.m_pose(1, 3) = t24;
		pc.m_pose(2, 3) = t34;
		// pc.m_initial_pose = pc.m_pose;

		pc.m_pose = pc.m_pose.inverse();
		std::cout << "update pose: " << std::endl;
		std::cout << pc.m_pose.matrix() << std::endl;

		pcs.push_back(pc);
	}
	infile.close();

	if (point_clouds.size() == pcs.size())
	{
		for (int i = 0; i < point_clouds.size(); i++)
		{
			for (int j = 0; j < pcs.size(); j++)
			{
				if (std::filesystem::path(point_clouds[i].file_name).filename().string() == pcs[j].file_name)
				{
					// std::cout << "-------------------------" << std::endl;
					// std::cout << "update pose: " << i << std::endl;
					// std::cout << "previous pose: " << std::endl
					//		  << point_clouds[i].m_pose.matrix() << std::endl;
					// std::cout << "current pose: " << std::endl
					//		  << pcs[j].m_pose.matrix() << std::endl;

					point_clouds[i].m_pose = pcs[j].m_pose;
					point_clouds[i].pose = pose_tait_bryan_from_affine_matrix(point_clouds[i].m_pose);
					point_clouds[i].gui_translation[0] = point_clouds[i].pose.px;
					point_clouds[i].gui_translation[1] = point_clouds[i].pose.py;
					point_clouds[i].gui_translation[2] = point_clouds[i].pose.pz;
					point_clouds[i].gui_rotation[0] = rad2deg(point_clouds[i].pose.om);
					point_clouds[i].gui_rotation[1] = rad2deg(point_clouds[i].pose.fi);
					point_clouds[i].gui_rotation[2] = rad2deg(point_clouds[i].pose.ka);
				} // else{
				  //	std::cout << "std::filesystem::path(point_clouds[i].file_name).filename().string() != pcs[j].file_name" << std::endl;
				  //	std::cout << "std::filesystem::path(point_clouds[i].file_name).filename().string(): "<< std::filesystem::path(point_clouds[i].file_name).filename().string() << std::endl;
				  //	std::cout << "pcs[j].file_name: "<< pcs[j].file_name << std::endl;
				  //	std::cout << "j: " << j << std::endl;
				  // return false;
				  //}
			}

			/**/

			// point_clouds[i].m_initial_pose = point_clouds[i].m_pose;
		}
	}
	else
	{
		std::cout << "PROBLEM point_clouds.size() != pcs.size()" << std::endl;
		std::cout << "point_clouds.size() " << point_clouds.size() << std::endl;
		std::cout << "poses.size() " << pcs.size() << std::endl;
		return false;
	}
	return true;
}

bool PointClouds::update_initial_poses_from_RESSO(const std::string &folder_with_point_clouds, const std::string &poses_file_name)
{
	std::ifstream infile(poses_file_name);
	if (!infile.good())
	{
		std::cout << "problem with file: '" << poses_file_name << "'" << std::endl;
		return false;
	}
	std::string line;
	std::getline(infile, line);
	std::istringstream iss(line);

	int num_scans;
	iss >> num_scans;

	std::cout << "number of scans: " << num_scans << std::endl;

	std::vector<PointCloud> pcs;

	for (size_t i = 0; i < num_scans; i++)
	{
		std::getline(infile, line);
		std::istringstream iss(line);
		std::string point_cloud_file_name;
		iss >> point_cloud_file_name;

		double r11, r12, r13, r21, r22, r23, r31, r32, r33;
		double t14, t24, t34;

		std::getline(infile, line);
		std::istringstream iss1(line);
		iss1 >> r11 >> r12 >> r13 >> t14;

		std::getline(infile, line);
		std::istringstream iss2(line);
		iss2 >> r21 >> r22 >> r23 >> t24;

		std::getline(infile, line);
		std::istringstream iss3(line);
		iss3 >> r31 >> r32 >> r33 >> t34;

		std::getline(infile, line);

		PointCloud pc;
		pc.file_name = point_cloud_file_name;
		pc.m_initial_pose = Eigen::Affine3d::Identity();
		pc.m_initial_pose(0, 0) = r11;
		pc.m_initial_pose(0, 1) = r12;
		pc.m_initial_pose(0, 2) = r13;
		pc.m_initial_pose(1, 0) = r21;
		pc.m_initial_pose(1, 1) = r22;
		pc.m_initial_pose(1, 2) = r23;
		pc.m_initial_pose(2, 0) = r31;
		pc.m_initial_pose(2, 1) = r32;
		pc.m_initial_pose(2, 2) = r33;
		pc.m_initial_pose(0, 3) = t14;
		pc.m_initial_pose(1, 3) = t24;
		pc.m_initial_pose(2, 3) = t34;
		// pc.m_initial_pose = pc.m_pose;
		pcs.push_back(pc);
	}
	infile.close();

	if (point_clouds.size() == pcs.size())
	{
		for (int i = 0; i < point_clouds.size(); i++)
		{
			for (int j = 0; j < pcs.size(); j++)
			{
				if (std::filesystem::path(point_clouds[i].file_name).filename().string() == pcs[j].file_name)
				{
					// std::cout << "-------------------------" << std::endl;
					// std::cout << "update pose: " << i << std::endl;
					// std::cout << "previous pose: " << std::endl
					//		  << point_clouds[i].m_initial_pose.matrix() << std::endl;
					// std::cout << "current pose: " << std::endl
					//		  << pcs[j].m_initial_pose.matrix() << std::endl;

					point_clouds[i].m_initial_pose = pcs[j].m_initial_pose;
				}
			}
		}
	}
	return true;
}

bool PointClouds::load_eth(const std::string &folder_with_point_clouds, const std::string &poses_file_name, bool decimation, double bucket_x, double bucket_y, double bucket_z)
{
	point_clouds.clear();

	std::cout << "Loading from file: " << poses_file_name << std::endl;
	std::cout << "ply files are located at: " << folder_with_point_clouds << std::endl;

	std::vector<std::pair<std::string, std::string>> pairs;

	std::ifstream infile(poses_file_name);
	if (!infile.good())
	{
		std::cout << "problem with file: '" << poses_file_name << "'" << std::endl;
		return false;
	}
	std::string line;
	// std::getline(infile, line);
	while (std::getline(infile, line))
	{
		std::istringstream iss(line);
		std::pair<std::string, std::string> pair;
		iss >> pair.first >> pair.second;
		pairs.push_back(pair);
	}

	infile.close();

	std::cout << "pairs" << std::endl;
	for (size_t i = 0; i < pairs.size(); i++)
	{
		std::cout << pairs[i].first << " " << pairs[i].second << std::endl;
	}

	std::set<std::string> names_set;
	for (size_t i = 0; i < pairs.size(); i++)
	{
		names_set.insert(pairs[i].first);
		names_set.insert(pairs[i].second);
	}

	std::vector v(names_set.begin(), names_set.end());
	std::cout << "file names:" << std::endl;
	for (int i = 0; i < v.size(); i++)
	{
		std::cout << v[i] << std::endl;
	}

	std::cout << "ground truth files with poses" << std::endl;
	for (int i = 1; i < v.size(); i++)
	{
		std::filesystem::path path = folder_with_point_clouds;
		path /= std::string("groundtruth");
		std::string filename = v[i - 1] + "-" + v[i] + ".tfm";
		path /= filename;
		std::cout << path.string() << std::endl;
	}

	size_t sum_points_before_decimation = 0;
	size_t sum_points_after_decimation = 0;

	Eigen::Affine3d m_incremental = Eigen::Affine3d::Identity();
	for (int i = 0; i < v.size(); i++)
	{
		PointCloud pc;
		pc.file_name = v[i] + ".ply";
		// pc.m_pose = m_incremental;

		if (i > 0)
		{
			std::filesystem::path path = folder_with_point_clouds;
			path /= std::string("groundtruth");
			std::string filename = v[i] + "-" + v[i - 1] + ".tfm";
			path /= filename;
			Eigen::Affine3d m_increment;
			if (load_pose_ETH(path.string(), m_increment))
			{
				std::cout << "matrix from file: " << path.string() << std::endl;
				std::cout << m_increment.matrix() << std::endl;
				m_incremental = m_incremental * m_increment;
			}
		}
		pc.m_pose = m_incremental;

		pc.m_initial_pose = pc.m_pose;
		pc.pose = pose_tait_bryan_from_affine_matrix(pc.m_pose);
		pc.gui_translation[0] = pc.pose.px;
		pc.gui_translation[1] = pc.pose.py;
		pc.gui_translation[2] = pc.pose.pz;
		pc.gui_rotation[0] = rad2deg(pc.pose.om);
		pc.gui_rotation[1] = rad2deg(pc.pose.fi);
		pc.gui_rotation[2] = rad2deg(pc.pose.ka);

		if (!pc.load(folder_with_point_clouds + "/" + pc.file_name))
		{
			point_clouds.clear();
			std::cout << "problem with file '" << folder_with_point_clouds + "/" + pc.file_name << "'" << std::endl;
			return false;
		}
		if (decimation)
		{
			std::cout << "point cloud size before decimation: " << pc.points_local.size() << std::endl;
			sum_points_before_decimation += pc.points_local.size();
			pc.decimate(bucket_x, bucket_y, bucket_z);
			sum_points_after_decimation += pc.points_local.size();
			std::cout << "point cloud size after decimation: " << pc.points_local.size() << std::endl;
		}
		point_clouds.push_back(pc);
	}
	infile.close();
	folder_name = folder_with_point_clouds;

	std::cout << "sum_points before/after decimation: "
			  << sum_points_before_decimation << " / "
			  << sum_points_after_decimation << std::endl;
	print_point_cloud_dimension();
	return true;
}

#if WITH_GUI == 1
void PointClouds::draw_grids(bool xz_grid_10x10, bool xz_grid_1x1, bool xz_grid_01x01,
							 bool yz_grid_10x10, bool yz_grid_1x1, bool yz_grid_01x01,
							 bool xy_grid_10x10, bool xy_grid_1x1, bool xy_grid_01x01, PointClouds::PointCloudDimensions dims)
{
	float step;

	if (xz_grid_10x10)
	{
		step = 10.0f;
		float x_min = std::floor(dims.x_min / step) * step;
		float y_min = std::floor(dims.y_min / step) * step;
		float z_min = std::floor(dims.z_min / step) * step;
		float x_max = std::ceil(dims.x_max / step) * step;
		float y_max = std::ceil(dims.y_max / step) * step;
		float z_max = std::ceil(dims.z_max / step) * step;

		glColor3f(0.7, 0.7, 0.7);
		glBegin(GL_LINES);
		for (float x = x_min; x <= x_max; x += step)
		{
			glVertex3f(x, 0.0f, z_min);
			glVertex3f(x, 0.0f, z_max);
		}

		for (float z = z_min; z <= z_max; z += step)
		{
			glVertex3f(x_min, 0, z);
			glVertex3f(x_max, 0, z);
		}
		glEnd();
	}

	if (xz_grid_1x1)
	{
		step = 1.0f;
		float x_min = std::floor(dims.x_min / step) * step;
		float y_min = std::floor(dims.y_min / step) * step;
		float z_min = std::floor(dims.z_min / step) * step;
		float x_max = std::ceil(dims.x_max / step) * step;
		float y_max = std::ceil(dims.y_max / step) * step;
		float z_max = std::ceil(dims.z_max / step) * step;
		glColor3f(0.3, 0.3, 0.3);
		glBegin(GL_LINES);
		for (float x = x_min; x <= x_max; x += step)
		{
			glVertex3f(x, 0.0f, z_min);
			glVertex3f(x, 0.0f, z_max);
		}

		for (float z = z_min; z <= z_max; z += step)
		{
			glVertex3f(x_min, 0, z);
			glVertex3f(x_max, 0, z);
		}
		glEnd();
	}

	if (xz_grid_01x01)
	{
		step = 0.1f;
		float x_min = std::floor(dims.x_min / step) * step;
		float y_min = std::floor(dims.y_min / step) * step;
		float z_min = std::floor(dims.z_min / step) * step;
		float x_max = std::ceil(dims.x_max / step) * step;
		float y_max = std::ceil(dims.y_max / step) * step;
		float z_max = std::ceil(dims.z_max / step) * step;
		glColor3f(0.3, 0.3, 0.3);
		glBegin(GL_LINES);
		for (float x = x_min; x <= x_max; x += step)
		{
			glVertex3f(x, 0.0f, z_min);
			glVertex3f(x, 0.0f, z_max);
		}

		for (float z = z_min; z <= z_max; z += step)
		{
			glVertex3f(x_min, 0, z);
			glVertex3f(x_max, 0, z);
		}
		glEnd();
	}

	if (yz_grid_10x10)
	{
		step = 10.0f;
		float x_min = std::floor(dims.x_min / step) * step;
		float y_min = std::floor(dims.y_min / step) * step;
		float z_min = std::floor(dims.z_min / step) * step;
		float x_max = std::ceil(dims.x_max / step) * step;
		float y_max = std::ceil(dims.y_max / step) * step;
		float z_max = std::ceil(dims.z_max / step) * step;
		glColor3f(0.7, 0.7, 0.7);
		glBegin(GL_LINES);
		for (float y = y_min; y <= y_max; y += step)
		{
			glVertex3f(0.0f, y, z_min);
			glVertex3f(0.0f, y, z_max);
		}

		for (float z = z_min; z <= z_max; z += step)
		{
			glVertex3f(0, y_min, z);
			glVertex3f(0, y_max, z);
		}
		glEnd();
	}

	if (yz_grid_1x1)
	{
		step = 1.0f;
		float x_min = std::floor(dims.x_min / step) * step;
		float y_min = std::floor(dims.y_min / step) * step;
		float z_min = std::floor(dims.z_min / step) * step;
		float x_max = std::ceil(dims.x_max / step) * step;
		float y_max = std::ceil(dims.y_max / step) * step;
		float z_max = std::ceil(dims.z_max / step) * step;
		glColor3f(0.3, 0.3, 0.3);
		glBegin(GL_LINES);
		for (float y = y_min; y <= y_max; y += step)
		{
			glVertex3f(0.0f, y, z_min);
			glVertex3f(0.0f, y, z_max);
		}

		for (float z = z_min; z <= z_max; z += step)
		{
			glVertex3f(0, y_min, z);
			glVertex3f(0, y_max, z);
		}
		glEnd();
	}

	if (yz_grid_01x01)
	{
		step = 0.1f;
		float x_min = std::floor(dims.x_min / step) * step;
		float y_min = std::floor(dims.y_min / step) * step;
		float z_min = std::floor(dims.z_min / step) * step;
		float x_max = std::ceil(dims.x_max / step) * step;
		float y_max = std::ceil(dims.y_max / step) * step;
		float z_max = std::ceil(dims.z_max / step) * step;
		glColor3f(0.3, 0.3, 0.3);
		glBegin(GL_LINES);
		for (float y = y_min; y <= y_max; y += step)
		{
			glVertex3f(0.0f, y, z_min);
			glVertex3f(0.0f, y, z_max);
		}

		for (float z = z_min; z <= z_max; z += step)
		{
			glVertex3f(0, y_min, z);
			glVertex3f(0, y_max, z);
		}
		glEnd();
	}

	if (xy_grid_10x10)
	{
		step = 10.0f;
		float x_min = std::floor(dims.x_min / step) * step;
		float y_min = std::floor(dims.y_min / step) * step;
		float z_min = std::floor(dims.z_min / step) * step;
		float x_max = std::ceil(dims.x_max / step) * step;
		float y_max = std::ceil(dims.y_max / step) * step;
		float z_max = std::ceil(dims.z_max / step) * step;
		glColor3f(0.7, 0.7, 0.7);
		glBegin(GL_LINES);
		for (float x = x_min; x <= x_max; x += step)
		{
			glVertex3f(x, y_min, 0.0);
			glVertex3f(x, y_max, 0.0);
		}

		for (float y = y_min; y <= y_max; y += step)
		{
			glVertex3f(x_min, y, 0.0);
			glVertex3f(x_max, y, 0.0);
		}
		glEnd();
	}

	if (xy_grid_1x1)
	{
		step = 1.0f;
		float x_min = std::floor(dims.x_min / step) * step;
		float y_min = std::floor(dims.y_min / step) * step;
		float z_min = std::floor(dims.z_min / step) * step;
		float x_max = std::ceil(dims.x_max / step) * step;
		float y_max = std::ceil(dims.y_max / step) * step;
		float z_max = std::ceil(dims.z_max / step) * step;
		glColor3f(0.3, 0.3, 0.3);
		glBegin(GL_LINES);
		for (float x = x_min; x <= x_max; x += step)
		{
			glVertex3f(x, y_min, 0.0);
			glVertex3f(x, y_max, 0.0);
		}

		for (float y = y_min; y <= y_max; y += step)
		{
			glVertex3f(x_min, y, 0.0);
			glVertex3f(x_max, y, 0.0);
		}
		glEnd();
	}

	if (xy_grid_01x01)
	{
		step = 0.1f;
		float x_min = std::floor(dims.x_min / step) * step;
		float y_min = std::floor(dims.y_min / step) * step;
		float z_min = std::floor(dims.z_min / step) * step;
		float x_max = std::ceil(dims.x_max / step) * step;
		float y_max = std::ceil(dims.y_max / step) * step;
		float z_max = std::ceil(dims.z_max / step) * step;
		glColor3f(0.3, 0.3, 0.3);
		glBegin(GL_LINES);
		for (float x = x_min; x <= x_max; x += step)
		{
			glVertex3f(x, y_min, 0.0);
			glVertex3f(x, y_max, 0.0);
		}

		for (float y = y_min; y <= y_max; y += step)
		{
			glVertex3f(x_min, y, 0.0);
			glVertex3f(x_max, y, 0.0);
		}
		glEnd();
	}
}

void PointClouds::render(const ObservationPicking &observation_picking, int viewer_decmiate_point_cloud, bool xz_intersection, bool yz_intersection, bool xy_intersection,
						 bool xz_grid_10x10, bool xz_grid_1x1, bool xz_grid_01x01,
						 bool yz_grid_10x10, bool yz_grid_1x1, bool yz_grid_01x01,
						 bool xy_grid_10x10, bool xy_grid_1x1, bool xy_grid_01x01,
						 double intersection_width, PointClouds::PointCloudDimensions dims)
{
	// Draw grids once for the scene
	if (xz_grid_10x10 || xz_grid_1x1 || xz_grid_01x01 ||
		yz_grid_10x10 || yz_grid_1x1 || yz_grid_01x01 ||
		xy_grid_10x10 || xy_grid_1x1 || xy_grid_01x01)
		draw_grids(xz_grid_10x10, xz_grid_1x1, xz_grid_01x01,
				   yz_grid_10x10, yz_grid_1x1, yz_grid_01x01,
				   xy_grid_10x10, xy_grid_1x1, xy_grid_01x01, dims);

	// Render each point cloud (points + trajectories)
	for (auto &p : point_clouds)
	{
		p.render(this->show_with_initial_pose, observation_picking, viewer_decmiate_point_cloud,
				 xz_intersection, yz_intersection, xy_intersection, intersection_width, show_imu_to_lio_diff);
	}
}
#endif

bool PointClouds::save_poses(const std::string file_name, bool is_subsession)
{
	int number_pc = 0;
	if (!is_subsession)
	{
		number_pc = this->point_clouds.size();
	}
	else
	{
		for (size_t i = 0; i < this->point_clouds.size(); i++)
		{
			if (this->point_clouds[i].visible)
			{
				number_pc++;
			}
		}
	}

	std::ofstream outfile;
	outfile.open(file_name);
	if (!outfile.good())
	{
		std::cout << "can not save file: " << file_name << std::endl;
		std::cout << "if you can see only '' it means there is no filename assigned to poses, please read manual or ask for support" << std::endl;
		std::cout << "To assign filename to poses please use following two buttons in multi_view_tls_registration_step_2" << std::endl;
		std::cout << "1: update initial poses from RESSO file" << std::endl;
		std::cout << "2: update poses from RESSO file" << std::endl;
		return false;
	}

	outfile << number_pc << std::endl;
	for (size_t i = 0; i < this->point_clouds.size(); i++)
	{
		if (!is_subsession)
		{
			outfile << std::filesystem::path(point_clouds[i].file_name).filename().string() << std::endl;
			outfile << this->point_clouds[i].m_pose(0, 0) << " " << this->point_clouds[i].m_pose(0, 1) << " " << this->point_clouds[i].m_pose(0, 2) << " " << this->point_clouds[i].m_pose(0, 3) << std::endl;
			outfile << this->point_clouds[i].m_pose(1, 0) << " " << this->point_clouds[i].m_pose(1, 1) << " " << this->point_clouds[i].m_pose(1, 2) << " " << this->point_clouds[i].m_pose(1, 3) << std::endl;
			outfile << this->point_clouds[i].m_pose(2, 0) << " " << this->point_clouds[i].m_pose(2, 1) << " " << this->point_clouds[i].m_pose(2, 2) << " " << this->point_clouds[i].m_pose(2, 3) << std::endl;
			outfile << "0 0 0 1" << std::endl;
		}
		else
		{
			if (this->point_clouds[i].visible)
			{
				outfile << std::filesystem::path(point_clouds[i].file_name).filename().string() << std::endl;
				outfile << this->point_clouds[i].m_pose(0, 0) << " " << this->point_clouds[i].m_pose(0, 1) << " " << this->point_clouds[i].m_pose(0, 2) << " " << this->point_clouds[i].m_pose(0, 3) << std::endl;
				outfile << this->point_clouds[i].m_pose(1, 0) << " " << this->point_clouds[i].m_pose(1, 1) << " " << this->point_clouds[i].m_pose(1, 2) << " " << this->point_clouds[i].m_pose(1, 3) << std::endl;
				outfile << this->point_clouds[i].m_pose(2, 0) << " " << this->point_clouds[i].m_pose(2, 1) << " " << this->point_clouds[i].m_pose(2, 2) << " " << this->point_clouds[i].m_pose(2, 3) << std::endl;
				outfile << "0 0 0 1" << std::endl;
			}
		}
	}
	outfile.close();

	return true;
}

bool PointClouds::save_scans()
{
	for (size_t i = 0; i < this->point_clouds.size(); i++)
	{
		if (!this->point_clouds[i].save_as_global(this->out_folder_name))
		{
			std::cout << "problem with saving to folder: " << this->out_folder_name << std::endl;
		}
	}
	return true;
}

void PointClouds::show_all()
{
	for (auto &pc : point_clouds)
	{
		pc.visible = true;
	}
}

void PointClouds::show_all_from_range(int index_begin, int index_end)
{
	if (index_end > index_begin)
	{
		for (int i = 0; i < point_clouds.size(); i++)
		{
			point_clouds[i].visible = false;
		}
		for (int i = 0; i < point_clouds.size(); i++)
		{
			if (i >= index_begin && i <= index_end)
			{
				point_clouds[i].visible = true;
			}
		}
	}
}

void PointClouds::hide_all()
{
	for (auto &pc : point_clouds)
	{
		pc.visible = false;
	}
}

double get_mean_uncertainty_xyz_impact6x6(std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> &uncertainty_before,
										  std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> &uncertainty_after)
{
	double mui = 0.0;

	double sum_before = 0.0;
	double sum_after = 0.0;

	for (size_t i = 0; i < uncertainty_before.size(); i++)
	{
		sum_before += ((uncertainty_before[i].coeffRef(0, 0) + uncertainty_before[i].coeffRef(1, 1) + uncertainty_before[i].coeffRef(2, 2)) / 3.0);
		sum_after += ((uncertainty_after[i].coeffRef(0, 0) + uncertainty_after[i].coeffRef(1, 1) + uncertainty_after[i].coeffRef(2, 2)) / 3.0);
	}
	sum_before /= uncertainty_before.size();
	sum_after /= uncertainty_before.size();

	mui = sum_before / sum_after;

	return mui;
}

double get_mean_uncertainty_xyz_impact7x7(std::vector<Eigen::Matrix<double, 7, 7, Eigen::RowMajor>> &uncertainty_before,
										  std::vector<Eigen::Matrix<double, 7, 7, Eigen::RowMajor>> &uncertainty_after)
{
	double mui = 0.0;

	double sum_before = 0.0;
	double sum_after = 0.0;

	for (size_t i = 0; i < uncertainty_before.size(); i++)
	{
		sum_before += ((uncertainty_before[i].coeffRef(0, 0) + uncertainty_before[i].coeffRef(1, 1) + uncertainty_before[i].coeffRef(2, 2)) / 3.0);
		sum_after += ((uncertainty_after[i].coeffRef(0, 0) + uncertainty_after[i].coeffRef(1, 1) + uncertainty_after[i].coeffRef(2, 2)) / 3.0);
	}
	sum_before /= uncertainty_before.size();
	sum_after /= uncertainty_before.size();

	mui = sum_before / sum_after;

	return mui;
}

double get_mean_uncertainty_xyz_impact(std::vector<Eigen::SparseMatrix<double>> &uncertainty_before, std::vector<Eigen::SparseMatrix<double>> &uncertainty_after)
{
	double mui = 0.0;

	double sum_before = 0.0;
	double sum_after = 0.0;

	for (size_t i = 0; i < uncertainty_before.size(); i++)
	{
		sum_before += ((uncertainty_before[i].coeffRef(0, 0) + uncertainty_before[i].coeffRef(1, 1) + uncertainty_before[i].coeffRef(2, 2)) / 3.0);
		sum_after += ((uncertainty_after[i].coeffRef(0, 0) + uncertainty_after[i].coeffRef(1, 1) + uncertainty_after[i].coeffRef(2, 2)) / 3.0);
	}
	sum_before /= uncertainty_before.size();
	sum_after /= uncertainty_before.size();

	mui = sum_before / sum_after;

	return mui;
}

bool PointClouds::load_pose_ETH(const std::string &fn, Eigen::Affine3d &m_increment)
{
	m_increment = Eigen::Affine3d::Identity();

	std::ifstream infile(fn);
	if (!infile.good())
	{
		std::cout << "problem with file: '" << fn << "'" << std::endl;
		return false;
	}

	std::string line;
	std::getline(infile, line);
	std::istringstream iss(line);
	double r11, r12, r13, r21, r22, r23, r31, r32, r33;
	double t14, t24, t34;
	iss >> r11 >> r12 >> r13 >> t14;

	std::getline(infile, line);
	std::istringstream iss1(line);
	iss1 >> r21 >> r22 >> r23 >> t24;

	std::getline(infile, line);
	std::istringstream iss2(line);
	iss2 >> r31 >> r32 >> r33 >> t34;

	m_increment(0, 0) = r11;
	m_increment(0, 1) = r12;
	m_increment(0, 2) = r13;
	m_increment(0, 3) = t14;

	m_increment(1, 0) = r21;
	m_increment(1, 1) = r22;
	m_increment(1, 2) = r23;
	m_increment(1, 3) = t24;

	m_increment(2, 0) = r31;
	m_increment(2, 1) = r32;
	m_increment(2, 2) = r33;
	m_increment(2, 3) = t34;

	infile.close();
	return true;
}

bool PointClouds::load_pc(PointCloud &pc, std::string input_file_name, bool load_cache_mode)
{
	return pc.load_pc(input_file_name, load_cache_mode);
#if 0
	laszip_POINTER laszip_reader;
	if (laszip_create(&laszip_reader))
	{
		fprintf(stderr, ":DLL ERROR: creating laszip reader\n");
		/*PointCloud pc;
		pc.m_pose = Eigen::Affine3d::Identity();
		pc.m_initial_pose = pc.m_pose;
		pc.pose = pose_tait_bryan_from_affine_matrix(pc.m_pose);
		pc.gui_translation[0] = pc.pose.px;
		pc.gui_translation[1] = pc.pose.py;
		pc.gui_translation[2] = pc.pose.pz;
		pc.gui_rotation[0] = rad2deg(pc.pose.om);
		pc.gui_rotation[1] = rad2deg(pc.pose.fi);
		pc.gui_rotation[2] = rad2deg(pc.pose.ka);
		pc.file_name = input_file_names[i];
		point_clouds.push_back(pc);*/
		return false;
	}

	laszip_BOOL is_compressed = 0;
	if (laszip_open_reader(laszip_reader, input_file_name.c_str(), &is_compressed))
	{
		fprintf(stderr, ":DLL ERROR: opening laszip reader for '%s'\n", input_file_name.c_str());
		/*PointCloud pc;
		pc.m_pose = Eigen::Affine3d::Identity();
		pc.m_initial_pose = pc.m_pose;
		pc.pose = pose_tait_bryan_from_affine_matrix(pc.m_pose);
		pc.gui_translation[0] = pc.pose.px;
		pc.gui_translation[1] = pc.pose.py;
		pc.gui_translation[2] = pc.pose.pz;
		pc.gui_rotation[0] = rad2deg(pc.pose.om);
		pc.gui_rotation[1] = rad2deg(pc.pose.fi);
		pc.gui_rotation[2] = rad2deg(pc.pose.ka);
		pc.file_name = input_file_names[i];
		point_clouds.push_back(pc);*/
		return false;
	}
	laszip_header *header;

	if (laszip_get_header_pointer(laszip_reader, &header))
	{
		fprintf(stderr, ":DLL ERROR: getting header pointer from laszip reader\n");
		/*PointCloud pc;
		pc.m_pose = Eigen::Affine3d::Identity();
		pc.m_initial_pose = pc.m_pose;
		pc.pose = pose_tait_bryan_from_affine_matrix(pc.m_pose);
		pc.gui_translation[0] = pc.pose.px;
		pc.gui_translation[1] = pc.pose.py;
		pc.gui_translation[2] = pc.pose.pz;
		pc.gui_rotation[0] = rad2deg(pc.pose.om);
		pc.gui_rotation[1] = rad2deg(pc.pose.fi);
		pc.gui_rotation[2] = rad2deg(pc.pose.ka);
		pc.file_name = input_file_names[i];
		point_clouds.push_back(pc);*/
		return false;
	}

	// fprintf(stderr, "file '%s' contains %u points\n", input_file_name.c_str(), header->number_of_point_records);

	laszip_point *point;
	if (laszip_get_point_pointer(laszip_reader, &point))
	{
		fprintf(stderr, ":DLL ERROR: getting point pointer from laszip reader\n");
		return false;
	}

	pc.m_pose = Eigen::Affine3d::Identity();
	pc.m_initial_pose = pc.m_pose;
	pc.pose = pose_tait_bryan_from_affine_matrix(pc.m_pose);
	pc.gui_translation[0] = static_cast<float>(pc.pose.px);
	pc.gui_translation[1] = static_cast<float>(pc.pose.py);
	pc.gui_translation[2] = static_cast<float>(pc.pose.pz);
	pc.gui_rotation[0] = rad2deg(pc.pose.om);
	pc.gui_rotation[1] = rad2deg(pc.pose.fi);
	pc.gui_rotation[2] = rad2deg(pc.pose.ka);

	/*for (int j = 0; j < header->number_of_point_records; j++)
	{
		if (laszip_read_point(laszip_reader))
		{
			fprintf(stderr, ":DLL ERROR: reading point %u\n", j);
			laszip_close_reader(laszip_reader);
			return true;
			// continue;
		}

		LAZPoint p;
		p.x = header->x_offset + header->x_scale_factor * static_cast<double>(point->X);
		p.y = header->y_offset + header->y_scale_factor * static_cast<double>(point->Y);
		p.z = header->z_offset + header->z_scale_factor * static_cast<double>(point->Z);
		p.timestamp = point->gps_time;

		Eigen::Vector3d pp(p.x, p.y, p.z);
		pc.points_local.push_back(pp);
		pc.intensities.push_back(point->intensity);
		pc.timestamps.push_back(p.timestamp);
	}*/

	laszip_I64 npoints = (header->number_of_point_records ? header->number_of_point_records : header->extended_number_of_point_records);

	std::cout << (is_compressed ? "" : "un") << "compressed file '" << (std::filesystem::path(input_file_name).filename().string()) << "' contains " << npoints << " points" << std::endl;

	laszip_I64 p_count = 0;

	while (p_count < npoints)
	{
		if (laszip_read_point(laszip_reader))
		{
			fprintf(stderr, "DLL ERROR: reading point %I64d\n", p_count);
			laszip_close_reader(laszip_reader);
			return false;
		}

		LAZPoint p;
		p.x = header->x_offset + header->x_scale_factor * static_cast<double>(point->X);
		p.y = header->y_offset + header->y_scale_factor * static_cast<double>(point->Y);
		p.z = header->z_offset + header->z_scale_factor * static_cast<double>(point->Z);
		p.timestamp = point->gps_time;

		Eigen::Vector3d pp(p.x, p.y, p.z);
		pc.points_local.push_back(pp);
		pc.intensities.push_back(point->intensity);
		pc.timestamps.push_back(p.timestamp);

		// Eigen::Vector3d color(
		//	static_cast<uint8_t>(0xFFU * ((point->rgb[0] > 0) ? static_cast<float>(point->rgb[0]) / static_cast<float>(0xFFFFU) : 1.0f)) / 256.0,
		//	static_cast<uint8_t>(0xFFU * ((point->rgb[1] > 0) ? static_cast<float>(point->rgb[1]) / static_cast<float>(0xFFFFU) : 1.0f)) / 256.0,
		//	static_cast<uint8_t>(0xFFU * ((point->rgb[2] > 0) ? static_cast<float>(point->rgb[2]) / static_cast<float>(0xFFFFU) : 1.0f)) / 256.0);

		Eigen::Vector3d color(
			static_cast<float>(point->rgb[0]) / 256.0,
			static_cast<float>(point->rgb[1]) / 256.0,
			static_cast<float>(point->rgb[2]) / 256.0);

		// std::cout << point->rgb[0] << " " << point->rgb[1] << " " << point->rgb[2] << std::endl;

		pc.colors.push_back(color);

		p_count++;
	}

	laszip_close_reader(laszip_reader);
	// laszip_clean(laszip_reader);
	// laszip_destroy(laszip_reader);

	return true;

	#endif
}

bool PointClouds::load_whu_tls(std::vector<std::string> input_file_names, bool is_decimate, double bucket_x, double bucket_y, double bucket_z, bool calculate_offset, bool load_cache_mode)
{
	const auto start = std::chrono::system_clock::now();
	std::vector<PointCloud> point_clouds_nodata;

	point_clouds_nodata.resize(input_file_names.size()); // pre-allocate

	for (size_t i = 0; i < input_file_names.size(); i++)
	{
		std::cout << "Loading file " << i + 1 << "/" << input_file_names.size() << " (" << (std::filesystem::path(input_file_names[i]).filename().string()) << "). ";

		auto &pc = point_clouds_nodata[i]; // reference directly to vector slot

		pc.file_name = input_file_names[i];
		//---
		std::filesystem::path path = std::filesystem::path(pc.file_name);

		auto only_fn = (path.filename()).stem();

		std::vector<std::string> strs;
		std::string line = only_fn.string();
		split(line, '_', strs);

		//--

		auto trj_path = std::filesystem::path(input_file_names[i]).parent_path();
		std::filesystem::path trajectorypath(pc.file_name);
		// std::string fn = (trajectorypath.filename().stem()).string();

		trajectorypath.remove_filename();

		// std::string trj_fn = "trajectory_lio_" + std::to_string(i) + ".csv";
		std::string trj_fn = "trajectory_lio_" + strs[strs.size() - 1] + ".csv";

		// fn.replace(0, 9, "trajectory_lio_");
		// std::string trajectory_filename = (fn + ".csv");
		// trj_path /= trajectory_filename;

		trj_path /= trj_fn;

		std::cout << "From trajectory file (" << (std::filesystem::path(trj_path).filename().string()) << ")";

		if (std::filesystem::exists(trj_path))
		{
			std::cout << " loading.. ";

			std::vector<PointCloud::LocalTrajectoryNode> local_trajectory;
			//
			std::ifstream infile(trj_path.string());
			if (!infile.good())
			{
				std::cout << "problem with file: '" << trj_path.string() << "'" << std::endl;
				return false;
			}

			std::string s;
			getline(infile, s); // csv header
			while (!infile.eof())
			{
				getline(infile, s);
				std::vector<std::string> strs;
				split(s, ' ', strs);

				if (strs.size() == 13)
				{
					PointCloud::LocalTrajectoryNode ltn;
					std::istringstream(strs[0]) >> ltn.timestamps.first;
					std::istringstream(strs[1]) >> ltn.m_pose(0, 0);
					std::istringstream(strs[2]) >> ltn.m_pose(0, 1);
					std::istringstream(strs[3]) >> ltn.m_pose(0, 2);
					std::istringstream(strs[4]) >> ltn.m_pose(0, 3);
					std::istringstream(strs[5]) >> ltn.m_pose(1, 0);
					std::istringstream(strs[6]) >> ltn.m_pose(1, 1);
					std::istringstream(strs[7]) >> ltn.m_pose(1, 2);
					std::istringstream(strs[8]) >> ltn.m_pose(1, 3);
					std::istringstream(strs[9]) >> ltn.m_pose(2, 0);
					std::istringstream(strs[10]) >> ltn.m_pose(2, 1);
					std::istringstream(strs[11]) >> ltn.m_pose(2, 2);
					std::istringstream(strs[12]) >> ltn.m_pose(2, 3);

					ltn.timestamps.second = 0.0;
					ltn.imu_om_fi_ka = {0, 0, 0};
					local_trajectory.push_back(ltn);
				}

				if (strs.size() == 14)
				{
					PointCloud::LocalTrajectoryNode ltn;
					std::istringstream(strs[0]) >> ltn.timestamps.first;
					std::istringstream(strs[1]) >> ltn.m_pose(0, 0);
					std::istringstream(strs[2]) >> ltn.m_pose(0, 1);
					std::istringstream(strs[3]) >> ltn.m_pose(0, 2);
					std::istringstream(strs[4]) >> ltn.m_pose(0, 3);
					std::istringstream(strs[5]) >> ltn.m_pose(1, 0);
					std::istringstream(strs[6]) >> ltn.m_pose(1, 1);
					std::istringstream(strs[7]) >> ltn.m_pose(1, 2);
					std::istringstream(strs[8]) >> ltn.m_pose(1, 3);
					std::istringstream(strs[9]) >> ltn.m_pose(2, 0);
					std::istringstream(strs[10]) >> ltn.m_pose(2, 1);
					std::istringstream(strs[11]) >> ltn.m_pose(2, 2);
					std::istringstream(strs[12]) >> ltn.m_pose(2, 3);
					std::istringstream(strs[13]) >> ltn.timestamps.second;
					ltn.imu_om_fi_ka = {0, 0, 0};
					local_trajectory.push_back(ltn);
				}

				if (strs.size() == 17)
				{
					PointCloud::LocalTrajectoryNode ltn;
					std::istringstream(strs[0]) >> ltn.timestamps.first;
					std::istringstream(strs[1]) >> ltn.m_pose(0, 0);
					std::istringstream(strs[2]) >> ltn.m_pose(0, 1);
					std::istringstream(strs[3]) >> ltn.m_pose(0, 2);
					std::istringstream(strs[4]) >> ltn.m_pose(0, 3);
					std::istringstream(strs[5]) >> ltn.m_pose(1, 0);
					std::istringstream(strs[6]) >> ltn.m_pose(1, 1);
					std::istringstream(strs[7]) >> ltn.m_pose(1, 2);
					std::istringstream(strs[8]) >> ltn.m_pose(1, 3);
					std::istringstream(strs[9]) >> ltn.m_pose(2, 0);
					std::istringstream(strs[10]) >> ltn.m_pose(2, 1);
					std::istringstream(strs[11]) >> ltn.m_pose(2, 2);
					std::istringstream(strs[12]) >> ltn.m_pose(2, 3);
					std::istringstream(strs[13]) >> ltn.timestamps.second;
					std::istringstream(strs[14]) >> ltn.imu_om_fi_ka.x();
					std::istringstream(strs[15]) >> ltn.imu_om_fi_ka.y();
					std::istringstream(strs[16]) >> ltn.imu_om_fi_ka.z();

					local_trajectory.push_back(ltn);
				}
			}

			std::cout << local_trajectory.size() << " local nodes" << std::endl;
			infile.close();

			pc.local_trajectory = local_trajectory;
		}
		else
			std::cout << "trajectory path: '" << trj_path.string() << "' does not exist" << std::endl;
	}

	//// load actual pointclouds
	point_clouds.resize(point_clouds_nodata.size());

	std::transform(std::execution::par_unseq, std::begin(point_clouds_nodata), std::end(point_clouds_nodata), std::begin(point_clouds), [&](auto &pc)
		{
			if(!load_cache_mode)
			{
				if (load_pc(pc, pc.file_name, load_cache_mode))
				{
					if (is_decimate && pc.points_local.size() > 0)
					{
						std::cout << "start downsampling.." << std::endl;

						size_t sum_points_before_decimation = pc.points_local.size();
						pc.decimate(bucket_x, bucket_y, bucket_z);
						size_t sum_points_after_decimation = pc.points_local.size();

						std::cout << "downsampling finished. sum_points before/after decimation: "
							<< sum_points_before_decimation << " / "
							<< sum_points_after_decimation << std::endl;
					}
				}
			}
		
			return pc;
		});

	//calculate average position of a subset of points from all clouds to center the point clouds around the origin
	if (calculate_offset)
	{
		int num = 0;
		for (int i = 0; i < point_clouds.size(); i++)
		{
			for (int j = 0; j < point_clouds[i].points_local.size(); j += 1000)
			{
				this->offset += point_clouds[i].points_local[j];
				num++;
			}
		}
		this->offset /= num;
	}

	//recenter point clouds around calculated offset
	if (!this->offset.isZero())
	{
		for (int i = 0; i < point_clouds.size(); i++)
			for (int j = 0; j < point_clouds[i].points_local.size(); j++)
				point_clouds[i].points_local[j] -= this->offset;
	}

	print_point_cloud_dimension();
	const auto end = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed_seconds = end - start;
	std::cout << "Load time: " << std::fixed << std::setprecision(1) << elapsed_seconds.count() << " [s]" << std::endl;
	return true;
}

PointClouds::PointCloudDimensions PointClouds::compute_point_cloud_dimension() const
{
	PointCloudDimensions dim{
		std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest(),
		std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest(),
		std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest(),
		0, 0, 0};

	int sum = 0;
	for (const auto &pc : this->point_clouds)
	{
		for (const auto &p : pc.points_local)
		{
			auto pt = pc.m_initial_pose * p;
			dim.x_min = std::min(dim.x_min, pt.x());
			dim.x_max = std::max(dim.x_max, pt.x());
			dim.y_min = std::min(dim.y_min, pt.y());
			dim.y_max = std::max(dim.y_max, pt.y());
			dim.z_min = std::min(dim.z_min, pt.z());
			dim.z_max = std::max(dim.z_max, pt.z());

			sum ++;
		}
	}

	if (sum == 0){
		dim.length = 0.0;
		dim.width = 0.0;
		dim.height = 0.0;

		dim.x_min = 0.0;
		dim.y_min = 0.0;
		dim.z_min = 0.0;

		dim.x_max = 0.0;
		dim.y_max = 0.0;
		dim.z_max = 0.0;
	}else{
		dim.length = dim.x_max - dim.x_min;
		dim.width = dim.y_max - dim.y_min;
		dim.height = dim.z_max - dim.z_min;
	}
	return dim;
}

void PointClouds::print_point_cloud_dimension()
{
	auto dims = compute_point_cloud_dimension();

	std::cout << "------------------------" << std::endl;

	std::cout << "Coordinates: min / max / size [m]" << std::endl;
	std::cout << "X (length): " << std::fixed << std::setprecision(3) << dims.x_min
			  << " / " << std::fixed << std::setprecision(3) << dims.x_max
			  << " / " << std::fixed << std::setprecision(3) << dims.length << std::endl;
	std::cout << "Y (width) : " << std::fixed << std::setprecision(3) << dims.y_min
			  << " / " << std::fixed << std::setprecision(3) << dims.y_max
			  << " / " << std::fixed << std::setprecision(3) << dims.width << std::endl;
	std::cout << "Z (height): " << std::setprecision(3) << dims.z_min
			  << " / " << std::fixed << std::setprecision(3) << dims.z_max
			  << " / " << std::fixed << std::setprecision(3) << dims.height << std::endl;

	std::cout << "------------------------" << std::endl;
}

bool PointClouds::load_3DTK_tls(std::vector<std::string> input_file_names, bool is_decimate, double bucket_x, double bucket_y, double bucket_z)
{
	this->offset = Eigen::Vector3d(0, 0, 0);
	size_t sum_points_before_decimation = 0;
	size_t sum_points_after_decimation = 0;

	for (size_t i = 0; i < input_file_names.size(); i++)
	{
		std::cout << "loading file " << i + 1 << "/" << input_file_names.size() << " (" << (std::filesystem::path(input_file_names[i]).filename().string()) << ")" << std::endl;
		PointCloud pc;
		pc.m_pose = Eigen::Affine3d::Identity();
		pc.m_initial_pose = pc.m_pose;
		pc.pose = pose_tait_bryan_from_affine_matrix(pc.m_pose);
		pc.gui_translation[0] = pc.pose.px;
		pc.gui_translation[1] = pc.pose.py;
		pc.gui_translation[2] = pc.pose.pz;
		pc.gui_rotation[0] = rad2deg(pc.pose.om);
		pc.gui_rotation[1] = rad2deg(pc.pose.fi);
		pc.gui_rotation[2] = rad2deg(pc.pose.ka);

		std::ifstream f;
		f.open(input_file_names[i]);
		if (f.good())
		{
			std::string s;
			getline(f, s);
			while (!f.eof())
			{
				getline(f, s);
				std::vector<std::string> strs;
				split(s, ' ', strs);

				if (strs.size() >= 3)
				{
					Eigen::Vector3d pp;
					std::istringstream(strs[0]) >> pp.x();
					std::istringstream(strs[1]) >> pp.y();
					std::istringstream(strs[2]) >> pp.z();
					pc.points_local.push_back(pp);
					pc.intensities.push_back(0);
					pc.timestamps.push_back(0);
					sum_points_before_decimation++;
				}
			}
		}
		else
		{
			std::cout << "problem opening file: " << input_file_names[i] << std::endl;
		}
		pc.file_name = input_file_names[i];
		point_clouds.push_back(pc);
	}

	int num = 0;
	for (int i = 0; i < point_clouds.size(); i++)
	{
		for (int j = 0; j < point_clouds[i].points_local.size(); j++)
		{
			this->offset += point_clouds[i].points_local[j];
			num++;
		}
	}
	this->offset /= num;

	for (int i = 0; i < point_clouds.size(); i++)
	{
		for (int j = 0; j < point_clouds[i].points_local.size(); j++)
		{
			point_clouds[i].points_local[j] -= this->offset;
		}
	}

	std::cout << "start downsampling.." << std::endl;
	if (is_decimate)
	{
		for (int i = 0; i < point_clouds.size(); i++)
		{
			sum_points_before_decimation += point_clouds[i].points_local.size();
			point_clouds[i].decimate(bucket_x, bucket_y, bucket_z);
			sum_points_after_decimation += point_clouds[i].points_local.size();
		}
	}
	else
	{
		sum_points_after_decimation = sum_points_before_decimation;
	}
	std::cout << "downsampling finished. sum_points before/after decimation: "
			  << sum_points_before_decimation << " / "
			  << sum_points_after_decimation << std::endl;
	print_point_cloud_dimension();
	return true;
}
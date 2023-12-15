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

bool PointClouds::load(const std::string &folder_with_point_clouds, const std::string &poses_file_name, bool decimation, double bucket_x, double bucket_y, double bucket_z)
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
		point_clouds.push_back(pc);
	}
	infile.close();
	folder_name = folder_with_point_clouds;

	std::cout << "all scans, sum_points_before_decimation: " << sum_points_before_decimation << std::endl;
	std::cout << "all scans, sum_points_after_decimation: " << sum_points_after_decimation << std::endl;
	print_point_cloud_dimention();
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
					std::cout << "-------------------------" << std::endl;
					std::cout << "update pose: " << i << std::endl;
					std::cout << "previous pose: " << std::endl
							  << point_clouds[i].m_pose.matrix() << std::endl;
					std::cout << "current pose: " << std::endl
							  << pcs[j].m_pose.matrix() << std::endl;

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
					std::cout << "-------------------------" << std::endl;
					std::cout << "update pose: " << i << std::endl;
					std::cout << "previous pose: " << std::endl
							  << point_clouds[i].m_pose.matrix() << std::endl;
					std::cout << "current pose: " << std::endl
							  << pcs[j].m_pose.matrix() << std::endl;

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
					std::cout << "-------------------------" << std::endl;
					std::cout << "update pose: " << i << std::endl;
					std::cout << "previous pose: " << std::endl
							  << point_clouds[i].m_initial_pose.matrix() << std::endl;
					std::cout << "current pose: " << std::endl
							  << pcs[j].m_initial_pose.matrix() << std::endl;

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

	std::cout << "Lodaing from file: " << poses_file_name << std::endl;
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

	std::cout << "all scans, sum_points_before_decimation: " << sum_points_before_decimation << std::endl;
	std::cout << "all scans, sum_points_after_decimation: " << sum_points_after_decimation << std::endl;
	print_point_cloud_dimention();
	return true;
}

void PointClouds::render(const ObservationPicking &observation_picking, int viewer_decmiate_point_cloud)
{
	for (auto &p : point_clouds)
	{
		p.render(this->show_with_initial_pose, observation_picking, viewer_decmiate_point_cloud);
	}
}

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
			if (i < index_begin || i > index_end)
			{
				point_clouds[i].visible = false;
			}
			else
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

bool load_pc(PointCloud &pc, std::string input_file_name)
{

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
	std::cout << "compressed : " << is_compressed << std::endl;
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
	fprintf(stderr, "file '%s' contains %u points\n", input_file_name.c_str(), header->number_of_point_records);
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

	for (int j = 0; j < header->number_of_point_records; j++)
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
	}
	laszip_close_reader(laszip_reader);
	// laszip_clean(laszip_reader);
	// laszip_destroy(laszip_reader);

	return true;
}

bool PointClouds::load_whu_tls(std::vector<std::string> input_file_names, bool is_decimate, double bucket_x, double bucket_y, double bucket_z, bool calculate_offset)
{
	const auto start = std::chrono::system_clock::now();
	std::vector<PointCloud> point_clouds_nodata;
	for (size_t i = 0; i < input_file_names.size(); i++)
	{
		std::cout << "loading file: " << input_file_names[i] << " [" << i + 1 << "] of " << input_file_names.size() << std::endl;
		PointCloud pc;

		pc.file_name = input_file_names[i];
		auto trj_path = std::filesystem::path(input_file_names[i]).parent_path();
		std::filesystem::path trajectorypath(pc.file_name);
		std::string fn = (trajectorypath.filename().stem()).string();
		fn.replace(0, 9, "trajectory_lio_");
		std::string trajectory_filename = (fn + ".csv");
		trj_path /= trajectory_filename;

		if (std::filesystem::exists(trj_path))
		{
			std::cout << "loading trajectory from file: " << trj_path << std::endl;

			std::vector<PointCloud::LocalTrajectoryNode> local_trajectory;
			//
			std::ifstream infile(trj_path.string());
			if (!infile.good())
			{
				std::cout << "problem with file: '" << trj_path.string() << "'" << std::endl;
				return false;
			}

			std::string s;
			while (!infile.eof())
			{
				getline(infile, s);
				std::vector<std::string> strs;
				split(s, ' ', strs);

				if (strs.size() == 13)
				{
					PointCloud::LocalTrajectoryNode ltn;
					std::istringstream(strs[0]) >> ltn.timestamp;
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

					local_trajectory.push_back(ltn);
				}
			}

			std::cout << "loaded: [" << local_trajectory.size() << "] local trajectory nodes" << std::endl;
			infile.close();
			///
			pc.local_trajectory = local_trajectory;
		}

		point_clouds_nodata.push_back(pc);
	}

	//// load actual pointclouds
	point_clouds.resize(point_clouds_nodata.size());

	std::transform(std::execution::par_unseq, std::begin(point_clouds_nodata), std::end(point_clouds_nodata), std::begin(point_clouds), [&](auto &pc)
				   {

			if (load_pc(pc, pc.file_name))
			{
				if (is_decimate && pc.points_local.size() > 0)
				{
					std::cout << "start downsampling" << std::endl;

					size_t sum_points_before_decimation = pc.points_local.size();
					pc.decimate(bucket_x, bucket_y, bucket_z);
					size_t sum_points_after_decimation = pc.points_local.size();

					std::cout << "downsampling finished" << std::endl;
					std::cout << "all scans, sum_points_before_decimation: " << sum_points_before_decimation << std::endl;
					std::cout << "all scans, sum_points_after_decimation: " << sum_points_after_decimation << std::endl;
				}
			}
			return pc; });

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

	for (int i = 0; i < point_clouds.size(); i++)
	{
		for (int j = 0; j < point_clouds[i].points_local.size(); j++)
		{
			point_clouds[i].points_local[j] -= this->offset;
		}
	}

	print_point_cloud_dimention();
	const auto end = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed_seconds = end - start;
	std::cout << "Load time :" << elapsed_seconds.count() << std::endl;
	return true;
}

void PointClouds::print_point_cloud_dimention()
{
	double x_min = std::numeric_limits<double>::max();
	double x_max = std::numeric_limits<double>::lowest();
	double y_min = std::numeric_limits<double>::max();
	double y_max = std::numeric_limits<double>::lowest();
	double z_min = std::numeric_limits<double>::max();
	double z_max = std::numeric_limits<double>::lowest();

	for (const auto &pc : this->point_clouds)
	{
		for (const auto &p : pc.points_local)
		{
			auto pt = pc.m_initial_pose * p;
			if (pt.x() < x_min)
			{
				x_min = pt.x();
			}
			if (pt.x() > x_max)
			{
				x_max = pt.x();
			}
			if (pt.y() < y_min)
			{
				y_min = pt.y();
			}
			if (pt.y() > y_max)
			{
				y_max = pt.y();
			}
			if (pt.z() < z_min)
			{
				z_min = pt.z();
			}
			if (pt.z() > z_max)
			{
				z_max = pt.z();
			}
		}
	}

	std::cout << "Bounaries" << std::endl;
	std::cout << "x: " << x_max - x_min << std::endl;
	std::cout << "y: " << y_max - y_min << std::endl;
	std::cout << "z: " << z_max - z_min << std::endl;
}

bool PointClouds::load_3DTK_tls(std::vector<std::string> input_file_names, bool is_decimate, double bucket_x, double bucket_y, double bucket_z)
{
	this->offset = Eigen::Vector3d(0, 0, 0);
	size_t sum_points_before_decimation = 0;
	size_t sum_points_after_decimation = 0;

	for (size_t i = 0; i < input_file_names.size(); i++)
	{
		std::cout << "loading file: " << input_file_names[i] << " [" << i + 1 << "] of " << input_file_names.size() << std::endl;
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
			std::cout << "problem with opening file: " << input_file_names[i] << std::endl;
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

	std::cout << "start downsampling" << std::endl;
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
	std::cout << "downsampling finished" << std::endl;
	std::cout << "all scans, sum_points_before_decimation: " << sum_points_before_decimation << std::endl;
	std::cout << "all scans, sum_points_after_decimation: " << sum_points_after_decimation << std::endl;
	print_point_cloud_dimention();
	return true;
}
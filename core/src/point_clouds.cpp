#include <point_clouds.h>
#include <transformations.h>

#include <iostream>
#include <cstdlib>
#include <sstream>
#include <string>
#include <fstream>
#include <vector>
#include <algorithm>

bool PointClouds::load(const std::string& folder_with_point_clouds, const std::string& poses_file_name)
{
	point_clouds.clear();

	std::ifstream infile(poses_file_name);
	if (!infile.good()) {
		std::cout << "problem with file: '" << poses_file_name << "'" << std::endl;
		return false;
	}
	std::string line;
	std::getline(infile, line);
	std::istringstream iss(line);

	int num_scans;
	iss >> num_scans;

	std::cout << "number of scans: " << num_scans << std::endl;

	for (size_t i = 0; i < num_scans; i++) {
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

		if (!pc.load(folder_with_point_clouds + "/" + pc.file_name)) {
			point_clouds.clear();
			std::cout << "problem with file '" << folder_with_point_clouds + "/" + pc.file_name << "'" << std::endl;
			return false;
		}
		point_clouds.push_back(pc);
	}
	infile.close();
	folder_name = folder_with_point_clouds;


#if 0
	std::ifstream infile(poses_file_name);
	if (!infile.good()) {
		std::cout << "problem with file: '" << poses_file_name << "'" << std::endl;
		return false;
	}
	std::string line;
	std::getline(infile, line);
	while (std::getline(infile, line))
	{
		std::replace(line.begin(), line.end(), ';', ' ');
		std::istringstream iss(line);

		std::string point_cloud_file_name;
		double r11, r12, r13, r21, r22, r23, r31, r32, r33;
		double t14, t24, t34;

		iss >> point_cloud_file_name >> r11 >> r12 >> r13 >> r21 >> r22 >> r23 >> r31 >> r32 >> r33 >> t14 >> t24 >> t34;
		//printf("%s,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", point_cloud_file_name.c_str(), r11, r12, r13, r21, r22, r23, r31, r32, r33, t14, t24, t34);
	
		PointCloud pc;
		pc.file_name = point_cloud_file_name;
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
		pc.pose = pose_tait_bryan_from_affine_matrix(pc.m_pose);
		pc.gui_translation[0] = pc.pose.px;
		pc.gui_translation[1] = pc.pose.py;
		pc.gui_translation[2] = pc.pose.pz;
		pc.gui_rotation[0] = rad2deg(pc.pose.om);
		pc.gui_rotation[1] = rad2deg(pc.pose.fi);
		pc.gui_rotation[2] = rad2deg(pc.pose.ka);
		
		if (!pc.load(folder_with_point_clouds + "/" + pc.file_name)) {
			point_clouds.clear();
			std::cout << "problem with file '" << folder_with_point_clouds + "/" + pc.file_name << "'" << std::endl;
			return false;
		}
		point_clouds.push_back(pc);
	}

	infile.close();
	folder_name = folder_with_point_clouds;
#endif
	return true;
}

/*std::vector<Eigen::Vector3d> PointClouds::load_points(const std::string& point_clouds_file_name)
{
	std::vector<Eigen::Vector3d> points;
	std::ifstream infile(point_clouds_file_name);
	if (!infile.good()) {
		return points;
	}

	std::string line;
	std::getline(infile, line);

	while (std::getline(infile, line))
	{
		std::replace(line.begin(), line.end(), ';', ' ');
		std::istringstream iss(line);

		float x, y, z, intensity;

		iss >> x >> y >> z >> intensity;
	
		Eigen::Vector3d point(x, y, z);
		points.push_back(point);
	}

	return points;
}*/

void PointClouds::render(const ObservationPicking& observation_picking)
{
	for (auto& p : point_clouds) {
		p.render(this->show_with_initial_pose, observation_picking);
	}
}

bool PointClouds::save_poses(const std::string file_name)
{
	std::ofstream outfile;
	outfile.open(file_name);
	if (!outfile.good()) {
		std::cout << "can not save file: " << file_name << std::endl;
		return false;
	}

	outfile << this->point_clouds.size() << std::endl;
	for (size_t i = 0; i < this->point_clouds.size(); i++) {
		outfile << this->point_clouds[i].file_name << std::endl;
		outfile << this->point_clouds[i].m_pose(0, 0) << " " << this->point_clouds[i].m_pose(0, 1) << " " << this->point_clouds[i].m_pose(0, 2) << " " << this->point_clouds[i].m_pose(0, 3) << std::endl;
		outfile << this->point_clouds[i].m_pose(1, 0) << " " << this->point_clouds[i].m_pose(1, 1) << " " << this->point_clouds[i].m_pose(1, 2) << " " << this->point_clouds[i].m_pose(1, 3) << std::endl;
		outfile << this->point_clouds[i].m_pose(2, 0) << " " << this->point_clouds[i].m_pose(2, 1) << " " << this->point_clouds[i].m_pose(2, 2) << " " << this->point_clouds[i].m_pose(2, 3) << std::endl;
		outfile << "0 0 0 1" << std::endl;
	}
	outfile.close();

	return true;
}

#if 0
bool PointClouds::save_poses()
{
	std::ofstream outfile;
	outfile.open(this->out_poses_file_name);
	if (!outfile.good()) {
		return false;
	}
	
	outfile << this->point_clouds.size() << std::endl;
	for (size_t i = 0; i < this->point_clouds.size(); i++) {
		outfile << this->point_clouds[i].file_name << std::endl;
		outfile << this->point_clouds[i].m_pose(0, 0) << " " << this->point_clouds[i].m_pose(0, 1) << " " << this->point_clouds[i].m_pose(0, 2) << " " << this->point_clouds[i].m_pose(0, 3) << std::endl;
		outfile << this->point_clouds[i].m_pose(1, 0) << " " << this->point_clouds[i].m_pose(1, 1) << " " << this->point_clouds[i].m_pose(1, 2) << " " << this->point_clouds[i].m_pose(1, 3) << std::endl;
		outfile << this->point_clouds[i].m_pose(2, 0) << " " << this->point_clouds[i].m_pose(2, 1) << " " << this->point_clouds[i].m_pose(2, 2) << " " << this->point_clouds[i].m_pose(2, 3) << std::endl;
		outfile << "0 0 0 1" << std::endl;
	}

	/*outfile << "point_cloud_file_name; r11; r12; r13; r21; r22; r23; r31; r32; r33; t14; t24; t34" << std::endl;
	for (size_t i = 0; i < this->point_clouds.size(); i++) {
		outfile << this->point_clouds[i].file_name << ";" <<
			this->point_clouds[i].m_pose(0, 0) << ";" <<
			this->point_clouds[i].m_pose(0, 1) << ";" <<
			this->point_clouds[i].m_pose(0, 2) << ";" <<
			this->point_clouds[i].m_pose(1, 0) << ";" <<
			this->point_clouds[i].m_pose(1, 1) << ";" <<
			this->point_clouds[i].m_pose(1, 2) << ";" <<
			this->point_clouds[i].m_pose(2, 0) << ";" <<
			this->point_clouds[i].m_pose(2, 1) << ";" <<
			this->point_clouds[i].m_pose(2, 2) << ";" <<
			this->point_clouds[i].m_pose(0, 3) << ";" <<
			this->point_clouds[i].m_pose(1, 3) << ";" <<
			this->point_clouds[i].m_pose(2, 3) << std::endl;
	}*/

	return true;
}
#endif

bool PointClouds::save_scans()
{
	for (size_t i = 0; i < this->point_clouds.size(); i++) {
		if (!this->point_clouds[i].save_as_global(this->out_folder_name)) {
			std::cout << "problem with saving to folder: " << this->out_folder_name << std::endl;
		}
	}
	return true;
}

void PointClouds::show_all()
{
	for (auto& pc: point_clouds) {
		pc.visible = true;
	}
}

void PointClouds::hide_all()
{
	for (auto& pc: point_clouds) {
		pc.visible = false;
	}
}

double get_mean_uncertainty_xyz_impact6x6(std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>& uncertainty_before,
	std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>& uncertainty_after)
{
	double mui = 0.0;

	double sum_before = 0.0;
	double sum_after = 0.0;

	for (size_t i = 0; i < uncertainty_before.size(); i++) {
		sum_before += ((uncertainty_before[i].coeffRef(0, 0) + uncertainty_before[i].coeffRef(1, 1) + uncertainty_before[i].coeffRef(2, 2)) / 3.0);
		sum_after += ((uncertainty_after[i].coeffRef(0, 0) + uncertainty_after[i].coeffRef(1, 1) + uncertainty_after[i].coeffRef(2, 2)) / 3.0);
	}
	sum_before /= uncertainty_before.size();
	sum_after /= uncertainty_before.size();

	mui = sum_before / sum_after;

	return mui;
}

double get_mean_uncertainty_xyz_impact7x7(std::vector<Eigen::Matrix<double, 7, 7, Eigen::RowMajor>>& uncertainty_before,
	std::vector<Eigen::Matrix<double, 7, 7, Eigen::RowMajor>>& uncertainty_after)
{
	double mui = 0.0;

	double sum_before = 0.0;
	double sum_after = 0.0;

	for (size_t i = 0; i < uncertainty_before.size(); i++) {
		sum_before += ((uncertainty_before[i].coeffRef(0, 0) + uncertainty_before[i].coeffRef(1, 1) + uncertainty_before[i].coeffRef(2, 2)) / 3.0);
		sum_after += ((uncertainty_after[i].coeffRef(0, 0) + uncertainty_after[i].coeffRef(1, 1) + uncertainty_after[i].coeffRef(2, 2)) / 3.0);
	}
	sum_before /= uncertainty_before.size();
	sum_after /= uncertainty_before.size();

	mui = sum_before / sum_after;

	return mui;
}

double get_mean_uncertainty_xyz_impact(std::vector<Eigen::SparseMatrix<double>>& uncertainty_before, std::vector<Eigen::SparseMatrix<double>>& uncertainty_after)
{
	double mui = 0.0;

	double sum_before = 0.0;
	double sum_after = 0.0;

	for (size_t i = 0; i < uncertainty_before.size(); i++) {
		sum_before += ((uncertainty_before[i].coeffRef(0, 0) + uncertainty_before[i].coeffRef(1, 1) + uncertainty_before[i].coeffRef(2, 2)) / 3.0);
		sum_after += ((uncertainty_after[i].coeffRef(0, 0) + uncertainty_after[i].coeffRef(1, 1) + uncertainty_after[i].coeffRef(2, 2)) / 3.0);
	}
	sum_before /= uncertainty_before.size();
	sum_after /= uncertainty_before.size();

	mui = sum_before / sum_after;

	return mui;
}
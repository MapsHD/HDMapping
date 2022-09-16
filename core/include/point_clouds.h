#ifndef _POINT_CLOUDS_H_
#define _POINT_CLOUDS_H_

#include <string>
#include <vector>

#include <point_cloud.h>
#include <observation_picking.h>

class PointClouds {
public:
	PointClouds() { ; };
	~PointClouds() { ; };

	std::string folder_name;
	std::string out_folder_name;
	std::string out_poses_file_name;
	bool load(const std::string& folder_with_point_clouds, const std::string& poses_file_name);
	//std::vector<Eigen::Vector3d> load_points(const std::string& point_clouds_file_name);
	void render(const ObservationPicking& observation_picking);
	//bool save_poses();
	bool save_poses(const std::string file_name);
	bool save_scans();
	void show_all();
	void hide_all();

	std::vector<PointCloud> point_clouds;
	bool show_with_initial_pose = false;
};

double get_mean_uncertainty_xyz_impact6x6(std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>& uncertainty_before, std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>& uncertainty_after);
double get_mean_uncertainty_xyz_impact7x7(std::vector<Eigen::Matrix<double, 7, 7, Eigen::RowMajor>>& uncertainty_before, std::vector<Eigen::Matrix<double, 7, 7, Eigen::RowMajor>>& uncertainty_after);
double get_mean_uncertainty_xyz_impact(std::vector<Eigen::SparseMatrix<double>>& uncertainty_before, std::vector<Eigen::SparseMatrix<double>>& uncertainty_after);


#endif
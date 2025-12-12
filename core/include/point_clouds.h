#ifndef _POINT_CLOUDS_H_
#define _POINT_CLOUDS_H_

#include <string>
#include <vector>

#include <point_cloud.h>
#include <transformations.h>
#if WITH_GUI == 1
#include <observation_picking.h>
#endif

class PointClouds {
public:
	PointClouds() { ; };
	~PointClouds() { ; };

	bool xz_intersection = false;
	bool yz_intersection = false;
	bool xy_intersection = false;

	double intersection_width = 0.05;
	bool xz_grid_10x10 = false;
	bool xz_grid_1x1 = false;
	bool xz_grid_01x01 = false;
	bool yz_grid_10x10 = false;
	bool yz_grid_1x1 = false;
	bool yz_grid_01x01 = false;
	bool xy_grid_10x10 = false;
	bool xy_grid_1x1 = false;
	bool xy_grid_01x01 = false;

	Eigen::Vector3d offset = Eigen::Vector3d(0, 0, 0);
	Eigen::Vector3d offset_to_apply = Eigen::Vector3d(0, 0, 0);
	std::string folder_name;
	std::string out_folder_name;
	std::string poses_file_name;
	std::string initial_poses_file_name;
	std::string out_poses_file_name;
	bool show_imu_to_lio_diff = false;

	struct PointCloudDimensions {
		double x_min, x_max, y_min, y_max, z_min, z_max;
		double length, width, height;
	};

	bool load(const std::string &folder_with_point_clouds, const std::string &poses_file_name, bool decimation,
			  double bucket_x, double bucket_y, double bucket_z, bool load_cache_mode);
	bool update_poses_from_RESSO(const std::string& folder_with_point_clouds, const std::string& poses_file_name);
	bool update_poses_from_RESSO_inverse(const std::string &folder_with_point_clouds, const std::string &poses_file_name);
	bool update_initial_poses_from_RESSO(const std::string &folder_with_point_clouds, const std::string &poses_file_name);
	bool load_eth(const std::string& folder_with_point_clouds, const std::string& poses_file_name, bool decimation, double bucket_x, double bucket_y, double bucket_z);
	//std::vector<Eigen::Vector3d> load_points(const std::string& point_clouds_file_name);
#if WITH_GUI == 1
	void draw_grids(bool xz_grid_10x10, bool xz_grid_1x1, bool xz_grid_01x01,
		bool yz_grid_10x10, bool yz_grid_1x1, bool yz_grid_01x01,
		bool xy_grid_10x10, bool xy_grid_1x1, bool xy_grid_01x01, PointClouds::PointCloudDimensions dims);
	void render(const ObservationPicking &observation_picking, int viewer_decmiate_point_cloud, bool xz_intersection, bool yz_intersection, bool xy_intersection,
				bool xz_grid_10x10, bool xz_grid_1x1, bool xz_grid_01x01, bool yz_grid_10x10,
				bool yz_grid_1x1, bool yz_grid_01x01, bool xy_grid_10x10, bool xy_grid_1x1, bool xy_grid_01x01, double intersection_width, PointClouds::PointCloudDimensions dims = {});
#endif
	//bool save_poses();
	bool save_poses(const std::string file_name, bool is_subsession);
	bool save_scans();
	void show_all();
	void show_all_from_range(int index_begin, int index_end);
	void hide_all();

	std::vector<PointCloud> point_clouds;
	bool show_with_initial_pose = false;
	
	bool load_pose_ETH(const std::string& fn, Eigen::Affine3d &m_increment);
	bool load_whu_tls(std::vector<std::string> input_file_names, bool is_decimate, double bucket_x, double bucket_y, double bucket_z, bool calculate_offset, bool load_cache_mode);
	PointCloudDimensions compute_point_cloud_dimension() const;
	void print_point_cloud_dimension();
	bool load_3DTK_tls(std::vector<std::string> input_file_names, bool is_decimate, double bucket_x, double bucket_y, double bucket_z);

	bool load_pc(PointCloud &pc, std::string input_file_name, bool load_cache_mode);
};

double get_mean_uncertainty_xyz_impact6x6(std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>& uncertainty_before, std::vector<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>& uncertainty_after);
double get_mean_uncertainty_xyz_impact7x7(std::vector<Eigen::Matrix<double, 7, 7, Eigen::RowMajor>>& uncertainty_before, std::vector<Eigen::Matrix<double, 7, 7, Eigen::RowMajor>>& uncertainty_after);
double get_mean_uncertainty_xyz_impact(std::vector<Eigen::SparseMatrix<double>>& uncertainty_before, std::vector<Eigen::SparseMatrix<double>>& uncertainty_after);


#endif
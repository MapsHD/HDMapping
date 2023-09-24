#pragma once
#include <laszip/laszip_api.h>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <execution>
#include <Fusion.h>
#include <vector>
#include "structures.h"
namespace mandeye::utilsIO
{

	bool saveLaz(const std::string& filename, const std::vector<Point3Di>& points_global);
	bool save_poses(const std::string file_name, std::vector<Eigen::Affine3d> m_poses, std::vector<std::string> filenames);
	std::vector<std::tuple<double, FusionVector, FusionVector>> load_imu(const std::string& imu_file);
	std::vector<Point3Di> load_point_cloud(const std::string& lazFile, bool ommit_points_with_timestamp_equals_zero = true);

}
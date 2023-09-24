#include <iostream>
#include "lidarOdomIO.h"
#include "lidarOdometry.h"
#include <vector>
#include <string>
int main()
{
	std::cout << "Hello" << std::endl;

	std::vector<std::string> imu_files{
		"E:/exp3/continousScanning_0032_sub/imu0000.csv",
		"E:/exp3/continousScanning_0032_sub/imu0001.csv",
		"E:/exp3/continousScanning_0032_sub/imu0002.csv",
		"E:/exp3/continousScanning_0032_sub/imu0003.csv",
		"E:/exp3/continousScanning_0032_sub/imu0004.csv",
		"E:/exp3/continousScanning_0032_sub/imu0005.csv",
		"E:/exp3/continousScanning_0032_sub/imu0006.csv",
		"E:/exp3/continousScanning_0032_sub/imu0007.csv",
		"E:/exp3/continousScanning_0032_sub/imu0008.csv",
		"E:/exp3/continousScanning_0032_sub/imu0009.csv",
		"E:/exp3/continousScanning_0032_sub/imu0010.csv",
	};

	std::vector<std::string> pc_files{
		"E:/exp3/continousScanning_0032_sub/lidar0000.laz",
		"E:/exp3/continousScanning_0032_sub/lidar0001.laz",
		"E:/exp3/continousScanning_0032_sub/lidar0002.laz",
		"E:/exp3/continousScanning_0032_sub/lidar0003.laz",
		"E:/exp3/continousScanning_0032_sub/lidar0004.laz",
		"E:/exp3/continousScanning_0032_sub/lidar0005.laz",
		"E:/exp3/continousScanning_0032_sub/lidar0006.laz",
		"E:/exp3/continousScanning_0032_sub/lidar0007.laz",
		"E:/exp3/continousScanning_0032_sub/lidar0008.laz",
		"E:/exp3/continousScanning_0032_sub/lidar0009.laz",
		"E:/exp3/continousScanning_0032_sub/lidar0010.laz",
	};

	std::vector<std::tuple<double, FusionVector, FusionVector>> imu_data;
	std::vector<Point3Di> pointcloud;
	for (const auto file : imu_files)
	{
		auto data = mandeye::utilsIO::load_imu(file);
		imu_data.insert(imu_data.end(), std::make_move_iterator(data.begin()), std::make_move_iterator(data.end()));
	}
	for (const auto file : pc_files)
	{
		auto data = mandeye::utilsIO::load_point_cloud(file);
		pointcloud.insert(pointcloud.end(), std::make_move_iterator(data.begin()), std::make_move_iterator(data.end()));
	}

	mandeye::SlamConfig config;
	config.tempSave = "E:/exp3/test/";
	mandeye::optimizeTrajectory(imu_data, pointcloud, config);
}
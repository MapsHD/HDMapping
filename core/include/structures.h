#ifndef _STRUCTURES_H_
#define _STRUCTURES_H_

#include <string>
#include <vector>
#include <fstream>
#include <iostream>

#include <Eigen/Eigen>

struct PerspectiveCameraParams {
	double cx;
	double cy;
	double fx;
	double fy;
};

struct MetricCameraParams{
	double c;
	double ksi_0;
	double eta_0;
};

struct TaitBryanPose
{
	double px;
	double py;
	double pz;
	double om;
	double fi;
	double ka;
};

struct RodriguesPose
{
	double px;
	double py;
	double pz;
	double sx;
	double sy;
	double sz;
};

struct QuaternionPose
{
	double px;
	double py;
	double pz;
	double q0;
	double q1;
	double q2;
	double q3;
};


struct Point3D
{
	//double x;
	//double y;
	//double z;
	float x;
	float y;
	float z;
	int index_pose;
	//double timestamp;
};

struct Point
{
	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;
	uint32_t intensity = 0;
	double time = 0.0;
};

struct ChunkFile {
	int filename;
	double time_begin_inclusive;
	double time_end_inclusive;
};

template<typename T>
inline bool save_vector_data(const std::string& file_name, std::vector<T>& vector_data) {
	std::ofstream ofs(file_name, std::ios::binary);
	if (!ofs.good()) {
		return false;
	}
	ofs.write(reinterpret_cast<char*>(vector_data.data()), vector_data.size()* sizeof(T));
	return true;
}

template<typename T>
inline bool load_vector_data(const std::string& file_name, std::vector<T>& vector_data) {
	std::basic_ifstream<char> vd_str(file_name, std::ios::binary);
	if (!vd_str.good()) {
		return false;
	}
	std::vector<char> data((std::istreambuf_iterator<char>(vd_str)), std::istreambuf_iterator<char>());
	std::vector<T> v(data.size() / sizeof(T));
	memcpy(v.data(), data.data(), data.size());
	vector_data = v;
	return true;
}

struct LaserBeam {
	Eigen::Vector3d position;
	Eigen::Vector3d direction;
	float distance;
	float range;
};

struct Plane {
	Plane() {
		a = b = c = d = 0.0;
	}
	double a;
	double b;
	double c;
	double d;

	Plane(Eigen::Vector3d p, Eigen::Vector3d nv) {
		a = nv.x();
		b = nv.y();
		c = nv.z();
		d = -a * p.x() - b * p.y() - c * p.z();
	}
};

struct CommonData {
	Eigen::Vector3d roi = Eigen::Vector3d(0, 0, 0);
	float roi_size = 50;
	float rotate_x = 0.0, rotate_y = 0.0;
	float translate_x, translate_y = 0.0;
	float translate_z = -5000;
	bool is_ortho = true;
	bool roi_exorter = false;
	bool laz_wrapper = false;
	bool single_trajectory_viewer = false;
	bool odo_with_gnss_fusion = false;
	float shift_x = 0.0;
	float shift_y = 0.0;
};

struct PointCloudWithPose {
	bool visible = true;
	double timestamp;
	std::string trajectory_file_name;
	Eigen::Affine3d m_pose;
	std::vector<Point> points_global;
	int point_size = 1;
};

struct Job {
	long long unsigned int index_begin_inclusive;
	long long unsigned int index_end_exclusive;
};

inline std::vector<Job> get_jobs(long long unsigned int size, int num_threads) {

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

struct LAZPoint
{
	double x = 0.0f;
	double y = 0.0f;
	double z = 0.0f;
	//uint16_t r;
	//uint16_t g;
	//uint16_t b;
	float r;
	float g;
	float b;
	uint8_t  classsification;
	//unsigned short RGB[4];
};

struct LAZSector {
	std::string file_name;
	std::vector<LAZPoint> point_cloud;
	double min_x;
	double max_x;
	double min_y;
	double max_y;
	bool visible = true;
	int point_size = 1;
};

struct ConstraintToGeoreference {
	std::string trajectory_file_name;
	double time_stamp;
	Eigen::Affine3d m_pose; 
};

struct ROIwithConstraints {
	std::vector<ConstraintToGeoreference> constraints;
};

struct Node {
	double timestamp = 0.0;
	Eigen::Affine3d m_pose = Eigen::Affine3d::Identity();
	int index_to_lidar_odometry_odo = -1;
	int index_to_gnss = -1;
};

struct BetweenNode {
	Node node_outer;
	std::vector<Node> nodes_between;
	float color_x = 0;
	float color_y = 0;
	float color_z = 0;
};

struct GeoPoint {
	std::string name;
	Eigen::Vector3d coordinates;
	bool choosen = false;
	double w_x = 1000000.0;
	double w_y = 1000000.0;
	double w_z = 1000000.0;
};


#endif

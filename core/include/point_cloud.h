#ifndef _POINT_CLOUD_H_
#define _POINT_CLOUD_H_

#include <string>
#include <vector>
#include <Eigen/Eigen>

#include <structures.h>
#include <observation_picking.h>
//#include <laszip/laszip_api.h>

class PointCloud {
public:
	struct GridParameters {
		double bounding_box_min_X = 0.0;
		double bounding_box_min_Y = 0.0;
		double bounding_box_min_Z = 0.0;
		double bounding_box_max_X = 0.0;
		double bounding_box_max_Y = 0.0;
		double bounding_box_max_Z = 0.0;
		double bounding_box_extension = 1.0;
		int number_of_buckets_X = 0;
		int number_of_buckets_Y = 0;
		int number_of_buckets_Z = 0;
		long long unsigned int number_of_buckets = 0;
		double resolution_X = 2.0;
		double resolution_Y = 2.0;
		double resolution_Z = 2.0;
	};

	struct PointBucketIndexPair {
		int index_of_point = 0;
		long long unsigned int index_of_bucket = 0;
		//int index_pose;
	};

	struct Bucket {
		long long unsigned int index_begin = 0;
		long long unsigned int index_end = 0;
		long long unsigned int number_of_points = 0;
	};

	struct Job {
		long long unsigned int index_begin_inclusive = 0;
		long long unsigned int index_end_exclusive = 0;
	};

	struct NearestNeighbour {
		int index_source = 0;
		int index_target = 0;
		bool found = false;
	};

	PointCloud() {
		m_pose = Eigen::Affine3d::Identity();
		pose.px = pose.py = pose.pz = pose.om = pose.fi = pose.ka = 0.0;
		gui_translation[0] = gui_translation[1] = gui_translation[2] = 0.0f;
		gui_rotation[0] = gui_rotation[1] = gui_rotation[2] = 0.0;

		render_color[0] = (float(rand() % 255)) / 255.0f;
		render_color[1] = (float(rand() % 255)) / 255.0f;
		render_color[2] = (float(rand() % 255)) / 255.0f;

		visible = true;
		gizmo = false;
	};
	~PointCloud() { ; };

	GridParameters rgd_params;
	std::vector<PointBucketIndexPair> index_pairs;
	std::vector < Bucket> buckets;
	std::string file_name;
	std::vector<Eigen::Vector3d> points_local;
	std::vector<Eigen::Vector3d> normal_vectors_local;
	std::vector <int> points_type;
	std::vector <unsigned short> intensities;
	Eigen::Affine3d m_pose;
	Eigen::Affine3d m_initial_pose;
	Eigen::Matrix<double, 6, 6, Eigen::RowMajor> covariance_matrix_tait_bryan;
	Eigen::Matrix<double, 6, 6, Eigen::RowMajor> information_matrix_tait_bryan;
	int number_points_vertical = 0;
	int number_points_horizontal = 0;
	int point_size = 1;
	std::vector<GeoPoint> available_geo_points;
	bool choosing_geo = false;

	TaitBryanPose pose;
	float gui_translation[3];
	float gui_rotation[3];
	float render_color[3];
	bool visible;
	bool gizmo;
	int num_threads = 16;
	bool fixed = false;
	//double search_radious = 2.0;

	bool load(const std::string& file_name);
	void render(bool show_with_initial_pose, const ObservationPicking& observation_picking, int viewer_decmiate_point_cloud);
	void update_from_gui();
	bool save_as_global(std::string file_name);
	//rgd
	bool build_rgd();
	void grid_calculate_params(std::vector<Eigen::Vector3d>& points, GridParameters& params);
	void reindex(std::vector<PointBucketIndexPair>& ip, std::vector<Eigen::Vector3d>& points, GridParameters rgd_params);
	std::vector<PointCloud::Job> get_jobs(long long unsigned int size, int num_threads);
	void cout_rgd();
	std::vector<std::pair<int,int>> nns(PointCloud& pc_target, double search_radious);
	void clean();
	void compute_normal_vectors(double search_radious);
	void decimate(double bucket_x, double bucket_y, double bucket_z);
};

#endif
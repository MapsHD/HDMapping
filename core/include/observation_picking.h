#ifndef _OBSERVATION_PICKING_H_
#define _OBSERVATION_PICKING_H_

#include <json.hpp>
#include <Eigen/Eigen>
#include <map>

class ObservationPicking {
public:
	ObservationPicking() { ; };
	~ObservationPicking() { ; };

	bool is_observation_picking_mode = false;
	float picking_plane_height = 0.0f;
	float picking_plane_threshold = 0.1f;
	float max_xy = 200.0f;
	int point_size = 1;
	//bool high_density_grid = false;
	bool grid10x10m = false;
	bool grid1x1m = false;
	bool grid01x01m = false;
	bool grid001x001m = false;
	void render();
	void add_picked_to_current_observation(int index_picked, Eigen::Vector3d p);
	void accept_current_observation(std::vector<Eigen::Affine3d> m_poses);
	void import_observations(/*std::vector<std::map<int, Eigen::Vector3d>>& observations,*/ const std::string& filename);
	void export_observation(/*std::vector<std::map<int, Eigen::Vector3d>>& observations,*/ const std::string& filename);

	

	std::map<int, Eigen::Vector3d> current_observation;
	std::vector<std::map<int, Eigen::Vector3d>> observations;

};

#endif
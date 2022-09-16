#include <observation_picking.h>
#include <GL/freeglut.h>

#include <iostream>
#include <fstream>

#include <nlohmann/json.hpp>

void ObservationPicking::render()
{
	if (is_observation_picking_mode) {
		if (grid10x10m) {
			glColor3f(0.7, 0.7, 0.7);
			glBegin(GL_LINES);
			for (float x = -max_xy; x <= max_xy; x += 10.0) {
				glVertex3f(x, -max_xy, picking_plane_height);
				glVertex3f(x, max_xy, picking_plane_height);
			}
			for (float y = -max_xy; y <= max_xy; y += 10.0) {
				glVertex3f(-max_xy, y, picking_plane_height);
				glVertex3f(max_xy, y, picking_plane_height);
			}
			glEnd();
		}
		if (grid1x1m) {
			glColor3f(0.3, 0.3, 0.3);
			glBegin(GL_LINES);
			for (float x = -max_xy; x <= max_xy; x += 1) {
				glVertex3f(x, -max_xy, picking_plane_height);
				glVertex3f(x, max_xy, picking_plane_height);
			}
			for (float y = -max_xy; y <= max_xy; y += 1) {
				glVertex3f(-max_xy, y, picking_plane_height);
				glVertex3f(max_xy, y, picking_plane_height);
			}
			glEnd();
		}
		if (grid01x01m) {
			glColor3f(0.1, 0.1, 0.1);
			glBegin(GL_LINES);
			for (float x = -max_xy; x <= max_xy; x += 0.1) {
				glVertex3f(x, -max_xy, picking_plane_height);
				glVertex3f(x, max_xy, picking_plane_height);
			}
			for (float y = -max_xy; y <= max_xy; y += 0.1) {
				glVertex3f(-max_xy, y, picking_plane_height);
				glVertex3f(max_xy, y, picking_plane_height);
			}
			glEnd();
		}
		if (grid001x001m) {
			glColor3f(0.8, 0.8, 0.8);
			glBegin(GL_LINES);
			for (float x = -max_xy; x <= max_xy; x += 0.01) {
				glVertex3f(x, -max_xy, picking_plane_height);
				glVertex3f(x, max_xy, picking_plane_height);
			}
			for (float y = -max_xy; y <= max_xy; y += 0.01) {
				glVertex3f(-max_xy, y, picking_plane_height);
				glVertex3f(max_xy, y, picking_plane_height);
			}
			glEnd();
		}
	}

	glColor3f(1,1,1);
	glPointSize(5);
	glBegin(GL_POINTS);
	for (const auto& [key, value] : current_observation) {
		glVertex3f(value.x(), value.y(), value.z());
	}
	glEnd();

	glColor3f(1.0, 0.2, 0.2);
	glBegin(GL_LINES);
	for (const auto& [key1, value1] : current_observation) {
		for (const auto& [key2, value2] : current_observation) {
			if (key1 != key2) {
				glVertex3f(value1.x(), value1.y(), value1.z());
				glVertex3f(value2.x(), value2.y(), value2.z());
			}
		}
	}
	glEnd();

}

void ObservationPicking::add_picked_to_current_observation(int index_picked, Eigen::Vector3d p)
{
	std::cout << "inserting observation: point cloud[" << index_picked << "], coordinates [" << p.x() << "," << p.y() << "," << p.z() << "]" << std::endl;

	//current_observation.insert_or_assign({ index_picked, p });
	auto [iterator, inserted] = current_observation.try_emplace(index_picked, p);
	if (!inserted) { iterator->second = p; }

	std::cout << "current_observation" << std::endl;
	for (const auto& [key, value] : current_observation) {
		std::cout << '[' << key << "] = " << value.x() << " " << value.y() << " " << value.z() << std::endl;
	}
}

void ObservationPicking::accept_current_observation(std::vector<Eigen::Affine3d> m_poses)
{
	for (auto& [key, value] : current_observation) {
		//glVertex3f(value.x(), value.y(), value.z());
		value = m_poses[key].inverse() * value;
	}

	observations.push_back(current_observation);

	current_observation.clear();
	std::cout << "current_observation.size(): " << current_observation.size() << std::endl;
}

void ObservationPicking::import_observations(/*std::vector<std::map<int, Eigen::Vector3d>>& observations,*/ const std::string& filename) {
	observations.clear();
	std::ifstream fs(filename);
	nlohmann::json data = nlohmann::json::parse(fs);
	fs.close();
	for (auto& observation_json : data["observations"])
	{
		std::map<int, Eigen::Vector3d> visible_point;
		for (auto& visible_point_json : observation_json)
		{
			const auto local_coords = visible_point_json["local_coords"];
			const int scan_id = visible_point_json["scan_id"];
			assert(local_coords.size() == 3);
			visible_point[scan_id] = Eigen::Vector3d(local_coords[0], local_coords[1], local_coords[2]);
		}
		observations.push_back(visible_point);
	}
	std::cout << "number of loaded observations: " << observations.size() << std::endl;
}

void ObservationPicking::export_observation(/*std::vector<std::map<int, Eigen::Vector3d>>& observations,*/ const std::string& filename)
{
	nlohmann::json json;
	for (const auto& observation : observations) {
		nlohmann::json json_obs;
		for (const auto& [scan_id, local_coords] : observation)
		{
			nlohmann::json visible_point{
					{"scan_id", scan_id},
					{"local_coords",{local_coords.x(),local_coords.y(),local_coords.z()}}
			};
			json_obs.push_back(visible_point);
		}
		json["observations"].push_back(json_obs);
	}
	std::ofstream fs(filename);
	fs << json.dump(2);
	fs.close();
	std::cout << "number of saved observations: " << observations.size() << std::endl;
}
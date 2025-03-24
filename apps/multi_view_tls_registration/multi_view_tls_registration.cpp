#include <pcl_wrapper.h>
#include <fstream>
#include <laszip/laszip_api.h>
#include <export_laz.h>
#include "multi_view_tls_registration.h"


bool has_extension(const std::string file_path, const std::string extension) {
	std::string::size_type dot_pos = file_path.find_last_of('.');
	if (dot_pos == std::string::npos) {
		return false; // No extension found
	}
	std::string file_extension = file_path.substr(dot_pos);
	return file_extension == extension;
}

void initial_pose_to_identity(Session& session)
{
	if (session.point_clouds_container.point_clouds.size() > 0)
	{
		auto m_inv = session.point_clouds_container.point_clouds[0].m_pose.inverse();
		for (int i = 0; i < session.point_clouds_container.point_clouds.size(); i++)
		{
			session.point_clouds_container.point_clouds[i].m_pose = m_inv * session.point_clouds_container.point_clouds[i].m_pose;
		}
	}
}

void save_all_to_las(const Session& session, std::string output_las_name)
{
	std::vector<Eigen::Vector3d> pointcloud;
	std::vector<unsigned short> intensity;
	std::vector<double> timestamps;

	for (auto& p : session.point_clouds_container.point_clouds)
	{
		if (p.visible)
		{
			for (int i = 0; i < p.points_local.size(); i++)
			{
				const auto& pp = p.points_local[i];
				Eigen::Vector3d vp;
				vp = p.m_pose * pp; // + session.point_clouds_container.offset;
				// std::cout << vp << std::endl;
				pointcloud.push_back(vp);
				if (i < p.intensities.size())
				{
					intensity.push_back(p.intensities[i]);
				}
				else
				{
					intensity.push_back(0);
				}
				if (i < p.timestamps.size())
				{
					timestamps.push_back(p.timestamps[i]);
				}				
			}
		}
	}
	if (!exportLaz(output_las_name, pointcloud, intensity, timestamps, session.point_clouds_container.offset.x(), session.point_clouds_container.offset.y(), session.point_clouds_container.offset.z()))
	{
		std::cout << "problem with saving file: " << output_las_name << std::endl;
	}
}

void save_separately_to_las(const Session& session, fs::path outwd, std::string extension)
{
	for (auto& p : session.point_clouds_container.point_clouds)
	{
		if (p.visible)
		{
			fs::path file_path_in = p.file_name;
			fs::path file_path_put = outwd;
			file_path_put /= (file_path_in.stem().string() + "_processed" + extension);
			std::cout << "file_in: " << file_path_in << std::endl;
			std::cout << "file_out: " << file_path_put << std::endl;
			std::cout << "start save_processed_pc" << std::endl;
			bool compressed = (extension == ".laz");
			save_processed_pc(file_path_in, file_path_put, p.m_pose, session.point_clouds_container.offset, compressed);
			std::cout << "save_processed_pc finished" << std::endl;
		}
	}
}

void save_trajectories_to_laz(const Session& session, std::string output_file_name, float curve_consecutive_distance_meters, float not_curve_consecutive_distance_meters, bool is_trajectory_export_downsampling)
{
	std::vector<Eigen::Vector3d> pointcloud;
	std::vector<unsigned short> intensity;
	std::vector<double> timestamps;

	float consecutive_distance = 0;
	for (auto& p : session.point_clouds_container.point_clouds)
	{
		if (p.visible)
		{
			for (int i = 0; i < p.local_trajectory.size(); i++)
			{
				const auto& pp = p.local_trajectory[i].m_pose.translation();
				Eigen::Vector3d vp;
				vp = p.m_pose * pp; // + session.point_clouds_container.offset;

				if (i > 0)
				{
					double dist = (p.local_trajectory[i].m_pose.translation() - p.local_trajectory[i - 1].m_pose.translation()).norm();
					consecutive_distance += dist;
				}

				bool is_curve = false;

				if (i > 100 && i < p.local_trajectory.size() - 100)
				{
					Eigen::Vector3d position_prev = p.local_trajectory[i - 100].m_pose.translation();
					Eigen::Vector3d position_curr = p.local_trajectory[i].m_pose.translation();
					Eigen::Vector3d position_next = p.local_trajectory[i + 100].m_pose.translation();

					Eigen::Vector3d v1 = position_curr - position_prev;
					Eigen::Vector3d v2 = position_next - position_curr;

					if (v1.norm() > 0 && v2.norm() > 0)
					{
						double angle_deg = fabs(acos(v1.dot(v2) / (v1.norm() * v2.norm())) * 180.0 / M_PI);

						if (angle_deg > 10.0)
						{
							is_curve = true;
						}
					}
				}
				double tol = not_curve_consecutive_distance_meters;

				if (is_curve)
				{
					tol = curve_consecutive_distance_meters;
				}

				if (!is_trajectory_export_downsampling)
				{
					pointcloud.push_back(vp);
					intensity.push_back(0);
					timestamps.push_back(p.local_trajectory[i].timestamps.first);
				}
				else
				{
					if (consecutive_distance >= tol)
					{
						consecutive_distance = 0;
						pointcloud.push_back(vp);
						intensity.push_back(0);
						timestamps.push_back(p.local_trajectory[i].timestamps.first);
					}
				}
			}
		}
	}
	// if (!exportLaz(output_file_name, pointcloud, intensity, gnss.offset_x, gnss.offset_y, gnss.offset_alt))
	if (!exportLaz(output_file_name, pointcloud, intensity, timestamps, session.point_clouds_container.offset.x(), session.point_clouds_container.offset.y(), session.point_clouds_container.offset.z()))
	{
		std::cout << "problem with saving file: " << output_file_name << std::endl;
	}
}

void createDXFPolyline(const std::string& filename, const std::vector<Eigen::Vector3d>& points)
{
	std::ofstream dxfFile(filename);
	dxfFile << std::setprecision(20);
	if (!dxfFile.is_open())
	{
		std::cerr << "Failed to open file: " << filename << std::endl;
		return;
	}

	// DXF header
	dxfFile << "0\nSECTION\n2\nHEADER\n0\nENDSEC\n";
	dxfFile << "0\nSECTION\n2\nTABLES\n0\nENDSEC\n";

	// Start the ENTITIES section
	dxfFile << "0\nSECTION\n2\nENTITIES\n";

	// Start the POLYLINE entity
	dxfFile << "0\nPOLYLINE\n";
	dxfFile << "8\n0\n";  // Layer 0
	dxfFile << "66\n1\n"; // Indicates the presence of vertices
	dxfFile << "70\n8\n"; // 1 = Open polyline

	// Write the VERTEX entities
	for (const auto& point : points)
	{
		dxfFile << "0\nVERTEX\n";
		dxfFile << "8\n0\n"; // Layer 0
		dxfFile << "10\n"
			<< point.x() << "\n"; // X coordinate
		dxfFile << "20\n"
			<< point.y() << "\n"; // Y coordinate
		dxfFile << "30\n"
			<< point.z() << "\n"; // Z coordinate
	}

	// End the POLYLINE
	dxfFile << "0\nSEQEND\n";

	// End the ENTITIES section
	dxfFile << "0\nENDSEC\n";

	// End the DXF file
	dxfFile << "0\nEOF\n";

	dxfFile.close();
	std::cout << "DXF file created: " << filename << std::endl;
}

void save_trajectories(
	Session& session, std::string output_file_name, float curve_consecutive_distance_meters,
	float not_curve_consecutive_distance_meters, bool is_trajectory_export_downsampling,
	bool write_lidar_timestamp, bool write_unix_timestamp, bool use_quaternions,
	bool save_to_dxf)
{
	std::ofstream outfile;
	if (!save_to_dxf)
	{
		outfile.open(output_file_name);
	}
	if (save_to_dxf || outfile.good())
	{
		float consecutive_distance = 0;
		std::vector<Eigen::Vector3d> polylinePoints;
		for (auto& p : session.point_clouds_container.point_clouds)
		{
			if (p.visible)
			{
				for (int i = 0; i < p.local_trajectory.size(); i++)
				{
					const auto& m = p.local_trajectory[i].m_pose;
					Eigen::Affine3d pose = p.m_pose * m;
					pose.translation() += session.point_clouds_container.offset;

					if (i > 0)
					{
						double dist = (p.local_trajectory[i].m_pose.translation() - p.local_trajectory[i - 1].m_pose.translation()).norm();
						consecutive_distance += dist;
					}

					bool is_curve = false;

					if (i > 100 && i < p.local_trajectory.size() - 100)
					{
						Eigen::Vector3d position_prev = p.local_trajectory[i - 100].m_pose.translation();
						Eigen::Vector3d position_curr = p.local_trajectory[i].m_pose.translation();
						Eigen::Vector3d position_next = p.local_trajectory[i + 100].m_pose.translation();

						Eigen::Vector3d v1 = position_curr - position_prev;
						Eigen::Vector3d v2 = position_next - position_curr;

						if (v1.norm() > 0 && v2.norm() > 0)
						{
							double angle_deg = fabs(acos(v1.dot(v2) / (v1.norm() * v2.norm())) * 180.0 / M_PI);

							if (angle_deg > 10.0)
							{
								is_curve = true;
							}
						}
					}
					double tol = not_curve_consecutive_distance_meters;

					if (is_curve)
					{
						tol = curve_consecutive_distance_meters;
					}

					if (!is_trajectory_export_downsampling || (is_trajectory_export_downsampling && consecutive_distance >= tol))
					{
						if (is_trajectory_export_downsampling) {
							consecutive_distance = 0;
						}
						if (save_to_dxf)
						{
							polylinePoints.push_back(pose.translation());
						}
						else
						{
							outfile << std::setprecision(20);
							if (write_lidar_timestamp)
							{
								outfile << p.local_trajectory[i].timestamps.first << ",";
							}
							if (write_unix_timestamp)
							{
								outfile << p.local_trajectory[i].timestamps.second << ",";
							}
							outfile << pose(0, 3) << "," << pose(1, 3) << "," << pose(2, 3) << ",";
							if (use_quaternions)
							{
								Eigen::Quaterniond q(pose.rotation());
								outfile << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << std::endl;
							}
							else
							{
								outfile << pose(0, 0) << "," << pose(0, 1) << "," << pose(0, 2) << "," << pose(1, 0) << "," << pose(1, 1) << "," << pose(1, 2) << "," << pose(2, 0) << "," << pose(2, 1) << "," << pose(2, 2) << std::endl;
							}
						}
					}
				}
			}
		}
		if (!save_to_dxf)
		{
			outfile.close();
		}
		else
		{
			createDXFPolyline(output_file_name, polylinePoints);
		}
	}
}

/*void load_available_geo_points(Session& session, std::string input_file_name)
{
	std::vector<GeoPoint> available_geo_points;
	std::ifstream f;
	f.open(input_file_name.c_str());
	if (f.good())
	{
		std::cout << "parsing file: " << input_file_name << std::endl;

		std::string s;
		getline(f, s);
		while (!f.eof())
		{
			getline(f, s);

			// underground_mining::Intersection intersection;
			std::string name;
			double x;
			double y;
			double z;

			stringstream ss(s);
			ss >> name;
			ss >> x;
			ss >> y;
			ss >> z;

			GeoPoint geopoint;
			geopoint.choosen = false;
			geopoint.coordinates.x() = x;
			geopoint.coordinates.y() = y;
			geopoint.coordinates.z() = z;
			geopoint.name = name;

			std::cout << "adding geo point: " << geopoint.name << " " << geopoint.coordinates.x() << " " << geopoint.coordinates.y() << " " << geopoint.coordinates.z() << std::endl;

			available_geo_points.push_back(geopoint);
		}
		f.close();

		auto geo = available_geo_points;
		for (auto& g : geo)
		{
			g.coordinates -= session.point_clouds_container.offset;
		}
		for (auto& p : session.point_clouds_container.point_clouds)
		{
			p.available_geo_points = geo;
		}
	}
}*/

void save_scale_board_to_laz(const Session& session, std::string output_file_name, float dec, float side_len)
{
	std::vector<Eigen::Vector3d> pointcloud;
	std::vector<unsigned short> intensity;
	std::vector<double> timestamps;

	float min_x = 1000000000.0;
	float max_x = -1000000000.0;
	float min_y = 1000000000.0;
	float max_y = -1000000000.0;
	float min_z = 1000000000.0;
	float max_z = -1000000000.0;
	float xy_incr = 0.001;
	float z = 0.0;
	if (side_len < 0.0)
	{
		for (auto& p : session.point_clouds_container.point_clouds)
		{
			if (p.visible)
			{
				for (int i = 0; i < p.local_trajectory.size(); i++)
				{
					const auto& pp = p.local_trajectory[i].m_pose.translation();
					Eigen::Vector3d vp;
					vp = p.m_pose * pp; 
					if (vp.x() < min_x)
					{
						min_x = vp.x();
					}
					if (vp.x() > max_x)
					{
						max_x = vp.x();
					}
					if (vp.y() < min_y)
					{
						min_y = vp.y();
					}
					if (vp.y() > max_y)
					{
						max_y = vp.y();
					}
					if (vp.z() < min_z)
					{
						min_z = vp.z();
					}
					if (vp.z() > max_z)
					{
						max_z = vp.z();
					}
				}
			}
		}
		min_x -= 100.0;
		min_y -= 100.0;
		max_x += 100.0;
		max_y += 100.0;
		z = (max_z + min_z) / 2.0;
	}
	else
	{
		min_x = -side_len / 2.0;
		min_y = -side_len / 2.0;
		max_x = side_len / 2.0;
		max_y = side_len / 2.0;
		xy_incr = 0.2;
		z = 0.0;
	}
	
	for (float x = min_x; x <= max_x; x += dec)
	{
		for (float y = min_y; y <= max_y; y += xy_incr)
		{
			Eigen::Vector3d vp(x, y, z);
			pointcloud.push_back(vp);
			intensity.push_back(0);
			timestamps.push_back(0.0);
		}
	}

	for (float y = min_y; y <= max_y; y += dec)
	{
		for (float x = min_x; x <= max_x; x += xy_incr)
		{
			Eigen::Vector3d vp(x, y, z);
			pointcloud.push_back(vp);
			intensity.push_back(0);
			timestamps.push_back(0.0);
		}
	}

	if (!exportLaz(output_file_name, pointcloud, intensity, timestamps, session.point_clouds_container.offset.x(), session.point_clouds_container.offset.y(), session.point_clouds_container.offset.z()))
	{
		std::cout << "problem with saving file: " << output_file_name << std::endl;
	}
}

std::vector<std::string> get_matching_files(const std::string& directory, const std::string& pattern) {
    std::vector<std::string> matching_files;
    std::regex regex_pattern(pattern); 
    try {
        for (const auto& entry : fs::directory_iterator(directory)) {
            if (entry.is_regular_file()) { // Ensure it's a regular file
                const std::string filename = entry.path().filename().string();
                if (std::regex_match(filename, regex_pattern)) {
                    matching_files.push_back(entry.path().string());
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error accessing directory: " << e.what() << std::endl;
    }

    return matching_files;
}

void run_multi_view_tls_registration(
	std::string input_file_name, TLSRegistration& tls_registration, std::string output_dir)
{
	fs::path outwd = fs::path(output_dir);
	Session session;
	if (!fs::exists(input_file_name))
	{
		std::cout << "Provided input path does not exist." << std::endl;
		return;
	}
	if (has_extension(input_file_name, ".json")) 
	{
		std::cout << "Session file: '" << input_file_name << "'" << std::endl;
		session.working_directory = fs::path(input_file_name).parent_path().string();
		session.load(fs::path(input_file_name).string(), tls_registration.is_decimate, tls_registration.bucket_x, tls_registration.bucket_y, tls_registration.bucket_z, tls_registration.calculate_offset);
	}
	else if (has_extension(input_file_name, ".reg"))
	{
		std::cout << "RESSO file: '" << input_file_name << "'" << std::endl;
		session.working_directory = fs::path(input_file_name).parent_path().string();
		session.point_clouds_container.load(session.working_directory.c_str(), input_file_name.c_str(), tls_registration.is_decimate, tls_registration.bucket_x, tls_registration.bucket_y, tls_registration.bucket_z);
	}
	else if (has_extension(input_file_name, ".txt"))
	{
		std::cout << "ETH file: '" << input_file_name << "'" << std::endl;
		session.working_directory = fs::path(input_file_name).parent_path().string();
		session.point_clouds_container.load_eth(session.working_directory.c_str(), input_file_name.c_str(), tls_registration.is_decimate, tls_registration.bucket_x, tls_registration.bucket_y, tls_registration.bucket_z);
	}
	else if (!has_extension(input_file_name, ""))
	{
		session.point_clouds_container.point_clouds.clear();
		session.working_directory = fs::path(input_file_name).string();
		std::vector<std::string> las_files;
		std::vector<std::string> txt_files; 
		for (const auto& entry : std::filesystem::directory_iterator(input_file_name)) {
			auto file_name = entry.path().string();
			if (has_extension(file_name, ".laz") || (has_extension(file_name, ".las"))) 
			{
				las_files.push_back(file_name);
			}
			else if (has_extension(file_name, ".txt"))
			{
				txt_files.push_back(file_name);
			}
		}
		if (las_files.size() > 0)
		{
			std::cout << "Las/Laz files:" << std::endl;
			for (size_t i = 0; i < las_files.size(); i++)
			{
				std::cout << las_files[i] << std::endl;
			}
			session.point_clouds_container.load_whu_tls(las_files, tls_registration.is_decimate, tls_registration.bucket_x, tls_registration.bucket_y, tls_registration.bucket_z, tls_registration.calculate_offset);
		}
		else if (txt_files.size() > 0)
		{
			std::cout << "txt files:" << std::endl;
			for (size_t i = 0; i < txt_files.size(); i++)
			{
				std::cout << txt_files[i] << std::endl;
			}
			session.point_clouds_container.load_3DTK_tls(txt_files, tls_registration.is_decimate, tls_registration.bucket_x, tls_registration.bucket_y, tls_registration.bucket_z);
		}
		else
		{
			std::cout << "No WHU-TLS / 3DTK files available in the given directory, check path." << std::endl;
			return;
		}
	}
	else
	{
		std::cout << "Session file: '" << input_file_name << "'" << std::endl;
		session.load(fs::path(input_file_name).string(), tls_registration.is_decimate, tls_registration.bucket_x, tls_registration.bucket_y, tls_registration.bucket_z, tls_registration.calculate_offset);
	}
	std::cout << "loaded: " << session.point_clouds_container.point_clouds.size() << " point_clouds" << std::endl;

	int number_of_point = 0;
	for (const auto& pc : session.point_clouds_container.point_clouds)
	{
		number_of_point += pc.points_local.size();
	}
	session.point_clouds_container.print_point_cloud_dimention();

	if (tls_registration.resso_upd_init.size() > 0)
	{
		std::cout << "RESSO file: '" << tls_registration.resso_upd_init << "'" << std::endl;
		session.working_directory = fs::path(tls_registration.resso_upd_init).parent_path().string();
		if (!session.point_clouds_container.update_initial_poses_from_RESSO(session.working_directory.c_str(), tls_registration.resso_upd_init.c_str()))
		{
			std::cout << "check input files" << std::endl;
			return;
		}
		else
		{
			session.point_clouds_container.initial_poses_file_name = tls_registration.resso_upd_init;
			std::cout << "updated: " << session.point_clouds_container.point_clouds.size() << " point_clouds" << std::endl;
		}
	}
	if (tls_registration.resso_upd.size() > 0)
	{
		std::cout << "RESSO file: '" << tls_registration.resso_upd << "'" << std::endl;
		session.working_directory = fs::path(tls_registration.resso_upd).parent_path().string();
		if (!session.point_clouds_container.update_poses_from_RESSO(session.working_directory.c_str(), tls_registration.resso_upd.c_str()))
		{
			std::cout << "check input files" << std::endl;
			return;
		}
		else
		{
			std::cout << "updated: " << session.point_clouds_container.point_clouds.size() << " point_clouds" << std::endl;
			session.point_clouds_container.poses_file_name = tls_registration.resso_upd;
		}
	}
	if (tls_registration.resso_upd_inv.size() > 0)
	{
		std::cout << "RESSO file: '" << tls_registration.resso_upd_inv << "'" << std::endl;
		session.working_directory = fs::path(tls_registration.resso_upd_inv).parent_path().string();
		if (!session.point_clouds_container.update_poses_from_RESSO_inverse(session.working_directory.c_str(), tls_registration.resso_upd_inv.c_str()))
		{
			std::cout << "check input files" << std::endl;
			return;
		}
		else
		{
			std::cout << "updated: " << session.point_clouds_container.point_clouds.size() << " point_clouds" << std::endl;
			session.point_clouds_container.poses_file_name = tls_registration.resso_upd_inv;
		}
	}

	if (tls_registration.initial_pose_to_identity)
	{
		initial_pose_to_identity(session);
	}

	if (tls_registration.use_ndt)
	{
		if (tls_registration.compute_only_mahalanobis_distance)
		{
			tls_registration.ndt.optimize(session.point_clouds_container.point_clouds, true, tls_registration.compute_mean_and_cov_for_bucket);
		}
		else if (tls_registration.use_lie_algebra_left_jacobian_ndt)
		{
			tls_registration.ndt.optimize_lie_algebra_left_jacobian(session.point_clouds_container.point_clouds, tls_registration.compute_mean_and_cov_for_bucket);
		}
		else if (tls_registration.use_lie_algebra_right_jacobian_ndt)
		{
			tls_registration.ndt.optimize_lie_algebra_right_jacobian(session.point_clouds_container.point_clouds, tls_registration.compute_mean_and_cov_for_bucket);
		}
		else
		{
			std::cout << "No additional optimization option selected for NDT: using default..." << std::endl;
			tls_registration.ndt.optimize(session.point_clouds_container.point_clouds, false, tls_registration.compute_mean_and_cov_for_bucket);
		}
	}

	if (tls_registration.use_icp)
	{
		if (tls_registration.point_to_point_source_to_target)
		{
			tls_registration.icp.optimization_point_to_point_source_to_target(session.point_clouds_container);
		}
		else if (tls_registration.use_lie_algebra_left_jacobian_icp)
		{
			tls_registration.icp.optimize_source_to_target_lie_algebra_left_jacobian(session.point_clouds_container);
		}
		else if (tls_registration.use_lie_algebra_right_jacobian_icp)
		{
			tls_registration.icp.optimize_source_to_target_lie_algebra_right_jacobian(session.point_clouds_container);
		}
		else if (tls_registration.point_to_point_source_to_target_compute_rms)
		{
			double rms = 0.0;
			tls_registration.icp.optimization_point_to_point_source_to_target_compute_rms(session.point_clouds_container, rms);
			std::cout << "rms(optimization_point_to_point_source_to_target): " << rms << std::endl;
		}
		else
		{
			std::cout << "No optimization option selected for ICP: skipping..." << std::endl;
		}
	}

	if (tls_registration.use_plane_features)
	{
		if (tls_registration.point_to_projection_onto_plane_source_to_target)
		{
			tls_registration.registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target(session.point_clouds_container);
		}
		else if (tls_registration.use_lie_algebra_left_jacobian_plane_features)
		{
			tls_registration.registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_left_jacobian(session.point_clouds_container);
		}
		else if (tls_registration.use_lie_algebra_right_jacobian_plane_features)
		{
			tls_registration.registration_plane_feature.optimize_point_to_projection_onto_plane_source_to_target_lie_algebra_right_jacobian(session.point_clouds_container);
		}
		else if (tls_registration.point_to_plane_source_to_target_dot_product)
		{
			tls_registration.registration_plane_feature.optimize_point_to_plane_source_to_target(session.point_clouds_container);
		}
		else if (tls_registration.point_to_plane_source_to_target)
		{
			tls_registration.registration_plane_feature.optimize_distance_point_to_plane_source_to_target(session.point_clouds_container);
		}
		else if (tls_registration.plane_to_plane_source_to_target)
		{
			tls_registration.registration_plane_feature.optimize_plane_to_plane_source_to_target(session.point_clouds_container);
		}
		else
		{
			std::cout << "No optimization option selected for the plane features: skipping..." << std::endl;
		}
	}

	// TODO: add with GTSAM and with MANIF
	if (tls_registration.use_pgslam)
	{
		tls_registration.pose_graph_slam.ndt_bucket_size[0] = tls_registration.ndt.bucket_size[0];
		tls_registration.pose_graph_slam.ndt_bucket_size[1] = tls_registration.ndt.bucket_size[1];
		tls_registration.pose_graph_slam.ndt_bucket_size[2] = tls_registration.ndt.bucket_size[2];
		tls_registration.pose_graph_slam.optimize(session.point_clouds_container);
	}

	if (output_dir.length() > 0)
	{
		if (session.point_clouds_container.initial_poses_file_name.empty() && tls_registration.save_initial_poses)
		{
			std::string initial_poses_file_name = (outwd / "initial_poses.reg").string();
			std::cout << "saving initial poses to: " << initial_poses_file_name << std::endl;
			session.point_clouds_container.save_poses(initial_poses_file_name, false);
		}

		if (session.point_clouds_container.poses_file_name.empty() && tls_registration.save_poses)
		{
			std::string poses_file_name = (outwd / "poses.reg").string();
			std::cout << "saving poses to: " << poses_file_name << std::endl;
			session.point_clouds_container.save_poses(poses_file_name, false);
		}
		session.save(
			(outwd / "session_step_2.json").string(), session.point_clouds_container.poses_file_name, 
			session.point_clouds_container.initial_poses_file_name, false);
		std::cout << "saving result to: " << session.point_clouds_container.poses_file_name << std::endl;
		session.point_clouds_container.save_poses(fs::path(session.point_clouds_container.poses_file_name).string(), false);
	}

	if (tls_registration.save_laz)
	{
		save_all_to_las(session, (outwd / "all_step_2.laz").string());
	}
	if (tls_registration.save_las)
	{
		save_all_to_las(session, (outwd / "all_step_2.las").string());
	}
	if (tls_registration.save_as_separate_las)
	{
		save_separately_to_las(session, outwd, ".las");
	}
	if (tls_registration.save_as_separate_laz)
	{
		save_separately_to_las(session, outwd, ".laz");
	}

	if (tls_registration.save_trajectories_laz)
	{
		save_trajectories_to_laz(session, (outwd / "trajectories.laz").string(), tls_registration.curve_consecutive_distance_meters, 
		tls_registration.not_curve_consecutive_distance_meters, tls_registration.is_trajectory_export_downsampling);
	}

	if (tls_registration.save_gnss_laz)
	{
		tls_registration.gnss.save_to_laz((outwd / "gnss.laz").string(), session.point_clouds_container.offset.x(), 
		session.point_clouds_container.offset.y(), session.point_clouds_container.offset.z());
	}

	if (tls_registration.save_scale_board_laz)
	{
		save_scale_board_to_laz(session, (outwd / "scale_board.laz").string(), tls_registration.scale_board_dec, tls_registration.scale_board_side_len);
	}

	if (tls_registration.save_trajectories_csv)
	{
		save_trajectories(
			session, (outwd / "trajectories.csv").string(), tls_registration.curve_consecutive_distance_meters,
			tls_registration.not_curve_consecutive_distance_meters, tls_registration.is_trajectory_export_downsampling,
			tls_registration.write_lidar_timestamp, tls_registration.write_unix_timestamp,
			tls_registration.use_quaternions, false);
	}
	if (tls_registration.save_trajectories_dxf)
	{
		save_trajectories(
			session, (outwd / "trajectories.dxf").string(), tls_registration.curve_consecutive_distance_meters,
			tls_registration.not_curve_consecutive_distance_meters, tls_registration.is_trajectory_export_downsampling,
			tls_registration.write_lidar_timestamp, tls_registration.write_unix_timestamp,
			tls_registration.use_quaternions, true);
	}
}
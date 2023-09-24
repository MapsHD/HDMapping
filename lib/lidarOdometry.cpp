#include "lidarOdometry.h"
#include "lidarOdomIO.h"
namespace mandeye
{

	void optimizeTrajectory(const InputImuData& imu_data, const InputPointCloudData& inputPoints, const SlamConfig& config)
	{
		std::cout << "imu_data " << imu_data.size() << std::endl;
		std::cout << "inputPoints " << inputPoints.size() << std::endl;

		// globals
		Eigen::Affine3d m_g = Eigen::Affine3d::Identity(); //!< Matrix Global
		std::vector<Point3Di> initial_points;
		NDTBucketMapType buckets;
		NDTBucketMapType reference_buckets;
		std::vector<WorkerData> worker_data;
		NDT::GridParameters in_out_params;
		double consecutive_distance = 0.0;

		std::vector<Point3Di> reference_points;

		in_out_params.resolution_X = 0.3;
		in_out_params.resolution_Y = 0.3;
		in_out_params.resolution_Z = 0.3;
		in_out_params.bounding_box_extension = 20.0;

		// step 1;
		const bool quiet = config.quiet;
		if (imu_data.empty())
		{
			std::cerr << "imu_data empty";
			return;
		}
		if (inputPoints.size() < config.threshold_initial_points)
		{
			std::cerr << "initialPoints to small : " << config.threshold_initial_points << " was not met, "<< inputPoints.size();
			return;
		}


		// run fusion
		FusionAhrs ahrs;
		FusionAhrsInitialise(&ahrs);
		int chosenConventions = 0;
		if (config.fusionConventionNwu)
		{
			ahrs.settings.convention = FusionConventionNwu;
			chosenConventions++;
		}
		if (config.fusionConventionEnu)
		{
			ahrs.settings.convention = FusionConventionEnu;
			chosenConventions++;
		}
		if (config.fusionConventionNed)
		{
			ahrs.settings.convention = FusionConventionNed;
			chosenConventions++;
		}

		if (chosenConventions != 1)
		{
			std::cerr << "You need to choose exactly one IMU convention";
			return;
		}

		std::map<double, Eigen::Matrix4d> trajectory;

		for (const auto& [timestamp, gyr, acc] : imu_data)
		{
			const FusionVector gyroscope = { static_cast<float>(gyr.axis.x * 180.0 / M_PI), static_cast<float>(gyr.axis.y * 180.0 / M_PI), static_cast<float>(gyr.axis.z * 180.0 / M_PI) };
			const FusionVector accelerometer = { acc.axis.x, acc.axis.y, acc.axis.z };

			FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, config.sample_peroid);

			FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);

			Eigen::Quaterniond d{ quat.element.w, quat.element.x, quat.element.y, quat.element.z };
			Eigen::Affine3d t{ Eigen::Matrix4d::Identity() };
			t.rotate(d);

			//
			// TaitBryanPose rot_y;
			// rot_y.px = rot_y.py = rot_y.pz = rot_y.px = rot_y.py = rot_y.pz;
			// rot_y.fi = -5 * M_PI / 180.0;
			// Eigen::Affine3d m_rot_y = affine_matrix_from_pose_tait_bryan(rot_y);
			// t = t * m_rot_y;
			//

			trajectory[timestamp] = t.matrix();
			const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
			if (trajectory.size() % 100 == 0)
			{
				if (!quiet)printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f [%d of %d]\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw, trajectory.size(), imu_data.size());
			}
		}

		int number_of_initial_points = 0;
		double timestamp_begin;
	
		// number_of_points += pp.size();
		for (const auto& p : inputPoints)
		{
			number_of_initial_points++;
			initial_points.push_back(p);
			if (number_of_initial_points > config.threshold_initial_points)
			{
				timestamp_begin = p.timestamp;
				break;
			}
		}

		std::cout << "timestamp_begin: " << timestamp_begin << std::endl;

		//ToDo (mpelka) - refactor
		std::vector<double> timestamps;
		std::vector<Eigen::Affine3d> poses;
		for (const auto& t : trajectory)
		{
			if (t.first >= timestamp_begin)
			{
				timestamps.push_back(t.first);
				Eigen::Affine3d m;
				m.matrix() = t.second;
				poses.push_back(m);
			}
		}
		
		std::cout << "poses.size(): " << poses.size() << std::endl;


		int thershold = 20;
		WorkerData wd;
		int index_begin = 0;

		for (size_t i = 0; i < poses.size(); i++)
		{
			if (i % 1000 == 0)
			{
				std::cout << "preparing data " << i + 1 << " of " << poses.size() << std::endl;
			}
			wd.intermediate_trajectory.emplace_back(poses[i]);
			wd.intermediate_trajectory_motion_model.emplace_back(poses[i]);
			wd.intermediate_trajectory_timestamps.emplace_back(timestamps[i]);

			//
			TaitBryanPose tb = pose_tait_bryan_from_affine_matrix(poses[i]);
			wd.imu_roll_pitch.emplace_back(tb.om, tb.fi);

			// temp_ts.emplace_back(timestamps[i]);

			if (wd.intermediate_trajectory.size() >= thershold)
			{
				/*auto index_lower = std::lower_bound(points.begin(), points.end(), wd.intermediate_trajectory_timestamps[0],
													[](Point3Di lhs, double rhs) -> bool
													{ return lhs.timestamp < rhs; });
				unsigned long long int i_begin = std::distance(points.begin(), index_lower);

				auto index_upper = std::lower_bound(points.begin(), points.end(), wd.intermediate_trajectory_timestamps[wd.intermediate_trajectory_timestamps.size() - 1],
													[](Point3Di lhs, double rhs) -> bool
													{ return lhs.timestamp < rhs; });
				unsigned long long int i_end = std::distance(points.begin(), index_upper);*/

				std::vector<Point3Di> points;

				for (const auto& p : inputPoints)
				{
					if (p.timestamp >= wd.intermediate_trajectory_timestamps.front() && p.timestamp <= wd.intermediate_trajectory_timestamps.back())
					{
						points.push_back(p);
					}
					if (p.timestamp > wd.intermediate_trajectory_timestamps.back())
					{
						break;
					}
				}

				// for (unsigned long long int k = i_begin; k < i_end; k++)
				// if (i % 1000 == 0)
				//{
				// std::cout << "points.size() " << points.size() << std::endl;
				//}

				for (unsigned long long int k = 0; k < points.size(); k++)
				{
					// if (points[k].timestamp > wd.intermediate_trajectory_timestamps[0] && points[k].timestamp < wd.intermediate_trajectory_timestamps[wd.intermediate_trajectory_timestamps.size() - 1])
					//{
					auto p = points[k];
					auto lower = std::lower_bound(wd.intermediate_trajectory_timestamps.begin(), wd.intermediate_trajectory_timestamps.end(), p.timestamp);
					p.index_pose = std::distance(wd.intermediate_trajectory_timestamps.begin(), lower);
					wd.intermediate_points.emplace_back(p);
					wd.original_points.emplace_back(p);
					//}
				}

				if (config.decimation > 0.0)
				{
					wd.intermediate_points = decimate(wd.intermediate_points, config.decimation, config.decimation, config.decimation);
				}

				worker_data.push_back(wd);
				wd.intermediate_points.clear();
				wd.original_points.clear();
				wd.intermediate_trajectory.clear();
				wd.intermediate_trajectory_motion_model.clear();
				wd.intermediate_trajectory_timestamps.clear();
				wd.imu_roll_pitch.clear();

				wd.intermediate_points.reserve(1000000);
				wd.original_points.reserve(1000000);
				wd.intermediate_trajectory.reserve(1000);
				wd.intermediate_trajectory_motion_model.reserve(1000);
				wd.intermediate_trajectory_timestamps.reserve(1000);
				wd.imu_roll_pitch.reserve(1000);

				// temp_ts.clear();
			}
		}
		m_g = worker_data[0].intermediate_trajectory[0];

		// step 2 
		if (worker_data.empty())
		{
			std::cout << "empty worker data" << std::endl;
			return;
		}

		std::chrono::time_point<std::chrono::system_clock> start, end;
		start = std::chrono::system_clock::now();
		double acc_distance = 0.0;
		std::vector<Point3Di> points_global;

		Eigen::Affine3d m_last = m_g;
		auto tmp = worker_data[0].intermediate_trajectory;

		worker_data[0].intermediate_trajectory[0] = m_last;
		for (int k = 1; k < tmp.size(); k++)
		{
			Eigen::Affine3d m_update = tmp[k - 1].inverse() * tmp[k];
			m_last = m_last * m_update;
			worker_data[0].intermediate_trajectory[k] = m_last;
		}
		worker_data[0].intermediate_trajectory_motion_model = worker_data[0].intermediate_trajectory;

		auto pp = initial_points;
		for (int i = 0; i < pp.size(); i++)
		{
			pp[i].point = m_g * pp[i].point;
		}
		
		update_rgd(in_out_params, buckets, pp);

		for (int i = 0; i < worker_data.size(); i++)
		{
			//std::cout << "computing worker_data [" << i + 1 << "] of " << worker_data.size() << " acc_distance: " << acc_distance << std::endl;
			
			Eigen::Vector3d mean_shift(0.0, 0.0, 0.0);
			if (i > 1 && config.use_motion_from_previous_step)
			{
				Eigen::Affine3d m_relative = worker_data[i - 2].intermediate_trajectory[worker_data[i - 2].intermediate_trajectory.size() - 1].inverse() *
					worker_data[i - 1].intermediate_trajectory[0];

				mean_shift /= (worker_data[i].intermediate_trajectory.size());

				if (mean_shift.norm() > 1.0)
				{
					mean_shift = Eigen::Vector3d(1.0, 1.0, 1.0);
				}

				Eigen::Affine3d m_mean_shift = Eigen::Affine3d::Identity();
				m_mean_shift.translation() = mean_shift;

				std::vector<Eigen::Affine3d> new_trajectory;
				Eigen::Affine3d current_node = worker_data[i].intermediate_trajectory[0];
				new_trajectory.push_back(current_node);

				for (int tr = 1; tr < worker_data[i].intermediate_trajectory.size(); tr++)
				{
					current_node = current_node * (worker_data[i].intermediate_trajectory[tr - 1].inverse() * worker_data[i].intermediate_trajectory[tr]);
					current_node = current_node * m_mean_shift;
					new_trajectory.push_back(current_node);
				}

				worker_data[i].intermediate_trajectory = new_trajectory;
				////////////////////////////////////////////////////////////////////////
				std::vector<Eigen::Affine3d> new_trajectory_motion_model;
				Eigen::Affine3d current_node_motion_model = worker_data[i].intermediate_trajectory_motion_model[0];
				new_trajectory_motion_model.push_back(current_node_motion_model);

				for (int tr = 1; tr < worker_data[i].intermediate_trajectory_motion_model.size(); tr++)
				{
					current_node_motion_model = current_node_motion_model * (worker_data[i].intermediate_trajectory_motion_model[tr - 1].inverse() * worker_data[i].intermediate_trajectory_motion_model[tr]);
					current_node_motion_model = current_node_motion_model * m_mean_shift;
					new_trajectory_motion_model.push_back(current_node_motion_model);
				}

				worker_data[i].intermediate_trajectory_motion_model = new_trajectory_motion_model;
			}

			bool add_pitch_roll_constraint = false;
			TaitBryanPose pose;
			pose = pose_tait_bryan_from_affine_matrix(worker_data[i].intermediate_trajectory[0]);

			double residual1;
			double residual2;
			residual_constraint_fixed_optimization_parameter(residual1, normalize_angle(worker_data[i].imu_roll_pitch[0].first), normalize_angle(pose.om));
			residual_constraint_fixed_optimization_parameter(residual2, normalize_angle(worker_data[i].imu_roll_pitch[0].second), normalize_angle(pose.fi));

			if (fabs(worker_data[i].imu_roll_pitch[0].first) < 30.0 / 180.0 * M_PI && fabs(worker_data[i].imu_roll_pitch[0].second) < 30.0 / 180.0 * M_PI)
			{
				if (consecutive_distance > 10.0)
				{
					add_pitch_roll_constraint = true;
					consecutive_distance = 0.0;
				}
			}

			if (add_pitch_roll_constraint)
			{
				std::cout << "residual_imu_roll_deg before: " << residual1 / M_PI * 180.0 << std::endl;
				std::cout << "residual_imu_pitch_deg before: " << residual2 / M_PI * 180.0 << std::endl;
			}

			std::chrono::time_point<std::chrono::system_clock> start1, end1;
			start1 = std::chrono::system_clock::now();

			for (int iter = 0; iter < config.nr_iter; iter++)
			{
				optimize(worker_data[i].intermediate_points, worker_data[i].intermediate_trajectory, worker_data[i].intermediate_trajectory_motion_model,
					in_out_params, buckets, config.useMultithread, add_pitch_roll_constraint, worker_data[i].imu_roll_pitch);
			}
			end1 = std::chrono::system_clock::now();
			std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
			std::cout << "optimizing worker_data [" << i + 1 << "] of " << worker_data.size() << " acc_distance: " << acc_distance << " elapsed time: " << elapsed_seconds1.count() << std::endl;

			if (add_pitch_roll_constraint)
			{
				pose = pose_tait_bryan_from_affine_matrix(worker_data[i].intermediate_trajectory[0]);

				residual_constraint_fixed_optimization_parameter(residual1, normalize_angle(worker_data[i].imu_roll_pitch[0].first), normalize_angle(pose.om));
				residual_constraint_fixed_optimization_parameter(residual2, normalize_angle(worker_data[i].imu_roll_pitch[0].second), normalize_angle(pose.fi));

				std::cout << "residual_imu_roll_deg after: " << residual1 / M_PI * 180.0 << std::endl;
				std::cout << "residual_imu_pitch_deg after: " << residual2 / M_PI * 180.0 << std::endl;
			}

			// align to reference
			if (reference_points.size() > 0)
			{
				std::cout << "align to reference" << std::endl;
				Eigen::Affine3d m_first = worker_data[i].intermediate_trajectory[0];
				Eigen::Affine3d m_first_inv = m_first.inverse();

				// create rigid scan
				std::vector<Point3Di> local_points;
				for (int k = 0; k < worker_data[i].intermediate_points.size(); k++)
				{
					Point3Di p = worker_data[i].intermediate_points[k];
					int index_pose = p.index_pose;
					p.point = worker_data[i].intermediate_trajectory[index_pose] * p.point;
					p.point = m_first_inv * p.point;
					local_points.push_back(p);
				}
				// std::cout << "before " << m_first.matrix() << std::endl;
				if (config.decimation > 0)
				{
					local_points = decimate(local_points, config.decimation, config.decimation, config.decimation);
				}
				for (int iter = 0; iter < config.nr_iter; iter++)
				{
					align_to_reference(in_out_params, local_points, m_first, reference_buckets);
				}

				auto tmp = worker_data[i].intermediate_trajectory;

				worker_data[i].intermediate_trajectory[0] = m_first;

				for (int k = 1; k < tmp.size(); k++)
				{
					Eigen::Affine3d m_update = tmp[k - 1].inverse() * tmp[k];
					m_first = m_first * m_update;
					worker_data[i].intermediate_trajectory[k] = m_first;
				}
				worker_data[i].intermediate_trajectory_motion_model = worker_data[i].intermediate_trajectory;
			}

			// temp save
			if (!config.tempSave.empty() && i % 100 == 0)
			{
				std::vector<Point3Di> global_points;
				for (int k = 0; k < worker_data[i].intermediate_points.size(); k++)
				{
					Point3Di p = worker_data[i].intermediate_points[k];
					int index_pose = p.index_pose;
					p.point = worker_data[i].intermediate_trajectory[index_pose] * p.point;
					global_points.push_back(p);
				}
				std::string fn = config.tempSave + "/temp_point_cloud_" + std::to_string(i) + ".laz";
				mandeye::utilsIO::saveLaz(fn.c_str(), global_points);
			}
			acc_distance += ((worker_data[i].intermediate_trajectory[0].inverse()) *
				worker_data[i].intermediate_trajectory[worker_data[i].intermediate_trajectory.size() - 1])
				.translation()
				.norm();

			// update
			for (int j = i + 1; j < worker_data.size(); j++)
			{
				Eigen::Affine3d m_last = worker_data[j - 1].intermediate_trajectory[worker_data[j - 1].intermediate_trajectory.size() - 1];
				auto tmp = worker_data[j].intermediate_trajectory;

				worker_data[j].intermediate_trajectory[0] = m_last;
				for (int k = 1; k < tmp.size(); k++)
				{
					Eigen::Affine3d m_update = tmp[k - 1].inverse() * tmp[k];
					m_last = m_last * m_update;
					worker_data[j].intermediate_trajectory[k] = m_last;
				}
			}

			for (int j = 0; j < worker_data[i].intermediate_points.size(); j++)
			{
				Point3Di pp = worker_data[i].intermediate_points[j];
				pp.point = worker_data[i].intermediate_trajectory[worker_data[i].intermediate_points[j].index_pose] * pp.point;
				points_global.push_back(pp);
			}

			// if(reference_points.size() == 0){
			if (acc_distance > config.sliding_window_trajectory_length_threshold)
			{
				std::chrono::time_point<std::chrono::system_clock> startu, endu;
				startu = std::chrono::system_clock::now();

				if (reference_points.size() == 0)
				{
					buckets.clear();
				}

				std::vector<Point3Di> points_global_new;
				points_global_new.reserve(points_global.size() / 2 + 1);
				for (int k = points_global.size() / 2; k < points_global.size(); k++)
				{
					points_global_new.emplace_back(points_global[k]);
				}

				acc_distance = 0;
				points_global = points_global_new;

				// decimate
				if (config.decimation > 0)
				{
					decimate(points_global, config.decimation, config.decimation, config.decimation);
				}
				update_rgd(in_out_params, buckets, points_global);
				//
				endu = std::chrono::system_clock::now();

				std::chrono::duration<double> elapsed_secondsu = endu - startu;
				std::time_t end_timeu = std::chrono::system_clock::to_time_t(endu);

				std::cout << "finished computation at " << std::ctime(&end_timeu)
					<< "elapsed time update: " << elapsed_secondsu.count() << "s\n";
				// std::cout << "update" << std::endl;
			}
			else
			{
				std::vector<Point3Di> pg;
				for (int j = 0; j < worker_data[i].intermediate_points.size(); j++)
				{
					Point3Di pp = worker_data[i].intermediate_points[j];
					pp.point = worker_data[i].intermediate_trajectory[worker_data[i].intermediate_points[j].index_pose] * pp.point;
					pg.push_back(pp);
				}
				update_rgd(in_out_params, buckets, pg);
			}

			if (i > 1)
			{
				double translation = (worker_data[i - 1].intermediate_trajectory[0].translation() -
					worker_data[i - 2].intermediate_trajectory[0].translation())
					.norm();
				consecutive_distance += translation;
				// std::cout << "consecutive_distance " << consecutive_distance << std::endl;
			}
			//}
		}

		for (int i = 0; i < worker_data.size(); i++)
		{
			worker_data[i].intermediate_trajectory_motion_model = worker_data[i].intermediate_trajectory;
		}

		end = std::chrono::system_clock::now();

		std::chrono::duration<double> elapsed_seconds = end - start;
		std::time_t end_time = std::chrono::system_clock::to_time_t(end);

		std::cout << "finished computation at " << std::ctime(&end_time)
			<< "elapsed time: " << elapsed_seconds.count() << "s\n";

		// estimate total lenght of trajectory
		double length_of_trajectory = 0;
		for (int i = 1; i < worker_data.size(); i++)
		{
			length_of_trajectory += (worker_data[i].intermediate_trajectory[0].translation() - worker_data[i - 1].intermediate_trajectory[0].translation()).norm();
		}
		std::cout << "length_of_trajectory: " << length_of_trajectory << " [m]" << std::endl;
	}

	Eigen::Matrix4d getInterpolatedPose(const std::map<double, Eigen::Matrix4d>& trajectory, double query_time)
	{
		Eigen::Matrix4d ret(Eigen::Matrix4d::Zero());
		auto it_lower = trajectory.lower_bound(query_time);
		auto it_next = it_lower;
		if (it_lower == trajectory.begin())
		{
			return ret;
		}
		if (it_lower->first > query_time)
		{
			it_lower = std::prev(it_lower);
		}
		if (it_lower == trajectory.begin())
		{
			return ret;
		}
		if (it_lower == trajectory.end())
		{
			return ret;
		}

		double t1 = it_lower->first;
		double t2 = it_next->first;
		double difft1 = t1 - query_time;
		double difft2 = t2 - query_time;
		if (t1 == t2 && std::fabs(difft1) < 0.1)
		{
			ret = Eigen::Matrix4d::Identity();
			ret.col(3).head<3>() = it_next->second.col(3).head<3>();
			ret.topLeftCorner(3, 3) = it_lower->second.topLeftCorner(3, 3);
			return ret;
		}
		if (std::fabs(difft1) < 0.15 && std::fabs(difft2) < 0.15)
		{
			assert(t2 > t1);
			assert(query_time > t1);
			assert(query_time < t2);
			ret = Eigen::Matrix4d::Identity();
			double res = (query_time - t1) / (t2 - t1);
			Eigen::Vector3d diff = it_next->second.col(3).head<3>() - it_lower->second.col(3).head<3>();
			ret.col(3).head<3>() = it_next->second.col(3).head<3>() + diff * res;
			Eigen::Matrix3d r1 = it_lower->second.topLeftCorner(3, 3).matrix();
			Eigen::Matrix3d r2 = it_next->second.topLeftCorner(3, 3).matrix();
			Eigen::Quaterniond q1(r1);
			Eigen::Quaterniond q2(r2);
			Eigen::Quaterniond qt = q1.slerp(res, q2);
			ret.topLeftCorner(3, 3) = qt.toRotationMatrix();
			return ret;
		}
		// std::cout << "Problem with : " << difft1 << " " << difft2 << "  q : " << query_time << " t1 :" << t1 << " t2: " << t2 << std::endl;
		return ret;
	}

	std::vector<Point3Di> decimate(const std::vector<Point3Di>& points, double bucket_x, double bucket_y, double bucket_z)
	{
		// std::cout << "points.size before decimation: " << points.size() << std::endl;
		Eigen::Vector3d b(bucket_x, bucket_y, bucket_z);
		std::vector<Point3Di> out;

		std::vector<PointCloud::PointBucketIndexPair> ip;
		ip.resize(points.size());
		out.reserve(points.size());

		for (int i = 0; i < points.size(); i++)
		{
			ip[i].index_of_point = i;
			ip[i].index_of_bucket = get_rgd_index(points[i].point, b);
		}

		std::sort(ip.begin(), ip.end(), [](const PointCloud::PointBucketIndexPair& a, const PointCloud::PointBucketIndexPair& b)
			{ return a.index_of_bucket < b.index_of_bucket; });

		for (int i = 1; i < ip.size(); i++)
		{
			// std::cout << ip[i].index_of_bucket << " ";
			if (ip[i - 1].index_of_bucket != ip[i].index_of_bucket)
			{
				out.emplace_back(points[ip[i].index_of_point]);
			}
		}
		// std::cout << "points.size after decimation: " << out.size() << std::endl;
		return out;
	}

	void optimize(std::vector<Point3Di>& intermediate_points, std::vector<Eigen::Affine3d>& intermediate_trajectory,
		std::vector<Eigen::Affine3d>& intermediate_trajectory_motion_model,
		NDT::GridParameters& rgd_params, NDTBucketMapType& buckets, bool multithread,
		bool add_pitch_roll_constraint, const std::vector<std::pair<double, double>>& imu_roll_pitch)
	{
		std::vector<Eigen::Triplet<double>> tripletListA;
		std::vector<Eigen::Triplet<double>> tripletListP;
		std::vector<Eigen::Triplet<double>> tripletListB;

		Eigen::MatrixXd AtPAndt(intermediate_trajectory.size() * 6, intermediate_trajectory.size() * 6);
		AtPAndt.setZero();
		Eigen::MatrixXd AtPBndt(intermediate_trajectory.size() * 6, 1);
		AtPBndt.setZero();
		Eigen::Vector3d b(rgd_params.resolution_X, rgd_params.resolution_Y, rgd_params.resolution_Z);

		std::vector<std::mutex> mutexes(intermediate_trajectory.size());

		const auto hessian_fun = [&](const Point3Di& intermediate_points_i)
		{
			if (intermediate_points_i.point.norm() < 1.0)
			{
				return;
			}

			Eigen::Vector3d point_global = intermediate_trajectory[intermediate_points_i.index_pose] * intermediate_points_i.point;
			auto index_of_bucket = get_rgd_index(point_global, b);

			auto bucket_it = buckets.find(index_of_bucket);
			// no bucket found
			if (bucket_it == buckets.end())
			{
				return;
			}
			auto& this_bucket = bucket_it->second;

			// if(buckets[index_of_bucket].number_of_points >= 5){
			const Eigen::Matrix3d& infm = this_bucket.cov.inverse();
			const double threshold = 10000.0;

			if ((infm.array() > threshold).any())
			{
				return;
			}
			if ((infm.array() < -threshold).any())
			{
				return;
			}

			const Eigen::Affine3d& m_pose = intermediate_trajectory[intermediate_points_i.index_pose];
			const Eigen::Vector3d& p_s = intermediate_points_i.point;
			const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose);
			//

			Eigen::Matrix<double, 6, 6, Eigen::RowMajor> AtPA;
			point_to_point_source_to_target_tait_bryan_wc_AtPA_simplified(
				AtPA,
				pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
				p_s.x(), p_s.y(), p_s.z(),
				infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2));

			Eigen::Matrix<double, 6, 1> AtPB;
			point_to_point_source_to_target_tait_bryan_wc_AtPB_simplified(
				AtPB,
				pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
				p_s.x(), p_s.y(), p_s.z(),
				infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2),
				this_bucket.mean.x(), this_bucket.mean.y(), this_bucket.mean.z());

			int c = intermediate_points_i.index_pose * 6;

			std::mutex& m = mutexes[intermediate_points_i.index_pose];
			std::unique_lock lck(m);
			AtPAndt.block<6, 6>(c, c) += AtPA;
			AtPBndt.block<6, 1>(c, 0) -= AtPB;
		};

		if (multithread)
		{
			std::for_each(std::execution::par_unseq, std::begin(intermediate_points), std::end(intermediate_points), hessian_fun);
		}
		else
		{
			std::for_each(std::begin(intermediate_points), std::end(intermediate_points), hessian_fun);
		}
		std::vector<std::pair<int, int>> odo_edges;
		for (size_t i = 1; i < intermediate_trajectory.size(); i++)
		{
			odo_edges.emplace_back(i - 1, i);
		}

		std::vector<TaitBryanPose> poses;
		std::vector<TaitBryanPose> poses_desired;

		for (size_t i = 0; i < intermediate_trajectory.size(); i++)
		{
			poses.push_back(pose_tait_bryan_from_affine_matrix(intermediate_trajectory[i]));
		}
		for (size_t i = 0; i < intermediate_trajectory_motion_model.size(); i++)
		{
			poses_desired.push_back(pose_tait_bryan_from_affine_matrix(intermediate_trajectory_motion_model[i]));
		}

		/*for (size_t i = 0; i < odo_edges.size(); i++)
		{
			Eigen::Matrix<double, 6, 1> relative_pose_measurement_odo;
			relative_pose_tait_bryan_wc_case1(relative_pose_measurement_odo,
											  poses_desired[odo_edges[i].first].px,
											  poses_desired[odo_edges[i].first].py,
											  poses_desired[odo_edges[i].first].pz,
											  poses_desired[odo_edges[i].first].om,
											  poses_desired[odo_edges[i].first].fi,
											  poses_desired[odo_edges[i].first].ka,
											  poses_desired[odo_edges[i].second].px,
											  poses_desired[odo_edges[i].second].py,
											  poses_desired[odo_edges[i].second].pz,
											  poses_desired[odo_edges[i].second].om,
											  poses_desired[odo_edges[i].second].fi,
											  poses_desired[odo_edges[i].second].ka);

			Eigen::Matrix<double, 6, 1> delta;
			relative_pose_obs_eq_tait_bryan_wc_case1(
				delta,
				poses[odo_edges[i].first].px,
				poses[odo_edges[i].first].py,
				poses[odo_edges[i].first].pz,
				poses[odo_edges[i].first].om,
				poses[odo_edges[i].first].fi,
				poses[odo_edges[i].first].ka,
				poses[odo_edges[i].second].px,
				poses[odo_edges[i].second].py,
				poses[odo_edges[i].second].pz,
				poses[odo_edges[i].second].om,
				poses[odo_edges[i].second].fi,
				poses[odo_edges[i].second].ka,
				relative_pose_measurement_odo(0, 0),
				relative_pose_measurement_odo(1, 0),
				relative_pose_measurement_odo(2, 0),
				relative_pose_measurement_odo(3, 0),
				relative_pose_measurement_odo(4, 0),
				relative_pose_measurement_odo(5, 0));

			Eigen::Matrix<double, 6, 12, Eigen::RowMajor> jacobian;
			relative_pose_obs_eq_tait_bryan_wc_case1_jacobian(jacobian,
															  poses[odo_edges[i].first].px,
															  poses[odo_edges[i].first].py,
															  poses[odo_edges[i].first].pz,
															  poses[odo_edges[i].first].om,
															  poses[odo_edges[i].first].fi,
															  poses[odo_edges[i].first].ka,
															  poses[odo_edges[i].second].px,
															  poses[odo_edges[i].second].py,
															  poses[odo_edges[i].second].pz,
															  poses[odo_edges[i].second].om,
															  poses[odo_edges[i].second].fi,
															  poses[odo_edges[i].second].ka);

			int ir = tripletListB.size();

			int ic_1 = odo_edges[i].first * 6;
			int ic_2 = odo_edges[i].second * 6;

			for (size_t row = 0; row < 6; row++)
			{
				tripletListA.emplace_back(ir + row, ic_1, -jacobian(row, 0));
				tripletListA.emplace_back(ir + row, ic_1 + 1, -jacobian(row, 1));
				tripletListA.emplace_back(ir + row, ic_1 + 2, -jacobian(row, 2));
				tripletListA.emplace_back(ir + row, ic_1 + 3, -jacobian(row, 3));
				tripletListA.emplace_back(ir + row, ic_1 + 4, -jacobian(row, 4));
				tripletListA.emplace_back(ir + row, ic_1 + 5, -jacobian(row, 5));

				tripletListA.emplace_back(ir + row, ic_2, -jacobian(row, 6));
				tripletListA.emplace_back(ir + row, ic_2 + 1, -jacobian(row, 7));
				tripletListA.emplace_back(ir + row, ic_2 + 2, -jacobian(row, 8));
				tripletListA.emplace_back(ir + row, ic_2 + 3, -jacobian(row, 9));
				tripletListA.emplace_back(ir + row, ic_2 + 4, -jacobian(row, 10));
				tripletListA.emplace_back(ir + row, ic_2 + 5, -jacobian(row, 11));
			}

			tripletListB.emplace_back(ir, 0, delta(0, 0));
			tripletListB.emplace_back(ir + 1, 0, delta(1, 0));
			tripletListB.emplace_back(ir + 2, 0, delta(2, 0));
			tripletListB.emplace_back(ir + 3, 0, delta(3, 0));
			tripletListB.emplace_back(ir + 4, 0, delta(4, 0));
			tripletListB.emplace_back(ir + 5, 0, delta(5, 0));

			tripletListP.emplace_back(ir, ir, 1000000);
			tripletListP.emplace_back(ir + 1, ir + 1, 1000000);
			tripletListP.emplace_back(ir + 2, ir + 2, 1000000);
			tripletListP.emplace_back(ir + 3, ir + 3, 100000000);
			tripletListP.emplace_back(ir + 4, ir + 4, 100000000);
			tripletListP.emplace_back(ir + 5, ir + 5, 1000000);
		}*/
		for (size_t i = 0; i < odo_edges.size(); i++)
		{
			Eigen::Matrix<double, 6, 1> relative_pose_measurement_odo;
			relative_pose_tait_bryan_wc_case1_simplified_1(relative_pose_measurement_odo,
				poses_desired[odo_edges[i].first].px,
				poses_desired[odo_edges[i].first].py,
				poses_desired[odo_edges[i].first].pz,
				poses_desired[odo_edges[i].first].om,
				poses_desired[odo_edges[i].first].fi,
				poses_desired[odo_edges[i].first].ka,
				poses_desired[odo_edges[i].second].px,
				poses_desired[odo_edges[i].second].py,
				poses_desired[odo_edges[i].second].pz,
				poses_desired[odo_edges[i].second].om,
				poses_desired[odo_edges[i].second].fi,
				poses_desired[odo_edges[i].second].ka);

			Eigen::Matrix<double, 12, 12> AtPAodo;
			relative_pose_obs_eq_tait_bryan_wc_case1_AtPA_simplified(AtPAodo,
				poses[odo_edges[i].first].px,
				poses[odo_edges[i].first].py,
				poses[odo_edges[i].first].pz,
				poses[odo_edges[i].first].om,
				poses[odo_edges[i].first].fi,
				poses[odo_edges[i].first].ka,
				poses[odo_edges[i].second].px,
				poses[odo_edges[i].second].py,
				poses[odo_edges[i].second].pz,
				poses[odo_edges[i].second].om,
				poses[odo_edges[i].second].fi,
				poses[odo_edges[i].second].ka,
				1000000,
				1000000,
				1000000,
				100000000,
				100000000,
				1000000);
			Eigen::Matrix<double, 12, 1> AtPBodo;
			relative_pose_obs_eq_tait_bryan_wc_case1_AtPB_simplified(AtPBodo,
				poses[odo_edges[i].first].px,
				poses[odo_edges[i].first].py,
				poses[odo_edges[i].first].pz,
				poses[odo_edges[i].first].om,
				poses[odo_edges[i].first].fi,
				poses[odo_edges[i].first].ka,
				poses[odo_edges[i].second].px,
				poses[odo_edges[i].second].py,
				poses[odo_edges[i].second].pz,
				poses[odo_edges[i].second].om,
				poses[odo_edges[i].second].fi,
				poses[odo_edges[i].second].ka,
				relative_pose_measurement_odo(0, 0),
				relative_pose_measurement_odo(1, 0),
				relative_pose_measurement_odo(2, 0),
				relative_pose_measurement_odo(3, 0),
				relative_pose_measurement_odo(4, 0),
				relative_pose_measurement_odo(5, 0),
				1000000,
				1000000,
				1000000,
				100000000,
				100000000,
				1000000);
			int ic_1 = odo_edges[i].first * 6;
			int ic_2 = odo_edges[i].second * 6;

			for (int row = 0; row < 6; row++)
			{
				for (int col = 0; col < 6; col++)
				{
					AtPAndt(ic_1 + row, ic_1 + col) += AtPAodo(row, col);
					AtPAndt(ic_1 + row, ic_2 + col) += AtPAodo(row, col + 6);
					AtPAndt(ic_2 + row, ic_1 + col) += AtPAodo(row + 6, col);
					AtPAndt(ic_2 + row, ic_2 + col) += AtPAodo(row + 6, col + 6);
				}
			}

			for (int row = 0; row < 6; row++)
			{
				AtPBndt(ic_1 + row, 0) -= AtPBodo(row, 0);
				AtPBndt(ic_2 + row, 0) -= AtPBodo(row + 6, 0);
			}
		}

		// smoothness
		/*for (size_t i = 1; i < poses.size() - 1; i++)
		{
			Eigen::Matrix<double, 6, 1> delta;
			smoothness_obs_eq_tait_bryan_wc(delta,
											poses[i - 1].px,
											poses[i - 1].py,
											poses[i - 1].pz,
											poses[i - 1].om,
											poses[i - 1].fi,
											poses[i - 1].ka,
											poses[i].px,
											poses[i].py,
											poses[i].pz,
											poses[i].om,
											poses[i].fi,
											poses[i].ka,
											poses[i + 1].px,
											poses[i + 1].py,
											poses[i + 1].pz,
											poses[i + 1].om,
											poses[i + 1].fi,
											poses[i + 1].ka);

			Eigen::Matrix<double, 6, 18, Eigen::RowMajor> jacobian;
			smoothness_obs_eq_tait_bryan_wc_jacobian(jacobian,
													 poses[i - 1].px,
													 poses[i - 1].py,
													 poses[i - 1].pz,
													 poses[i - 1].om,
													 poses[i - 1].fi,
													 poses[i - 1].ka,
													 poses[i].px,
													 poses[i].py,
													 poses[i].pz,
													 poses[i].om,
													 poses[i].fi,
													 poses[i].ka,
													 poses[i + 1].px,
													 poses[i + 1].py,
													 poses[i + 1].pz,
													 poses[i + 1].om,
													 poses[i + 1].fi,
													 poses[i + 1].ka);

			int ir = tripletListB.size();

			int ic_1 = (i - 1) * 6;
			int ic_2 = i * 6;
			int ic_3 = (i + 1) * 6;

			for (size_t row = 0; row < 6; row++)
			{
				tripletListA.emplace_back(ir + row, ic_1, -jacobian(row, 0));
				tripletListA.emplace_back(ir + row, ic_1 + 1, -jacobian(row, 1));
				tripletListA.emplace_back(ir + row, ic_1 + 2, -jacobian(row, 2));
				tripletListA.emplace_back(ir + row, ic_1 + 3, -jacobian(row, 3));
				tripletListA.emplace_back(ir + row, ic_1 + 4, -jacobian(row, 4));
				tripletListA.emplace_back(ir + row, ic_1 + 5, -jacobian(row, 5));

				tripletListA.emplace_back(ir + row, ic_2, -jacobian(row, 6));
				tripletListA.emplace_back(ir + row, ic_2 + 1, -jacobian(row, 7));
				tripletListA.emplace_back(ir + row, ic_2 + 2, -jacobian(row, 8));
				tripletListA.emplace_back(ir + row, ic_2 + 3, -jacobian(row, 9));
				tripletListA.emplace_back(ir + row, ic_2 + 4, -jacobian(row, 10));
				tripletListA.emplace_back(ir + row, ic_2 + 5, -jacobian(row, 11));

				tripletListA.emplace_back(ir + row, ic_3, -jacobian(row, 12));
				tripletListA.emplace_back(ir + row, ic_3 + 1, -jacobian(row, 13));
				tripletListA.emplace_back(ir + row, ic_3 + 2, -jacobian(row, 14));
				tripletListA.emplace_back(ir + row, ic_3 + 3, -jacobian(row, 15));
				tripletListA.emplace_back(ir + row, ic_3 + 4, -jacobian(row, 16));
				tripletListA.emplace_back(ir + row, ic_3 + 5, -jacobian(row, 17));
			}
			tripletListB.emplace_back(ir, 0, delta(0, 0));
			tripletListB.emplace_back(ir + 1, 0, delta(1, 0));
			tripletListB.emplace_back(ir + 2, 0, delta(2, 0));
			tripletListB.emplace_back(ir + 3, 0, delta(3, 0));
			tripletListB.emplace_back(ir + 4, 0, delta(4, 0));
			tripletListB.emplace_back(ir + 5, 0, delta(5, 0));

			tripletListP.emplace_back(ir, ir, 10000);
			tripletListP.emplace_back(ir + 1, ir + 1, 10000);
			tripletListP.emplace_back(ir + 2, ir + 2, 10000);
			tripletListP.emplace_back(ir + 3, ir + 3, 10000);
			tripletListP.emplace_back(ir + 4, ir + 4, 10000);
			tripletListP.emplace_back(ir + 5, ir + 5, 10000);
		}*/

		// maintain angles
		if (add_pitch_roll_constraint)
		{
			for (int i = 0; i < imu_roll_pitch.size(); i++)
			{
				TaitBryanPose current_pose = poses[i];
				TaitBryanPose desired_pose = current_pose;
				desired_pose.om = imu_roll_pitch[i].first;
				desired_pose.fi = imu_roll_pitch[i].second;

				Eigen::Affine3d desired_mpose = affine_matrix_from_pose_tait_bryan(desired_pose);
				Eigen::Vector3d vx(desired_mpose(0, 0), desired_mpose(1, 0), desired_mpose(2, 0));
				Eigen::Vector3d vy(desired_mpose(0, 1), desired_mpose(1, 1), desired_mpose(2, 1));
				Eigen::Vector3d point_on_target_line(desired_mpose(0, 3), desired_mpose(1, 3), desired_mpose(2, 3));

				Eigen::Vector3d point_source_local(0, 0, 1);

				Eigen::Matrix<double, 2, 1> delta;
				point_to_line_tait_bryan_wc(delta,
					current_pose.px, current_pose.py, current_pose.pz, current_pose.om, current_pose.fi, current_pose.ka,
					point_source_local.x(), point_source_local.y(), point_source_local.z(),
					point_on_target_line.x(), point_on_target_line.y(), point_on_target_line.z(),
					vx.x(), vx.y(), vx.z(), vy.x(), vy.y(), vy.z());

				Eigen::Matrix<double, 2, 6> delta_jacobian;
				point_to_line_tait_bryan_wc_jacobian(delta_jacobian,
					current_pose.px, current_pose.py, current_pose.pz, current_pose.om, current_pose.fi, current_pose.ka,
					point_source_local.x(), point_source_local.y(), point_source_local.z(),
					point_on_target_line.x(), point_on_target_line.y(), point_on_target_line.z(),
					vx.x(), vx.y(), vx.z(), vy.x(), vy.y(), vy.z());

				int ir = tripletListB.size();

				for (int ii = 0; ii < 2; ii++)
				{
					for (int jj = 0; jj < 6; jj++)
					{
						int ic = i * 6;
						if (delta_jacobian(ii, jj) != 0.0)
						{
							tripletListA.emplace_back(ir + ii, ic + jj, -delta_jacobian(ii, jj));
						}
					}
				}
				// tripletListP.emplace_back(ir, ir, cauchy(delta(0, 0), 1));
				// tripletListP.emplace_back(ir + 1, ir + 1, cauchy(delta(1, 0), 1));
				tripletListP.emplace_back(ir, ir, 1);
				tripletListP.emplace_back(ir + 1, ir + 1, 1);

				tripletListB.emplace_back(ir, 0, delta(0, 0));
				tripletListB.emplace_back(ir + 1, 0, delta(1, 0));
			}
		}

		Eigen::SparseMatrix<double> matA(tripletListB.size(), intermediate_trajectory.size() * 6);
		Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
		Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

		matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
		matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
		matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

		Eigen::SparseMatrix<double> AtPA(intermediate_trajectory.size() * 6, intermediate_trajectory.size() * 6);
		Eigen::SparseMatrix<double> AtPB(intermediate_trajectory.size() * 6, 1);

		{
			Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
			AtPA = (AtP)*matA;
			AtPB = (AtP)*matB;
		}

		tripletListA.clear();
		tripletListP.clear();
		tripletListB.clear();

		AtPA += AtPAndt.sparseView();
		AtPB += AtPBndt.sparseView();
		Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);
		Eigen::SparseMatrix<double> x = solver.solve(AtPB);
		std::vector<double> h_x;
		for (int k = 0; k < x.outerSize(); ++k)
		{
			for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
			{
				h_x.push_back(it.value());
			}
		}

		if (h_x.size() == 6 * intermediate_trajectory.size())
		{
			int counter = 0;

			for (size_t i = 0; i < intermediate_trajectory.size(); i++)
			{
				TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(intermediate_trajectory[i]);
				pose.px += h_x[counter++];
				pose.py += h_x[counter++];
				pose.pz += h_x[counter++];
				pose.om += h_x[counter++];
				pose.fi += h_x[counter++];
				pose.ka += h_x[counter++];
				intermediate_trajectory[i] = affine_matrix_from_pose_tait_bryan(pose);
			}
		}
		return;
	}

	void update_rgd(NDT::GridParameters& rgd_params, NDTBucketMapType& buckets,
		std::vector<Point3Di>& points_global)
	{
		Eigen::Vector3d b(rgd_params.resolution_X, rgd_params.resolution_Y, rgd_params.resolution_Z);

		for (int i = 0; i < points_global.size(); i++)
		{
			auto index_of_bucket = get_rgd_index(points_global[i].point, b);

			auto bucket_it = buckets.find(index_of_bucket);

			if (bucket_it != buckets.end())
			{
				auto& this_bucket = bucket_it->second;
				this_bucket.number_of_points++;
				const auto& curr_mean = points_global[i].point;
				const auto& mean = this_bucket.mean;
				// buckets[index_of_bucket].mean += (mean - curr_mean) / buckets[index_of_bucket].number_of_points;

				auto mean_diff = mean - curr_mean;
				Eigen::Matrix3d cov_update;
				cov_update.row(0) = mean_diff.x() * mean_diff;
				cov_update.row(1) = mean_diff.y() * mean_diff;
				cov_update.row(2) = mean_diff.z() * mean_diff;

				this_bucket.cov = this_bucket.cov * (this_bucket.number_of_points - 1) / this_bucket.number_of_points +
					cov_update * (this_bucket.number_of_points - 1) / (this_bucket.number_of_points * this_bucket.number_of_points);
			}
			else
			{
				NDT::Bucket bucket_to_add;
				bucket_to_add.mean = points_global[i].point;
				bucket_to_add.cov = Eigen::Matrix3d::Identity() * 0.03 * 0.03;
				bucket_to_add.number_of_points = 1;
				buckets.emplace(index_of_bucket, bucket_to_add);
			}
		}
	}

	void align_to_reference(NDT::GridParameters& rgd_params, std::vector<Point3Di>& initial_points, Eigen::Affine3d& m_g, NDTBucketMapType& reference_buckets)
	{
		Eigen::SparseMatrix<double> AtPAndt(6, 6);
		Eigen::SparseMatrix<double> AtPBndt(6, 1);

		Eigen::Vector3d b(rgd_params.resolution_X, rgd_params.resolution_Y, rgd_params.resolution_Z);

		for (int i = 0; i < initial_points.size(); i += 1)
		{
			// if (initial_points[i].point.norm() < 1.0)
			//{
			//     continue;
			// }

			Eigen::Vector3d point_global = m_g * initial_points[i].point;
			auto index_of_bucket = get_rgd_index(point_global, b);

			if (!reference_buckets.contains(index_of_bucket))
			{
				continue;
			}

			// if(buckets[index_of_bucket].number_of_points >= 5){
			Eigen::Matrix3d infm = reference_buckets[index_of_bucket].cov.inverse();

			constexpr double threshold = 10000.0;

			if ((infm.array() > threshold).any())
			{
				continue;
			}
			if ((infm.array() < -threshold).any())
			{
				continue;
			}

			const Eigen::Affine3d& m_pose = m_g;
			const Eigen::Vector3d& p_s = initial_points[i].point;
			const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose);
			//
			Eigen::Matrix<double, 6, 6, Eigen::RowMajor> AtPA;
			point_to_point_source_to_target_tait_bryan_wc_AtPA_simplified(
				AtPA,
				pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
				p_s.x(), p_s.y(), p_s.z(),
				infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2));

			Eigen::Matrix<double, 6, 1> AtPB;
			point_to_point_source_to_target_tait_bryan_wc_AtPB_simplified(
				AtPB,
				pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
				p_s.x(), p_s.y(), p_s.z(),
				infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2),
				reference_buckets[index_of_bucket].mean.x(), reference_buckets[index_of_bucket].mean.y(), reference_buckets[index_of_bucket].mean.z());

			int c = 0;

			for (int row = 0; row < 6; row++)
			{
				for (int col = 0; col < 6; col++)
				{
					AtPAndt.coeffRef(c + row, c + col) += AtPA(row, col);
				}
			}

			for (int row = 0; row < 6; row++)
			{
				AtPBndt.coeffRef(c + row, 0) -= AtPB(row, 0);
			}
			//}
		}

		AtPAndt.coeffRef(0, 0) += 10000.0;
		AtPAndt.coeffRef(1, 1) += 10000.0;
		AtPAndt.coeffRef(2, 2) += 10000.0;
		AtPAndt.coeffRef(3, 3) += 10000.0;
		AtPAndt.coeffRef(4, 4) += 10000.0;
		AtPAndt.coeffRef(5, 5) += 10000.0;

		Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPAndt);
		Eigen::SparseMatrix<double> x = solver.solve(AtPBndt);
		std::vector<double> h_x;
		for (int k = 0; k < x.outerSize(); ++k)
		{
			for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
			{
				h_x.push_back(it.value());
			}
		}

		if (h_x.size() == 6)
		{
			int counter = 0;
			TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(m_g);
			pose.px += h_x[counter++];
			pose.py += h_x[counter++];
			pose.pz += h_x[counter++];
			pose.om += h_x[counter++];
			pose.fi += h_x[counter++];
			pose.ka += h_x[counter++];
			m_g = affine_matrix_from_pose_tait_bryan(pose);
		}
		else
		{
			std::cout << "align_to_reference FAILED" << std::endl;
		}
	}


}
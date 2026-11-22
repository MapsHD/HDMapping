#pragma once
// What an importer does to every IMU sample of a bag before HDMapping uses it:
// rotate it into the lidar frame (the frame HDMapping assumes the IMU is in) and
// bring acceleration to g. Shared by mcap_to_laz / bag_to_laz and the bag
// recording source of lidar_odometry_step_1.
#include "McapWriter.h" // McapImuSample

#include <vector>

namespace rosbags
{

// Standard gravity, the g HDMapping's IMU csv is scaled in.
inline constexpr double kStandardGravity = 9.80665;

struct ImuTransform
{
	double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0; // IMU frame -> lidar frame
	double acc_scale = 1.0; // 1 for a bag already in g, 1/9.80665 for m/s^2
};

inline void rotate(const ImuTransform& q, float& x, float& y, float& z)
{
	// v' = v + 2w (u x v) + 2 u x (u x v), u = (qx, qy, qz)
	const double vx = x, vy = y, vz = z;
	const double cx = q.qy * vz - q.qz * vy, cy = q.qz * vx - q.qx * vz, cz = q.qx * vy - q.qy * vx;
	x = static_cast<float>(vx + 2.0 * (q.qw * cx + q.qy * cz - q.qz * cy));
	y = static_cast<float>(vy + 2.0 * (q.qw * cy + q.qz * cx - q.qx * cz));
	z = static_cast<float>(vz + 2.0 * (q.qw * cz + q.qx * cy - q.qy * cx));
}

inline void applyImuTransform(const ImuTransform& t, McapImuSample& s)
{
	rotate(t, s.gyro_x, s.gyro_y, s.gyro_z);
	rotate(t, s.acc_x, s.acc_y, s.acc_z);
	s.acc_x = static_cast<float>(s.acc_x * t.acc_scale);
	s.acc_y = static_cast<float>(s.acc_y * t.acc_scale);
	s.acc_z = static_cast<float>(s.acc_z * t.acc_scale);
}

inline void applyImuTransform(const ImuTransform& t, std::vector<McapImuSample>& samples)
{
	for(auto& s : samples)
		applyImuTransform(t, s);
}

} // namespace rosbags

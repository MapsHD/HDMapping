#pragma once

#include <Eigen/Eigen>
#include <Core/structures.h>
#include <vector>
#include <memory>

struct IntegrationParams
{
    bool accel_units_in_g = true;
    bool gyro_units_in_deg_per_sec = true;
    double max_acceleration_threshold = 50.0; // m/s^2
    double max_dt_threshold = 0.1;            // seconds
    Eigen::Vector3d initial_velocity = Eigen::Vector3d::Zero();
    double vqf_tauAcc = 0.5; // VQF accelerometer time constant [s]
};

namespace imu_utils
{
Eigen::Vector3d convert_accel_to_ms2(const Eigen::Vector3d& raw, bool units_in_g, double g = 9.81);
Eigen::Vector3d convert_gyro_to_rads(const Eigen::Vector3d& raw, bool units_in_deg);
double safe_dt(double t_prev, double t_curr, double max_dt);
bool has_nan(const Eigen::Vector3d& v);
bool is_accel_valid(const Eigen::Vector3d& accel_ms2, double threshold);
} // namespace imu_utils

class AccelerationModel
{
public:
    virtual ~AccelerationModel() = default;

    virtual Eigen::Vector3d compute(
        const RawIMUData& imu,
        const Eigen::Affine3d& pose,
        const IntegrationParams& params) const = 0;
};

class BodyFrameAcceleration : public AccelerationModel
{
public:
    Eigen::Vector3d compute(
        const RawIMUData& imu,
        const Eigen::Affine3d& pose,
        const IntegrationParams& params) const override;
};

class GravityCompensatedAcceleration : public AccelerationModel
{
public:
    double gravity_magnitude = 9.81;
    Eigen::Vector3d gravity_vector = Eigen::Vector3d(0.0, 0.0, -9.81);

    Eigen::Vector3d compute(
        const RawIMUData& imu,
        const Eigen::Affine3d& pose,
        const IntegrationParams& params) const override;
};

class IntegrationMethod
{
public:
    virtual ~IntegrationMethod() = default;

    virtual Eigen::Vector3d integrate(
        const std::vector<RawIMUData>& raw_imu_data,
        const std::vector<Eigen::Affine3d>& new_trajectory,
        const AccelerationModel& accel_model,
        const IntegrationParams& params) const = 0;
};

class EulerIntegration : public IntegrationMethod
{
public:
    Eigen::Vector3d integrate(
        const std::vector<RawIMUData>& raw_imu_data,
        const std::vector<Eigen::Affine3d>& new_trajectory,
        const AccelerationModel& accel_model,
        const IntegrationParams& params) const override;
};

class TrapezoidalIntegration : public IntegrationMethod
{
public:
    Eigen::Vector3d integrate(
        const std::vector<RawIMUData>& raw_imu_data,
        const std::vector<Eigen::Affine3d>& new_trajectory,
        const AccelerationModel& accel_model,
        const IntegrationParams& params) const override;
};

class KalmanFilterIntegration : public IntegrationMethod
{
public:
    double process_noise_accel = 0.5;
    double process_noise_bias = 0.01;
    double measurement_noise_velocity = 1.0;
    Eigen::Vector3d initial_accel_bias = Eigen::Vector3d::Zero();

    Eigen::Vector3d integrate(
        const std::vector<RawIMUData>& raw_imu_data,
        const std::vector<Eigen::Affine3d>& new_trajectory,
        const AccelerationModel& accel_model,
        const IntegrationParams& params) const override;
};

enum class PreintegrationMethod
{
    euler_body_frame = 0,
    trapezoidal_body_frame = 1,
    euler_gravity_compensated = 2,
    trapezoidal_gravity_compensated = 3,
    kalman_filter = 4,
    euler_gyro_gravity_compensated = 5,
    trapezoidal_gyro_gravity_compensated = 6,
    kalman_gyro_gravity_compensated = 7,
};

inline const char* to_string(PreintegrationMethod method)
{
    switch (method)
    {
    case PreintegrationMethod::euler_body_frame: return "Euler, no gravity comp., SM velocity";
    case PreintegrationMethod::trapezoidal_body_frame: return "Trapezoidal, no gravity comp., SM velocity";
    case PreintegrationMethod::euler_gravity_compensated: return "Euler, gravity comp., SM velocity";
    case PreintegrationMethod::trapezoidal_gravity_compensated: return "Trapezoidal, gravity comp., SM velocity";
    case PreintegrationMethod::kalman_filter: return "Kalman, gravity comp., SM velocity";
    case PreintegrationMethod::euler_gyro_gravity_compensated: return "Euler, gravity comp., VQF velocity";
    case PreintegrationMethod::trapezoidal_gyro_gravity_compensated: return "Trapezoidal, gravity comp., VQF velocity";
    case PreintegrationMethod::kalman_gyro_gravity_compensated: return "Kalman, gravity comp., VQF velocity";
    default: return "unknown";
    }
}

class ImuPreintegration
{
public:
    IntegrationParams params;

    Eigen::Vector3d preintegrate(
        const std::vector<RawIMUData>& raw_imu_data,
        const std::vector<Eigen::Affine3d>& new_trajectory,
        const AccelerationModel& accel_model,
        const IntegrationMethod& integration_method);

    static Eigen::Vector3d create_and_preintegrate(
        PreintegrationMethod method,
        const std::vector<RawIMUData>& raw_imu_data,
        const std::vector<Eigen::Affine3d>& new_trajectory,
        const IntegrationParams& params = IntegrationParams());
};

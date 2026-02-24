#include <imu_preintegration.h>

#include <cmath>
#include <algorithm>
#include <iostream>

namespace imu_utils
{

Eigen::Vector3d convert_accel_to_ms2(const Eigen::Vector3d& raw, bool units_in_g, double g)
{
    if (units_in_g)
        return raw * g;
    return raw;
}

Eigen::Vector3d convert_gyro_to_rads(const Eigen::Vector3d& raw, bool units_in_deg)
{
    if (units_in_deg)
        return raw * (M_PI / 180.0);
    return raw;
}

double safe_dt(double t_prev, double t_curr, double max_dt)
{
    double dt = t_curr - t_prev;
    if (dt <= 0.0 || std::isnan(dt))
        return 0.0;
    return std::min(dt, max_dt);
}

bool has_nan(const Eigen::Vector3d& v)
{
    return std::isnan(v.x()) || std::isnan(v.y()) || std::isnan(v.z());
}

bool is_accel_valid(const Eigen::Vector3d& accel_ms2, double threshold)
{
    return accel_ms2.norm() < threshold && !has_nan(accel_ms2);
}

std::vector<Eigen::Matrix3d> estimate_orientations(
    const std::vector<RawIMUData>& raw_imu_data,
    const Eigen::Matrix3d& initial_orientation,
    const IntegrationParams& params)
{
    std::vector<Eigen::Matrix3d> orientations;
    orientations.reserve(raw_imu_data.size());

    Eigen::Matrix3d R = initial_orientation;
    orientations.push_back(R);

    for (size_t k = 1; k < raw_imu_data.size(); k++)
    {
        double dt = safe_dt(raw_imu_data[k - 1].timestamp, raw_imu_data[k].timestamp, params.max_dt_threshold);
        if (dt == 0.0)
        {
            orientations.push_back(R);
            continue;
        }

        Eigen::Vector3d omega = convert_gyro_to_rads(raw_imu_data[k].guroscopes, params.gyro_units_in_deg_per_sec);
        double angle = omega.norm() * dt;
        if (angle > 1e-10)
        {
            Eigen::Vector3d axis = omega.normalized();
            R = R * Eigen::AngleAxisd(angle, axis).toRotationMatrix();
        }

        if (params.ahrs_gain > 0.0)
        {
            Eigen::Vector3d a_body = convert_accel_to_ms2(raw_imu_data[k].accelerometers, params.accel_units_in_g);
            if (a_body.norm() > 1e-6)
            {
                Eigen::Vector3d a_norm = a_body.normalized();
                Eigen::Vector3d g_expected = R.transpose() * Eigen::Vector3d(0, 0, -1);
                Eigen::Vector3d correction_axis = g_expected.cross(a_norm);
                double correction_angle = std::asin(std::min(1.0, correction_axis.norm()));
                if (correction_angle > 1e-10)
                {
                    correction_axis.normalize();
                    R = R * Eigen::AngleAxisd(params.ahrs_gain * correction_angle, correction_axis).toRotationMatrix();
                }
            }
        }

        orientations.push_back(R);
    }
    return orientations;
}

} // namespace imu_utils

Eigen::Vector3d BodyFrameAcceleration::compute(
    const RawIMUData& imu,
    const Eigen::Affine3d& /*pose*/,
    const IntegrationParams& params) const
{
    return imu_utils::convert_accel_to_ms2(imu.accelerometers, params.accel_units_in_g);
}

Eigen::Vector3d GravityCompensatedAcceleration::compute(
    const RawIMUData& imu,
    const Eigen::Affine3d& pose,
    const IntegrationParams& params) const
{
    Eigen::Vector3d a_body = imu_utils::convert_accel_to_ms2(imu.accelerometers, params.accel_units_in_g, gravity_magnitude);
    Eigen::Matrix3d R = pose.rotation();
    return R * a_body + gravity_vector;
}

// v_{k} = v_{k-1} + a_k * dt
// p_{k} = p_{k-1} + v_{k} * dt
Eigen::Vector3d EulerIntegration::integrate(
    const std::vector<RawIMUData>& raw_imu_data,
    const std::vector<Eigen::Affine3d>& new_trajectory,
    const AccelerationModel& accel_model,
    const IntegrationParams& params) const
{
    const size_t n = raw_imu_data.size();
    Eigen::Vector3d velocity = params.initial_velocity;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();

    for (size_t k = 1; k < n; k++)
    {
        double dt = imu_utils::safe_dt(raw_imu_data[k - 1].timestamp, raw_imu_data[k].timestamp, params.max_dt_threshold);
        if (dt == 0.0)
            continue;

        Eigen::Vector3d accel = accel_model.compute(raw_imu_data[k], new_trajectory[k], params);
        if (!imu_utils::is_accel_valid(accel, params.max_acceleration_threshold))
            accel = Eigen::Vector3d::Zero();

        velocity += accel * dt;
        position += velocity * dt;
    }

    return position;
}

// v_{k} = v_{k-1} + 0.5 * (a_{k-1} + a_k) * dt
// p_{k} = p_{k-1} + 0.5 * (v_{k-1} + v_k) * dt
Eigen::Vector3d TrapezoidalIntegration::integrate(
    const std::vector<RawIMUData>& raw_imu_data,
    const std::vector<Eigen::Affine3d>& new_trajectory,
    const AccelerationModel& accel_model,
    const IntegrationParams& params) const
{
    const size_t n = raw_imu_data.size();
    Eigen::Vector3d velocity = params.initial_velocity;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();

    Eigen::Vector3d prev_accel = accel_model.compute(raw_imu_data[0], new_trajectory[0], params);
    if (!imu_utils::is_accel_valid(prev_accel, params.max_acceleration_threshold))
        prev_accel = Eigen::Vector3d::Zero();

    for (size_t k = 1; k < n; k++)
    {
        double dt = imu_utils::safe_dt(raw_imu_data[k - 1].timestamp, raw_imu_data[k].timestamp, params.max_dt_threshold);
        if (dt == 0.0)
            continue;

        Eigen::Vector3d curr_accel = accel_model.compute(raw_imu_data[k], new_trajectory[k], params);
        if (!imu_utils::is_accel_valid(curr_accel, params.max_acceleration_threshold))
            curr_accel = prev_accel;

        Eigen::Vector3d prev_velocity = velocity;
        velocity += 0.5 * (prev_accel + curr_accel) * dt;
        position += 0.5 * (prev_velocity + velocity) * dt;

        prev_accel = curr_accel;
    }

    return position;
}

// EKF state: x = [position(3), velocity(3), accel_bias(3)]^T (9-dim)
// F = [I, I*dt, 0; 0, I, -I*dt; 0, 0, I], H = [0, 0, -I]
Eigen::Vector3d KalmanFilterIntegration::integrate(
    const std::vector<RawIMUData>& raw_imu_data,
    const std::vector<Eigen::Affine3d>& new_trajectory,
    const AccelerationModel& accel_model,
    const IntegrationParams& params) const
{
    const size_t n = raw_imu_data.size();

    Eigen::Matrix<double, 9, 1> state = Eigen::Matrix<double, 9, 1>::Zero();
    state.segment<3>(0) = Eigen::Vector3d::Zero();
    state.segment<3>(3) = params.initial_velocity;
    state.segment<3>(6) = initial_accel_bias;

    Eigen::Matrix<double, 9, 9> P = Eigen::Matrix<double, 9, 9>::Identity();
    P.block<3, 3>(0, 0) *= 0.01;
    P.block<3, 3>(3, 3) *= 1.0;
    P.block<3, 3>(6, 6) *= 0.1;

    double sigma_a = process_noise_accel;
    double sigma_b = process_noise_bias;

    for (size_t k = 1; k < n; k++)
    {
        double dt = imu_utils::safe_dt(raw_imu_data[k - 1].timestamp, raw_imu_data[k].timestamp, params.max_dt_threshold);
        if (dt == 0.0)
            continue;

        Eigen::Vector3d accel = accel_model.compute(raw_imu_data[k], new_trajectory[k], params);
        if (!imu_utils::is_accel_valid(accel, params.max_acceleration_threshold))
            accel = Eigen::Vector3d::Zero();

        Eigen::Vector3d bias = state.segment<3>(6);
        Eigen::Vector3d a_corrected = accel - bias;

        Eigen::Matrix<double, 9, 9> F = Eigen::Matrix<double, 9, 9>::Identity();
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
        F.block<3, 3>(3, 6) = -Eigen::Matrix3d::Identity() * dt;

        Eigen::Matrix<double, 9, 1> state_pred;
        state_pred.segment<3>(0) = state.segment<3>(0) + state.segment<3>(3) * dt + 0.5 * a_corrected * dt * dt;
        state_pred.segment<3>(3) = state.segment<3>(3) + a_corrected * dt;
        state_pred.segment<3>(6) = state.segment<3>(6);

        Eigen::Matrix<double, 9, 9> Q = Eigen::Matrix<double, 9, 9>::Zero();
        Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (sigma_a * sigma_a * dt * dt * dt * dt / 4.0);
        Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (sigma_a * sigma_a * dt * dt);
        Q.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * (sigma_b * sigma_b * dt);

        Eigen::Matrix<double, 9, 9> P_pred = F * P * F.transpose() + Q;

        Eigen::Matrix<double, 3, 9> H = Eigen::Matrix<double, 3, 9>::Zero();
        H.block<3, 3>(0, 6) = -Eigen::Matrix3d::Identity();

        Eigen::Matrix3d R_meas = Eigen::Matrix3d::Identity() * (measurement_noise_accel * measurement_noise_accel);
        Eigen::Matrix3d S = H * P_pred * H.transpose() + R_meas;
        Eigen::Matrix<double, 9, 3> K = P_pred * H.transpose() * S.inverse();

        Eigen::Vector3d innovation = state_pred.segment<3>(6);

        state = state_pred + K * innovation;
        P = (Eigen::Matrix<double, 9, 9>::Identity() - K * H) * P_pred;
    }

    return state.segment<3>(0);
}

Eigen::Vector3d ImuPreintegration::preintegrate(
    const std::vector<RawIMUData>& raw_imu_data,
    const std::vector<Eigen::Affine3d>& new_trajectory,
    const AccelerationModel& accel_model,
    const IntegrationMethod& integration_method)
{
    if (raw_imu_data.size() < 2 || new_trajectory.size() < 2)
        return Eigen::Vector3d::Zero();

    if (raw_imu_data.size() != new_trajectory.size())
        return Eigen::Vector3d::Zero();

    Eigen::Vector3d total_displacement = integration_method.integrate(
        raw_imu_data, new_trajectory, accel_model, params);

    if (imu_utils::has_nan(total_displacement))
        return Eigen::Vector3d::Zero();

    Eigen::Vector3d mean_shift = total_displacement / static_cast<double>(raw_imu_data.size() - 1);
    return mean_shift;
}

Eigen::Vector3d ImuPreintegration::create_and_preintegrate(
    PreintegrationMethod method,
    const std::vector<RawIMUData>& raw_imu_data,
    const std::vector<Eigen::Affine3d>& new_trajectory,
    const IntegrationParams& params)
{
    ImuPreintegration preint;
    preint.params = params;

    std::unique_ptr<AccelerationModel> accel_model;
    std::unique_ptr<IntegrationMethod> integration_method;

    switch (method)
    {
    case PreintegrationMethod::euler_body_frame:
        accel_model = std::make_unique<BodyFrameAcceleration>();
        integration_method = std::make_unique<EulerIntegration>();
        break;
    case PreintegrationMethod::trapezoidal_body_frame:
        accel_model = std::make_unique<BodyFrameAcceleration>();
        integration_method = std::make_unique<TrapezoidalIntegration>();
        break;
    case PreintegrationMethod::euler_gravity_compensated:
        accel_model = std::make_unique<GravityCompensatedAcceleration>();
        integration_method = std::make_unique<EulerIntegration>();
        break;
    case PreintegrationMethod::trapezoidal_gravity_compensated:
        accel_model = std::make_unique<GravityCompensatedAcceleration>();
        integration_method = std::make_unique<TrapezoidalIntegration>();
        break;
    case PreintegrationMethod::kalman_filter:
        accel_model = std::make_unique<GravityCompensatedAcceleration>();
        integration_method = std::make_unique<KalmanFilterIntegration>();
        break;
    case PreintegrationMethod::euler_gyro_gravity_compensated:
    case PreintegrationMethod::trapezoidal_gyro_gravity_compensated:
    case PreintegrationMethod::kalman_gyro_gravity_compensated:
    {
        auto orientations = imu_utils::estimate_orientations(
            raw_imu_data, new_trajectory[0].rotation(), params);

        std::vector<Eigen::Affine3d> imu_trajectory = new_trajectory;
        for (size_t k = 0; k < imu_trajectory.size() && k < orientations.size(); k++)
            imu_trajectory[k].linear() = orientations[k];

        accel_model = std::make_unique<GravityCompensatedAcceleration>();
        if (method == PreintegrationMethod::euler_gyro_gravity_compensated)
            integration_method = std::make_unique<EulerIntegration>();
        else if (method == PreintegrationMethod::trapezoidal_gyro_gravity_compensated)
            integration_method = std::make_unique<TrapezoidalIntegration>();
        else
            integration_method = std::make_unique<KalmanFilterIntegration>();

        return preint.preintegrate(raw_imu_data, imu_trajectory, *accel_model, *integration_method);
    }
    default:
        std::cerr << "ImuPreintegration: unknown method " << static_cast<int>(method) << std::endl;
        return Eigen::Vector3d::Zero();
    }

    return preint.preintegrate(raw_imu_data, new_trajectory, *accel_model, *integration_method);
}

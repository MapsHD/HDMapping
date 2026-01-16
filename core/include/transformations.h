#pragma once

#include "structures.h"
// #include <opencv2/opencv.hpp>

#ifndef MAX
#define MAX(a, b) a > b ? a : b
#endif
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <random>

inline double deg2rad(double deg)
{
    return (deg * M_PI) / 180.0;
}

inline double rad2deg(double rad)
{
    return (rad * 180.0) / M_PI;
}

inline TaitBryanPose pose_tait_bryan_from_affine_matrix(Eigen::Affine3d m)
{
    TaitBryanPose pose;

    pose.px = m(0, 3);
    pose.py = m(1, 3);
    pose.pz = m(2, 3);

    if (m(0, 2) < 1)
    {
        if (m(0, 2) > -1)
        {
            // case 1
            pose.fi = asin(m(0, 2));
            pose.om = atan2(-m(1, 2), m(2, 2));
            pose.ka = atan2(-m(0, 1), m(0, 0));

            return pose;
        }
        else // r02 = −1
        {
            // case 2
            // not a unique solution: thetaz − thetax = atan2 ( r10 , r11 )
            pose.fi = -M_PI / 2.0;
            pose.om = -atan2(m(1, 0), m(1, 1));
            pose.ka = 0;
            return pose;
        }
    }
    else
    {
        // case 3
        // r02 = +1
        // not a unique solution: thetaz + thetax = atan2 ( r10 , r11 )
        pose.fi = M_PI / 2.0;
        pose.om = atan2(m(1, 0), m(1, 1));
        pose.ka = 0.0;
        return pose;
    }

    return pose;
}

inline Eigen::Affine3d affine_matrix_from_pose_tait_bryan(TaitBryanPose pose)
{
    Eigen::Affine3d m = Eigen::Affine3d::Identity();

    double sx = sin(pose.om);
    double cx = cos(pose.om);
    double sy = sin(pose.fi);
    double cy = cos(pose.fi);
    double sz = sin(pose.ka);
    double cz = cos(pose.ka);

    m(0, 0) = cy * cz;
    m(1, 0) = cz * sx * sy + cx * sz;
    m(2, 0) = -cx * cz * sy + sx * sz;

    m(0, 1) = -cy * sz;
    m(1, 1) = cx * cz - sx * sy * sz;
    m(2, 1) = cz * sx + cx * sy * sz;

    m(0, 2) = sy;
    m(1, 2) = -cy * sx;
    m(2, 2) = cx * cy;

    m(0, 3) = pose.px;
    m(1, 3) = pose.py;
    m(2, 3) = pose.pz;

    return m;
}

inline RodriguesPose pose_rodrigues_from_affine_matrix(Eigen::Affine3d m)
{
    // https://github.com/shimat/opencv_files_343/blob/master/include/x64/opencv2/core/affine.hpp
    // https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga61585db663d9da06b68e70cfbf6a1eac
    // https://docs.rs/opencv/0.22.1/opencv/calib3d/fn.rodrigues.html
    // https://github.com/opencv/opencv/blob/master/modules/calib3d/src/calibration.cpp
    RodriguesPose rp;

    double r_x = m(2, 1) - m(1, 2);
    double r_y = m(0, 2) - m(2, 0);
    double r_z = m(1, 0) - m(0, 1);

    // Point3d r(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
    double s = std::sqrt((r_x * r_x + r_y * r_y + r_z * r_z) * 0.25);
    double c = (m(0, 0) + m(1, 1) + m(2, 2) - 1) * 0.5;
    c = c > 1. ? 1. : c < -1. ? -1. : c;

    double theta = acos(c);

    if (s < 1e-8)
    {
        double t;

        if (c > 0)
            r_x = r_y = r_z = 0.0;
        else
        {
            t = (m(0, 0) + 1) * 0.5;
            r_x = std::sqrt(MAX(t, 0.));
            t = (m(1, 1) + 1) * 0.5;
            r_y = std::sqrt(MAX(t, 0.)) * (m(0, 1) < 0 ? -1. : 1.);
            t = (m(2, 2) + 1) * 0.5;
            r_z = std::sqrt(MAX(t, 0.)) * (m(0, 2) < 0 ? -1. : 1.);
            if (fabs(r_x) < fabs(r_y) && fabs(r_x) < fabs(r_z) && (m(1, 2) > 0) != (r_y * r_z > 0))
                r_z = -r_z;
            theta /= sqrt(r_x * r_x + r_y * r_y + r_z * r_z); // norm(r);
            r_x *= theta;
            r_y *= theta;
            r_z *= theta;
        }

        rp.sx = r_x;
        rp.sy = r_y;
        rp.sz = r_z;

        rp.px = m(0, 3);
        rp.py = m(1, 3);
        rp.pz = m(2, 3);
    }
    else
    {
        double vth = 1 / (2 * s);
        vth *= theta;
        r_x *= vth;
        r_y *= vth;
        r_z *= vth;

        rp.sx = r_x;
        rp.sy = r_y;
        rp.sz = r_z;

        rp.px = m(0, 3);
        rp.py = m(1, 3);
        rp.pz = m(2, 3);
    }
    return rp;
}

inline void orthogonalize_rotation(Eigen::Affine3d& m)
{
    Eigen::Matrix4d ret = m.matrix();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(ret.block<3, 3>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
    double d = (svd.matrixU() * svd.matrixV().transpose()).determinant();
    Eigen::Matrix3d diag = Eigen::Matrix3d::Identity() * d;
    ret.block<3, 3>(0, 0) = svd.matrixU() * diag * svd.matrixV().transpose();
    m = Eigen::Affine3d(ret);

#if 0
	Eigen::Affine3d mtemp = m;

	cv::Matx33d U, Vt;
	cv::Vec3d W;
	cv::Matx33d R (m(0,0), m(0,1), m(0,2),
		 m(1,0), m(1,1), m(1,2),
		 m(2,0), m(2,1), m(2,2));
	cv::SVD::compute(R, W, U, Vt);

	R = U*Vt;

	m(0,0) = R(0,0);
	m(0,1) = R(0,1);
	m(0,2) = R(0,2);

	m(1,0) = R(1,0);
	m(1,1) = R(1,1);
	m(1,2) = R(1,2);

	m(2,0) = R(2,0);
	m(2,1) = R(2,1);
	m(2,2) = R(2,2);
#endif
}

inline Eigen::Affine3d affine_matrix_from_pose_rodrigues(const RodriguesPose& pr)
{
// https://github.com/shimat/opencv_files_343/blob/master/include/x64/opencv2/core/affine.hpp
// https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga61585db663d9da06b68e70cfbf6a1eac
// https://docs.rs/opencv/0.22.1/opencv/calib3d/fn.rodrigues.html
// https://github.com/opencv/opencv/blob/master/modules/calib3d/src/calibration.cpp
#if 0
	double c = cos(theta);
	double s = sin(theta);
	double c1 = 1. - c;
	double itheta = theta ? 1./theta : 0.;

	r *= itheta;

	Matx33d rrt( r.x*r.x, r.x*r.y, r.x*r.z, r.x*r.y, r.y*r.y, r.y*r.z, r.x*r.z, r.y*r.z, r.z*r.z );
	Matx33d r_x(    0, -r.z,  r.y,
				  r.z,    0, -r.x,
				 -r.y,  r.x,    0 );

	// R = cos(theta)*I + (1 - cos(theta))*r*rT + sin(theta)*[r_x]
	Matx33d R = c*Matx33d::eye() + c1*rrt + s*r_x;
#endif

    Eigen::Affine3d m = Eigen::Affine3d::Identity();

    double norm = sqrt((pr.sx) * (pr.sx) + (pr.sy) * (pr.sy) + (pr.sz) * (pr.sz));
    double theta = norm;
    double c = cos(theta);
    double s = sin(theta);
    double c1 = 1. - c;

    double x = pr.sx;
    double y = pr.sy;
    double z = pr.sz;

    double itheta = theta ? 1. / theta : 0.;

    x *= itheta;
    y *= itheta;
    z *= itheta;

    double rrt[9] = { x * x, x * y, x * z, x * y, y * y, y * z, x * z, y * z, z * z };

    double r_x[9] = { 0, -z, y, z, 0, -x, -y, x, 0 };

    double c_eye[9] = { c, 0, 0, 0, c, 0, 0, 0, c };

    double c1_rrt[9] = {
        c1 * rrt[0], c1 * rrt[1], c1 * rrt[2], c1 * rrt[3], c1 * rrt[4], c1 * rrt[5], c1 * rrt[6], c1 * rrt[7], c1 * rrt[8]
    };

    double s_r_x[9] = { s * r_x[0], s * r_x[1], s * r_x[2], s * r_x[3], s * r_x[4], s * r_x[5], s * r_x[6], s * r_x[7], s * r_x[8] };

    // R = cos(theta)*I + (1 - cos(theta))*r*rT + sin(theta)*[r_x]
    // Matx33d R = c*Matx33d::eye() + c1*rrt + s*r_x;

    double R[9] = { c_eye[0] + c1_rrt[0] + s_r_x[0], c_eye[1] + c1_rrt[1] + s_r_x[1], c_eye[2] + c1_rrt[2] + s_r_x[2],
                    c_eye[3] + c1_rrt[3] + s_r_x[3], c_eye[4] + c1_rrt[4] + s_r_x[4], c_eye[5] + c1_rrt[5] + s_r_x[5],
                    c_eye[6] + c1_rrt[6] + s_r_x[6], c_eye[7] + c1_rrt[7] + s_r_x[7], c_eye[8] + c1_rrt[8] + s_r_x[8] };

    m(0, 0) = R[0];
    m(0, 1) = R[1];
    m(0, 2) = R[2];

    m(1, 0) = R[3];
    m(1, 1) = R[4];
    m(1, 2) = R[5];

    m(2, 0) = R[6];
    m(2, 1) = R[7];
    m(2, 2) = R[8];

    m(0, 3) = pr.px;
    m(1, 3) = pr.py;
    m(2, 3) = pr.pz;

    // orthogonalize_rotation(m);

    return m;
}

inline void normalize_quaternion(QuaternionPose& pq)
{
    double norm = sqrt((pq.q0 * pq.q0 + pq.q1 * pq.q1 + pq.q2 * pq.q2 + pq.q3 * pq.q3));

    pq.q0 /= norm;
    pq.q1 /= norm;
    pq.q2 /= norm;
    pq.q3 /= norm;
}

inline Eigen::Affine3d affine_matrix_from_pose_quaternion(const QuaternionPose& pq)
{
    Eigen::Affine3d m = Eigen::Affine3d::Identity();
    double q0 = pq.q0;
    double q1 = pq.q1;
    double q2 = pq.q2;
    double q3 = pq.q3;
    double t0 = pq.px;
    double t1 = pq.py;
    double t2 = pq.pz;

    double q11 = q1 * q1;
    double q22 = q2 * q2;
    double q33 = q3 * q3;
    double q03 = q0 * q3;
    double q13 = q1 * q3;
    double q23 = q2 * q3;
    double q02 = q0 * q2;
    double q12 = q1 * q2;
    double q01 = q0 * q1;

    m(0, 0) = 1 - 2 * (q22 + q33);
    m(1, 0) = 2.0 * (q12 + q03);
    m(2, 0) = 2.0 * (q13 - q02);

    m(0, 1) = 2.0 * (q12 - q03);
    m(1, 1) = 1 - 2 * (q11 + q33);
    m(2, 1) = 2.0 * (q23 + q01);

    m(0, 2) = 2.0 * (q13 + q02);
    m(1, 2) = 2.0 * (q23 - q01);
    m(2, 2) = 1 - 2 * (q11 + q22);

    m(0, 3) = t0;
    m(1, 3) = t1;
    m(2, 3) = t2;

    orthogonalize_rotation(m);

    return m;
}

inline QuaternionPose pose_quaternion_from_affine_matrix(const Eigen::Affine3d& m)
{
    QuaternionPose pq;

    double T, S, X, Y, Z, W;

    T = 1 + m(0, 0) + m(1, 1) + m(2, 2);
    if (T > 0.00000001)
    { // to avoid large distortions!
        S = sqrt(T) * 2;
        X = (m(1, 2) - m(2, 1)) / S;
        Y = (m(2, 0) - m(0, 2)) / S;
        Z = (m(0, 1) - m(1, 0)) / S;
        W = 0.25 * S;
    }
    else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2))
    { // Column 0:
        S = sqrt(1.0 + m(0, 0) - m(1, 1) - m(2, 2)) * 2;
        X = 0.25 * S;
        Y = (m(0, 1) + m(1, 0)) / S;
        Z = (m(2, 0) + m(0, 2)) / S;
        W = (m(1, 2) - m(2, 1)) / S;
    }
    else if (m(1, 1) > m(2, 2))
    { // Column 1:
        S = sqrt(1.0 + m(1, 1) - m(0, 0) - m(2, 2)) * 2;
        X = (m(0, 1) + m(1, 0)) / S;
        Y = 0.25 * S;
        Z = (m(1, 2) + m(2, 1)) / S;
        W = (m(2, 0) - m(0, 2)) / S;
    }
    else
    { // Column 2:
        S = sqrt(1.0 + m(2, 2) - m(0, 0) - m(1, 1)) * 2;
        X = (m(2, 0) + m(0, 2)) / S;
        Y = (m(1, 2) + m(2, 1)) / S;
        Z = 0.25 * S;
        W = (m(0, 1) - m(1, 0)) / S;
    }
    pq.q0 = W;
    pq.q1 = -X;
    pq.q2 = -Y;
    pq.q3 = -Z;

    normalize_quaternion(pq);

    pq.px = m(0, 3);
    pq.py = m(1, 3);
    pq.pz = m(2, 3);

    return pq;
}

inline double normalize_angle(double src_rad)
{
    double arg;

    arg = fmod(src_rad, 2.0 * M_PI);
    if (arg < 0)
        arg = arg + 2.0 * M_PI;
    if (arg > M_PI)
        arg = arg - 2.0 * M_PI;
    return arg;
}

inline Eigen::Affine3d pose_interpolation(double t, double t1, double t2, Eigen::Affine3d const& aff1, Eigen::Affine3d const& aff2)
{
    // assume here t1 <= t <= t2
    double alpha = 0.0;
    if (t2 != t1)
        alpha = (t - t1) / (t2 - t1);

    Eigen::Quaternion<double> rot1(aff1.linear());
    Eigen::Quaternion<double> rot2(aff2.linear());

    Eigen::Vector3d trans1 = aff1.translation();
    Eigen::Vector3d trans2 = aff2.translation();

    Eigen::Affine3d result;
    result.translation() = (1.0 - alpha) * trans1 + alpha * trans2;
    result.linear() = rot1.slerp(alpha, rot2).toRotationMatrix();

    return result;
}

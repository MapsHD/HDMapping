#pragma once

#include <Eigen/Eigen>

// this function provides unique index
unsigned long long int get_index(const int16_t x, const int16_t y, const int16_t z);

// this function provides unique index for input point p and 3D space decomposition into buckets b
unsigned long long int get_rgd_index(const Eigen::Vector3d p, const Eigen::Vector3d b);

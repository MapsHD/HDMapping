#include <pch/pch.h>

#include <icp.h>
#include <registration_plane_feature.h>

#include <m_estimators.h>
#include <transformations.h>

#include <python-scripts/feature-to-feature-metrics/plane_to_plane_source_to_target_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-feature-metrics/distance_point_to_plane_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-feature-metrics/point_to_plane_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-point-metrics/point_to_projection_onto_plane_tait_bryan_wc_jacobian.h>

std::vector<RegistrationPlaneFeature::Job> RegistrationPlaneFeature::get_jobs(long long unsigned int size, int num_threads)
{
    int hc = size / num_threads;
    if (hc < 1)
        hc = 1;

    std::vector<Job> jobs;
    for (long long unsigned int i = 0; i < size; i += hc)
    {
        long long unsigned int sequence_length = hc;
        if (i + hc >= size)
        {
            sequence_length = size - i;
        }
        if (sequence_length == 0)
            break;

        Job j;
        j.index_begin_inclusive = i;
        j.index_end_exclusive = i + sequence_length;
        jobs.push_back(j);
    }
    return jobs;
}

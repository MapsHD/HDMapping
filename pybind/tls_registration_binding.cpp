#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <multi_view_tls_registration.h>
#include <icp.h>
#include <gnss.h>
#include <registration_plane_feature.h>
#include <pose_graph_slam.h>

namespace py = pybind11;

PYBIND11_MODULE(multi_view_tls_registration_py, m) {
    py::class_<TLSRegistration>(m, "TLSRegistration")
        .def(py::init<>())
        .def_readwrite("use_ndt", &TLSRegistration::use_ndt)
        .def_readwrite("compute_only_mahalanobis_distance", &TLSRegistration::compute_only_mahalanobis_distance)
        .def_readwrite("compute_mean_and_cov_for_bucket", &TLSRegistration::compute_mean_and_cov_for_bucket)
        .def_readwrite("use_lie_algebra_left_jacobian_ndt", &TLSRegistration::use_lie_algebra_left_jacobian_ndt)
        .def_readwrite("use_lie_algebra_right_jacobian_ndt", &TLSRegistration::use_lie_algebra_right_jacobian_ndt)
        .def_readwrite("use_icp", &TLSRegistration::use_icp)
        .def_readwrite("point_to_point_source_to_target", &TLSRegistration::point_to_point_source_to_target)
        .def_readwrite("use_lie_algebra_left_jacobian_icp", &TLSRegistration::use_lie_algebra_left_jacobian_icp)
        .def_readwrite("use_lie_algebra_right_jacobian_icp", &TLSRegistration::use_lie_algebra_right_jacobian_icp)
        .def_readwrite("point_to_point_source_to_target_compute_rms", &TLSRegistration::point_to_point_source_to_target_compute_rms)
        .def_readwrite("use_plane_features", &TLSRegistration::use_plane_features)
        .def_readwrite("point_to_projection_onto_plane_source_to_target", &TLSRegistration::point_to_projection_onto_plane_source_to_target)
        .def_readwrite("use_lie_algebra_left_jacobian_plane_features", &TLSRegistration::use_lie_algebra_left_jacobian_plane_features)
        .def_readwrite("use_lie_algebra_right_jacobian_plane_features", &TLSRegistration::use_lie_algebra_right_jacobian_plane_features)
        .def_readwrite("point_to_plane_source_to_target_dot_product", &TLSRegistration::point_to_plane_source_to_target_dot_product)
        .def_readwrite("point_to_plane_source_to_target", &TLSRegistration::point_to_plane_source_to_target)
        .def_readwrite("plane_to_plane_source_to_target", &TLSRegistration::plane_to_plane_source_to_target)
        .def_readwrite("use_pgslam", &TLSRegistration::use_pgslam)
        .def_readwrite("calculate_offset", &TLSRegistration::calculate_offset)
        .def_readwrite("is_decimate", &TLSRegistration::is_decimate)
        .def_readwrite("bucket_x", &TLSRegistration::bucket_x)
        .def_readwrite("bucket_y", &TLSRegistration::bucket_y)
        .def_readwrite("bucket_z", &TLSRegistration::bucket_z)
        .def_readwrite("resso_upd_init", &TLSRegistration::resso_upd_init)
        .def_readwrite("resso_upd", &TLSRegistration::resso_upd)
        .def_readwrite("resso_upd_inv", &TLSRegistration::resso_upd_inv)
        .def_readwrite("initial_pose_to_identity", &TLSRegistration::initial_pose_to_identity)
        .def_readwrite("output_las_name", &TLSRegistration::output_las_name)
        .def_readwrite("save_as_separate_las", &TLSRegistration::save_as_separate_las)
        .def_readwrite("save_as_separate_laz", &TLSRegistration::save_as_separate_laz)
        .def_readwrite("trajectories_laz_name", &TLSRegistration::trajectories_laz_name)
        .def_readwrite("gnss_laz_name", &TLSRegistration::gnss_laz_name)
        .def_readwrite("scale_board_las_name", &TLSRegistration::scale_board_las_name)
        .def_readwrite("scale_board_dec", &TLSRegistration::scale_board_dec)
        .def_readwrite("scale_board_side_len", &TLSRegistration::scale_board_side_len)
        .def_readwrite("initial_poses_file_name", &TLSRegistration::initial_poses_file_name)
        .def_readwrite("poses_file_name", &TLSRegistration::poses_file_name)
        .def_readwrite("is_trajectory_export_downsampling", &TLSRegistration::is_trajectory_export_downsampling)
        .def_readwrite("curve_consecutive_distance_meters", &TLSRegistration::curve_consecutive_distance_meters)
        .def_readwrite("not_curve_consecutive_distance_meters", &TLSRegistration::not_curve_consecutive_distance_meters)
        .def_readwrite("trajectories_name", &TLSRegistration::trajectories_name)
        .def_readwrite("write_lidar_timestamp", &TLSRegistration::write_lidar_timestamp)
        .def_readwrite("write_unix_timestamp", &TLSRegistration::write_unix_timestamp)
        .def_readwrite("use_quaternions", &TLSRegistration::use_quaternions)
        .def_readwrite("save_to_dxf", &TLSRegistration::save_to_dxf)
        .def("set_zoller_frohlich_tls_imager_5006i_errors",
            &TLSRegistration::set_zoller_frohlich_tls_imager_5006i_errors,
            "Set the Zoller-Frohlich TLS Imager 5006i errors")
        .def("set_zoller_frohlich_tls_imager_5010c_errors",
            &TLSRegistration::set_zoller_frohlich_tls_imager_5010c_errors,
            "Set the Zoller-Frohlich TLS Imager 5010c errors")
        .def("set_zoller_frohlich_tls_imager_5016_errors",
            &TLSRegistration::set_zoller_frohlich_tls_imager_5016_errors,
            "Set the Zoller-Frohlich TLS Imager 5016 errors")
        .def("set_faro_focus3d_errors",
            &TLSRegistration::set_faro_focus3d_errors,
            "Set the FARO Focus3D errors")
        .def("set_leica_scanstation_c5_c10_errors",
            &TLSRegistration::set_leica_scanstation_c5_c10_errors,
            "Set the Leica ScanStation C5/C10 errors")
        .def("set_riegl_vz400_errors",
            &TLSRegistration::set_riegl_vz400_errors,
            "Set the Riegl VZ400 errors")
        .def("set_leica_hds6100_errors",
            &TLSRegistration::set_leica_hds6100_errors,
            "Set the Leica HDS6100 errors")
        .def("set_leica_p40_errors",
            &TLSRegistration::set_leica_p40_errors,
            "Set the Leica P40 errors");

    py::class_<ICP>(m, "ICP")
        .def(py::init<>())
        .def_readwrite("search_radious", &ICP::search_radious)
        .def_readwrite("number_of_threads", &ICP::number_of_threads)
        .def_readwrite("number_of_iterations", &ICP::number_of_iterations)
        .def_readwrite("is_adaptive_robust_kernel", &ICP::is_adaptive_robust_kernel)
        .def_readwrite("is_gauss_newton", &ICP::is_gauss_newton);

    py::class_<RegistrationPlaneFeature>(m, "RegistrationPlaneFeature")
        .def(py::init<>())
        .def_readwrite("search_radious", &RegistrationPlaneFeature::search_radious)
        .def_readwrite("number_of_threads", &RegistrationPlaneFeature::number_of_threads)
        .def_readwrite("number_of_iterations", &RegistrationPlaneFeature::number_of_iterations)
        .def_readwrite("is_adaptive_robust_kernel", &RegistrationPlaneFeature::is_adaptive_robust_kernel);

    py::class_<PoseGraphSLAM>(m, "PoseGraphSLAM")
        .def(py::init<>())
        .def_readwrite("overlap_threshold", &PoseGraphSLAM::overlap_threshold)
        .def_readwrite("iterations", &PoseGraphSLAM::iterations)
        .def_readwrite("search_radious", &PoseGraphSLAM::search_radious);

    py::class_<GNSS::GlobalPose>(m, "GlobalPose")
        .def(py::init<>())
        .def_readwrite("timestamp", &GNSS::GlobalPose::timestamp)
        .def_readwrite("lat", &GNSS::GlobalPose::lat)
        .def_readwrite("lon", &GNSS::GlobalPose::lon)
        .def_readwrite("alt", &GNSS::GlobalPose::alt);

    py::class_<GNSS>(m, "GNSS")
        .def(py::init<>())
        .def_readwrite("gnss_poses", &GNSS::gnss_poses)
        .def_readwrite("show_correspondences", &GNSS::show_correspondences)
        .def_readwrite("WGS84ReferenceLatitude", &GNSS::WGS84ReferenceLatitude)
        .def_readwrite("WGS84ReferenceLongitude", &GNSS::WGS84ReferenceLongitude);

    m.def("run_multi_view_tls_registration", &run_multi_view_tls_registration,
          py::arg("input_file_name"),
          py::arg("output_file_name"),
          py::arg("tls_registration"),
          "Run multi-view TLS registration with the provided input file, output file, and TLS registration configuration.");
}
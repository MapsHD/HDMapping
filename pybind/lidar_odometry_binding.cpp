#include <pybind11/pybind11.h>
#include <pybind11/stl/filesystem.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <structures.h>
#include <session.h>
#include <lidar_odometry.h>
#include <lidar_odometry_utils.h>

namespace py = pybind11;

PYBIND11_MODULE(lidar_odometry_py, m) {
    m.doc() = "Python bindings for lidar odometry";

    // Bind Point3Di
    py::class_<Point3Di>(m, "Point3Di")
        .def(py::init<>())
        .def_readwrite("point", &Point3Di::point)
        .def_readwrite("timestamp", &Point3Di::timestamp)
        .def_readwrite("intensity", &Point3Di::intensity)
        .def_readwrite("index_pose", &Point3Di::index_pose)
        .def_readwrite("lidarid", &Point3Di::lidarid)
        .def_readwrite("index_point", &Point3Di::index_point);

    // Bind WorkerData
    py::class_<WorkerData>(m, "WorkerData")
        .def(py::init<>())
        .def_readwrite("intermediate_points", &WorkerData::intermediate_points)
        .def_readwrite("original_points", &WorkerData::original_points)
        .def_readwrite("intermediate_trajectory", &WorkerData::intermediate_trajectory)
        .def_readwrite("intermediate_trajectory_motion_model", &WorkerData::intermediate_trajectory_motion_model)
        .def_readwrite("intermediate_trajectory_timestamps", &WorkerData::intermediate_trajectory_timestamps)
        .def_readwrite("imu_om_fi_ka", &WorkerData::imu_om_fi_ka);

    // Bind Session
    py::class_<Session>(m, "Session")
        .def(py::init<>())
        .def("load", &Session::load)
        .def("save", &Session::save);

    // Bind GridParameters
    py::class_<NDT::GridParameters>(m, "GridParameters")
        .def(py::init<>())
        .def_readwrite("bounding_box_min_X", &NDT::GridParameters::bounding_box_min_X)
        .def_readwrite("bounding_box_min_Y", &NDT::GridParameters::bounding_box_min_Y)
        .def_readwrite("bounding_box_min_Z", &NDT::GridParameters::bounding_box_min_Z)
        .def_readwrite("bounding_box_max_X", &NDT::GridParameters::bounding_box_max_X)
        .def_readwrite("bounding_box_max_Y", &NDT::GridParameters::bounding_box_max_Y)
        .def_readwrite("bounding_box_max_Z", &NDT::GridParameters::bounding_box_max_Z)
        .def_readwrite("bounding_box_extension", &NDT::GridParameters::bounding_box_extension)
        .def_readwrite("number_of_buckets_X", &NDT::GridParameters::number_of_buckets_X)
        .def_readwrite("number_of_buckets_Y", &NDT::GridParameters::number_of_buckets_Y)
        .def_readwrite("number_of_buckets_Z", &NDT::GridParameters::number_of_buckets_Z)
        .def_readwrite("number_of_buckets", &NDT::GridParameters::number_of_buckets)
        .def_readwrite("resolution_X", &NDT::GridParameters::resolution_X)
        .def_readwrite("resolution_Y", &NDT::GridParameters::resolution_Y)
        .def_readwrite("resolution_Z", &NDT::GridParameters::resolution_Z);

    // Bind Bucket
    py::class_<NDT::Bucket>(m, "Bucket")
        .def(py::init<>())
        .def_readwrite("index_begin", &NDT::Bucket::index_begin)
        .def_readwrite("index_end", &NDT::Bucket::index_end)
        .def_readwrite("number_of_points", &NDT::Bucket::number_of_points)
        .def_readwrite("mean", &NDT::Bucket::mean)
        .def_readwrite("cov", &NDT::Bucket::cov)
        .def_readwrite("normal_vector", &NDT::Bucket::normal_vector);

    // Bind PointBucketIndexPair
    py::class_<NDT::PointBucketIndexPair>(m, "PointBucketIndexPair")
        .def(py::init<>())
        .def_readwrite("index_of_point", &NDT::PointBucketIndexPair::index_of_point)
        .def_readwrite("index_of_bucket", &NDT::PointBucketIndexPair::index_of_bucket)
        .def_readwrite("index_pose", &NDT::PointBucketIndexPair::index_pose);

    // Bind LidarOdometryParams
    py::class_<LidarOdometryParams>(m, "LidarOdometryParams")
        .def(py::init<>())
        .def_readwrite("filter_threshold_xy_inner", &LidarOdometryParams::filter_threshold_xy_inner)
        .def_readwrite("filter_threshold_xy_outer", &LidarOdometryParams::filter_threshold_xy_outer)
        .def_readwrite("m_g", &LidarOdometryParams::m_g)
        .def_readwrite("initial_points", &LidarOdometryParams::initial_points)
        .def_readwrite("in_out_params_indoor", &LidarOdometryParams::in_out_params_indoor)
        .def_readwrite("in_out_params_outdoor", &LidarOdometryParams::in_out_params_outdoor)
        .def_readwrite("buckets_indoor", &LidarOdometryParams::buckets_indoor)
        .def_readwrite("buckets_outdoor", &LidarOdometryParams::buckets_outdoor)
        .def_readwrite("use_motion_from_previous_step", &LidarOdometryParams::use_motion_from_previous_step)
        .def_readwrite("consecutive_distance", &LidarOdometryParams::consecutive_distance)
        .def_readwrite("nr_iter", &LidarOdometryParams::nr_iter)
        .def_readwrite("useMultithread", &LidarOdometryParams::useMultithread)
        .def_readwrite("reference_points", &LidarOdometryParams::reference_points)
        .def_readwrite("decimation", &LidarOdometryParams::decimation)
        .def_readwrite("reference_buckets", &LidarOdometryParams::reference_buckets)
        .def_readwrite("working_directory_preview", &LidarOdometryParams::working_directory_preview)
        .def_readwrite("sliding_window_trajectory_length_threshold", &LidarOdometryParams::sliding_window_trajectory_length_threshold)
        .def_readwrite("save_calibration_validation", &LidarOdometryParams::save_calibration_validation)
        .def_readwrite("calibration_validation_points", &LidarOdometryParams::calibration_validation_points)
        .def_readwrite("max_distance", &LidarOdometryParams::max_distance)
        .def_readwrite("use_robust_and_accurate_lidar_odometry", &LidarOdometryParams::use_robust_and_accurate_lidar_odometry)
        .def_readwrite("distance_bucket", &LidarOdometryParams::distance_bucket)
        .def_readwrite("polar_angle_deg", &LidarOdometryParams::polar_angle_deg)
        .def_readwrite("azimutal_angle_deg", &LidarOdometryParams::azimutal_angle_deg)
        .def_readwrite("robust_and_accurate_lidar_odometry_iterations", &LidarOdometryParams::robust_and_accurate_lidar_odometry_iterations)
        .def_readwrite("max_distance_lidar", &LidarOdometryParams::max_distance_lidar)
        .def_readwrite("distance_bucket_rigid_sf", &LidarOdometryParams::distance_bucket_rigid_sf)
        .def_readwrite("polar_angle_deg_rigid_sf", &LidarOdometryParams::polar_angle_deg_rigid_sf)
        .def_readwrite("azimutal_angle_deg_rigid_sf", &LidarOdometryParams::azimutal_angle_deg_rigid_sf)
        .def_readwrite("robust_and_accurate_lidar_odometry_rigid_sf_iterations", &LidarOdometryParams::robust_and_accurate_lidar_odometry_rigid_sf_iterations)
        .def_readwrite("max_distance_lidar_rigid_sf", &LidarOdometryParams::max_distance_lidar_rigid_sf)
        .def_readwrite("rgd_sf_sigma_x_m", &LidarOdometryParams::rgd_sf_sigma_x_m)
        .def_readwrite("rgd_sf_sigma_y_m", &LidarOdometryParams::rgd_sf_sigma_y_m)
        .def_readwrite("rgd_sf_sigma_z_m", &LidarOdometryParams::rgd_sf_sigma_z_m)
        .def_readwrite("rgd_sf_sigma_om_deg", &LidarOdometryParams::rgd_sf_sigma_om_deg)
        .def_readwrite("rgd_sf_sigma_fi_deg", &LidarOdometryParams::rgd_sf_sigma_fi_deg)
        .def_readwrite("rgd_sf_sigma_ka_deg", &LidarOdometryParams::rgd_sf_sigma_ka_deg)
        .def_readwrite("total_length_of_calculated_trajectory", &LidarOdometryParams::total_length_of_calculated_trajectory)
        .def_readwrite("fusionConventionNwu", &LidarOdometryParams::fusionConventionNwu)
        .def_readwrite("fusionConventionEnu", &LidarOdometryParams::fusionConventionEnu)
        .def_readwrite("fusionConventionNed", &LidarOdometryParams::fusionConventionNed)
        .def_readwrite("threshold_initial_points", &LidarOdometryParams::threshold_initial_points)
        .def_readwrite("apply_consistency", &LidarOdometryParams::apply_consistency)
        .def_readwrite("use_mutliple_gaussian", &LidarOdometryParams::use_mutliple_gaussian)
        .def_readwrite("num_constistency_iter", &LidarOdometryParams::num_constistency_iter)
        .def_readwrite("threshould_output_filter", &LidarOdometryParams::threshould_output_filter)
        .def_readwrite("ahrs_gain", &LidarOdometryParams::ahrs_gain)
        .def_readwrite("threshold_nr_poses", &LidarOdometryParams::threshold_nr_poses)
        .def_readwrite("current_output_dir", &LidarOdometryParams::current_output_dir)
        .def_readwrite("min_counter", &LidarOdometryParams::min_counter)
        ;

    // Bind run_lidar_odometry function
    m.def("run_lidar_odometry", &run_lidar_odometry,
        py::arg("input_dir"),
        py::arg("params"),
        "Run lidar odometry with given parameters");

    // Bind run_consistency function
    m.def("run_consistency", &run_consistency,
        py::arg("worker_data"),
        py::arg("params"),
        "Run trajectory consistency.");
    
    // Bind save_results_automatic function
    m.def("save_results_automatic", &save_results_automatic,
        py::arg("params"),
        py::arg("worker_data"),
        py::arg("working_directory"),
        py::arg("elapsed_seconds"),
        "Save output results from lidar odometry pipeline.");

    // Bind save_all_to_las function
    m.def("save_all_to_las", &save_all_to_las,
        py::arg("worker_data"),
        py::arg("params"),
        py::arg("output_file_name"),
        py::arg("session"),
        py::arg("export_selected"),
        py::arg("filter_on_export"),
        py::arg("apply_pose"),
        py::arg("add_to_pc_container"),
        "Save output point cloud to a single file.");

    // Bind functions from utils
    m.def("loadLaz", &loadLaz,
        py::arg("filename"),
        py::arg("points_out"),
        py::arg("index_poses_wd"),
        py::arg("intermediate_trajectory"),
        py::arg("inverse_pose"),
        "Load .laz file to reconstruct WorkerData points");
    
    m.def("load_poses", &load_poses,
        py::arg("poses_file"),
        py::arg("out_poses"),
        "Load poses from file");

    m.def("load_trajectory_csv", &load_trajectory_csv,
        py::arg("filename"),
        py::arg("m_pose"),
        py::arg("intermediate_trajectory_timestamps"),
        py::arg("intermediate_trajectory"),
        py::arg("imu_om_fi_ka"),
        "Load trajectory CSV file");

    m.def("load_point_sizes", &load_point_sizes,
        py::arg("path"),
        py::arg("vector"),
        "Load original point sizes from JSON file");

    m.def("load_index_poses", &load_index_poses,
        py::arg("path"),
        py::arg("vector"),
        "Load index poses from JSON file");

m.def("load_worker_data_from_results",
    [](const fs::path& session_file) {
        std::vector<WorkerData> worker_data_out;
        if (!load_worker_data_from_results(session_file, worker_data_out)) {
            throw std::runtime_error("Failed to load worker data from session file: " + session_file.string());
        }
        return worker_data_out;
    },
    py::arg("session_file"),
    "Load worker data from session file results and return it as a list.");
}
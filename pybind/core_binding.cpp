#include <pybind11/pybind11.h>
#include <pybind11/stl/filesystem.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <structures.h>
#include <session.h>
#include <export_laz.h>

namespace py = pybind11;

PYBIND11_MODULE(core_py, m) {
    m.doc() = "Python bindings for HDMapping core";

    // Bind Point3Di
    py::class_<Point3Di>(m, "Point3Di")
        .def(py::init<>())
        .def_readwrite("point", &Point3Di::point)
        .def_readwrite("timestamp", &Point3Di::timestamp)
        .def_readwrite("intensity", &Point3Di::intensity)
        .def_readwrite("index_pose", &Point3Di::index_pose)
        .def_readwrite("lidarid", &Point3Di::lidarid)
        .def_readwrite("index_point", &Point3Di::index_point)
        ;

    // Bind WorkerData
    py::class_<WorkerData>(m, "WorkerData")
        .def(py::init<>())
        .def_readwrite("intermediate_points_cache_file_name", &WorkerData::intermediate_points_cache_file_name)
        .def_readwrite("original_points_cache_file_name", &WorkerData::original_points_cache_file_name)
        .def_readwrite("original_points_to_save_cache_file_name", &WorkerData::original_points_to_save_cache_file_name)
        .def_readwrite("intermediate_trajectory", &WorkerData::intermediate_trajectory)
        .def_readwrite("intermediate_trajectory_motion_model", &WorkerData::intermediate_trajectory_motion_model)
        .def_readwrite("intermediate_trajectory_timestamps", &WorkerData::intermediate_trajectory_timestamps)
        .def_readwrite("imu_om_fi_ka", &WorkerData::imu_om_fi_ka)
        .def_readwrite("show", &WorkerData::show)
        ;

    // Bind Session
    py::class_<Session>(m, "Session")
        .def(py::init<>())
        .def("load", &Session::load)
        .def("save", &Session::save)
        .def("fill_session_from_worker_data", &Session::fill_session_from_worker_data)
        ;

    // Bind save_all_to_las
    m.def("save_all_to_las", &save_all_to_las,
          py::arg("session"),
          py::arg("output_las_name"),
          py::arg("as_local") = false,
          "Saves point cloud data from session into a .las/.laz file.");
}
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <vector>
#include <string>

namespace py = pybind11;


int run_multi_session_registration(
    const std::vector<std::string>& session_files);


PYBIND11_MODULE(multi_session_registration_py, m)
{
    m.def(
        "run",
        &run_multi_session_registration,
        "Load sessions and export to TUM"
    );
}
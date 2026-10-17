cmake_minimum_required(VERSION 4.0.0)

project(easy_gui_tests)

# Unit tests for apps/easy_gui's easy_gui_core (everything but the window).
# Doctest, like the other suites. The pre-flight and staging tests write small
# synthetic Mandeye recordings to the temp directory; the progress parser test
# replays an excerpt of a real lidar_odometry_step_1 log.
add_executable(easy_gui_tests
    test_main.cpp
    test_child_process.cpp
    test_pipeline_integration.cpp
    test_preflight.cpp
    test_progress_parser.cpp
    test_staging_and_preset.cpp
)

target_include_directories(easy_gui_tests PRIVATE ${THIRDPARTY_DIRECTORY}/doctest)
target_link_libraries(easy_gui_tests PRIVATE easy_gui_core)

if (MSVC)
    target_compile_options(easy_gui_tests PRIVATE /bigobj)
endif()

include(CTest)
add_test(NAME easy_gui_tests COMMAND easy_gui_tests)

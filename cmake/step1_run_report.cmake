# Shared by the odometry applications, viewers/calibration tools and Python bindings.
add_library(step1_run_report STATIC
    ${REPOSITORY_DIRECTORY}/apps/lidar_odometry_step_1/run_report.cpp
    ${REPOSITORY_DIRECTORY}/apps/lidar_odometry_step_1/run_report.h
)
target_include_directories(step1_run_report PUBLIC
    ${REPOSITORY_DIRECTORY}/apps/lidar_odometry_step_1
    ${THIRDPARTY_DIRECTORY}/json/include
)
target_compile_features(step1_run_report PUBLIC cxx_std_20)
target_link_libraries(step1_run_report PUBLIC Threads::Threads)
# This static library is also linked into the Python extension module on Linux/macOS.
set_target_properties(step1_run_report PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(WIN32)
    target_link_libraries(step1_run_report PRIVATE psapi pdh)
endif()

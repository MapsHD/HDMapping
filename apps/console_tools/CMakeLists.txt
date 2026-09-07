cmake_minimum_required(VERSION 4.0.0)

project(console_tools)

add_executable(matrix_mul matrix_mul.cpp)

target_include_directories(
  matrix_mul PRIVATE include ${REPOSITORY_DIRECTORY}/core/include
                     ${EIGEN3_INCLUDE_DIR})

target_link_libraries(matrix_mul PRIVATE ${PLATFORM_MISCELLANEOUS_LIBS} spdlog::spdlog)


add_executable(multiply_timestamps_session_point_cloud_laz multiply_timestamps_session_point_cloud_laz.cpp)

target_include_directories(
  multiply_timestamps_session_point_cloud_laz PRIVATE include ${REPOSITORY_DIRECTORY}/core/include
                     ${EIGEN3_INCLUDE_DIR}
                     ${LASZIP_INCLUDE_DIR}/LASzip/include)

target_link_libraries(multiply_timestamps_session_point_cloud_laz PRIVATE ${PLATFORM_MISCELLANEOUS_LIBS} ${PLATFORM_LASZIP_LIB} spdlog::spdlog)

add_executable(multiply_timestamps_session_trajectory_csv multiply_timestamps_session_trajectory_csv.cpp)

target_include_directories(
  multiply_timestamps_session_trajectory_csv PRIVATE include ${REPOSITORY_DIRECTORY}/core/include
                     ${EIGEN3_INCLUDE_DIR})

target_link_libraries(multiply_timestamps_session_trajectory_csv PRIVATE ${PLATFORM_MISCELLANEOUS_LIBS} spdlog::spdlog)

add_executable(laz_to_ply laz_to_ply.cpp)
target_link_libraries(laz_to_ply PRIVATE ${PLATFORM_LASZIP_LIB} spdlog::spdlog)
target_include_directories(laz_to_ply PRIVATE ${LASZIP_INCLUDE_DIR}/LASzip/include ${PROJECT_BINARY_DIR}/include)

add_executable(laz_to_pcd laz_to_pcd.cpp)
target_link_libraries(laz_to_pcd PRIVATE ${PLATFORM_LASZIP_LIB} spdlog::spdlog)
target_include_directories(laz_to_pcd PRIVATE ${LASZIP_INCLUDE_DIR}/LASzip/include ${PROJECT_BINARY_DIR}/include)

add_executable(pcd_to_laz pcd_to_laz.cpp)
target_link_libraries(pcd_to_laz PRIVATE ${PLATFORM_LASZIP_LIB} spdlog::spdlog)
target_include_directories(pcd_to_laz PRIVATE ${LASZIP_INCLUDE_DIR}/LASzip/include ${PROJECT_BINARY_DIR}/include)

add_executable(laz_to_txt laz_to_txt.cpp)
target_link_libraries(laz_to_txt PRIVATE ${PLATFORM_LASZIP_LIB} spdlog::spdlog)
target_include_directories(laz_to_txt PRIVATE ${LASZIP_INCLUDE_DIR}/LASzip/include ${PROJECT_BINARY_DIR}/include)

# laz_to_mcap: LAZ (+ optional IMU csv, or a whole mandeye session directory)
# -> MCAP/ROS2 bag exporter. Reuses load_point_cloud()/load_imu() from
# lidar_odometry_step_1/lidar_odometry_utils.cpp (not load_data(), which
# additionally drops the first LAZ file's points as part of that app's own
# odometry preprocessing -- wrong for a lossless exporter), so it needs
# TBB/spdlog/laszip/core plus that file's own header-only dependencies
# (glm/toml++/json/observation_equations/vqf), but none of the GUI libs.
add_executable(
  laz_to_mcap laz_to_mcap.cpp
  ${REPOSITORY_DIRECTORY}/apps/lidar_odometry_step_1/lidar_odometry_utils.h
  ${REPOSITORY_DIRECTORY}/apps/lidar_odometry_step_1/lidar_odometry_utils.cpp
  ${REPOSITORY_DIRECTORY}/rosbags/McapWriter.h ${REPOSITORY_DIRECTORY}/rosbags/McapWriter.cpp
  )

target_include_directories(
  laz_to_mcap
  PRIVATE ${REPOSITORY_DIRECTORY}/apps/lidar_odometry_step_1
          ${REPOSITORY_DIRECTORY}/core/include
          ${REPOSITORY_DIRECTORY}/rosbags
          ${THIRDPARTY_DIRECTORY} # csv.hpp (used by load_imu)
          ${THIRDPARTY_DIRECTORY}/glm
          ${EIGEN3_INCLUDE_DIR}
          ${THIRDPARTY_DIRECTORY}/tomlplusplus/include
          ${THIRDPARTY_DIRECTORY}/json/include
          ${LASZIP_INCLUDE_DIR}/LASzip/include
          ${THIRDPARTY_DIRECTORY}/observation_equations/codes
          ${THIRDPARTY_DIRECTORY}/vqf/vqf/cpp)

target_link_libraries(
          laz_to_mcap
          PRIVATE
          mcap
          unordered_dense::unordered_dense
          spdlog::spdlog
          ${PLATFORM_LASZIP_LIB}
          ${PLATFORM_MISCELLANEOUS_LIBS}
          ${CORE_LIBRARIES})

if (MSVC)
    target_compile_options(laz_to_mcap PRIVATE /bigobj)
endif()

# bow_loop_closure_detect: headless verification tool for the automatic
# image-based (fbow) loop closure detector shared with step 2's GUI --
# loads a step-2 session + a CAMERA_0 folder and prints candidate pairs as
# CSV, without needing the interactive GUI. Links core_no_gui (WITH_GUI=0)
# since it only needs PointClouds/Session, not the ImGui-dependent
# ManualPoseGraphLoopClosure/observation_picking pieces core (WITH_GUI=1)
# adds.
add_executable(
  bow_loop_closure_detect bow_loop_closure_detect.cpp
  ${REPOSITORY_DIRECTORY}/apps/multi_view_tls_registration/bow_loop_closure_detector.h
  ${REPOSITORY_DIRECTORY}/apps/multi_view_tls_registration/bow_loop_closure_detector.cpp
)

target_compile_definitions(bow_loop_closure_detect PRIVATE
    WITH_GUI=0
    -DFBOW_VOCABULARY_PATH="${THIRDPARTY_DIRECTORY}/fbow/vocabularies/orb_mur.fbow")

target_include_directories(
  bow_loop_closure_detect PRIVATE
  ${REPOSITORY_DIRECTORY}/core/include
  ${REPOSITORY_DIRECTORY}/apps/multi_view_tls_registration
  ${EIGEN3_INCLUDE_DIR}
  ${LASZIP_INCLUDE_DIR}/LASzip/include
  ${THIRDPARTY_DIRECTORY}/json/include
  ${THIRDPARTY_DIRECTORY}/observation_equations/codes
  ${THIRDPARTY_DIRECTORY}/vqf/vqf/cpp
  ${THIRDPARTY_DIRECTORY}/Fusion/Fusion
  ${EXTERNAL_LIBRARIES_DIRECTORY}/include
  ${FBOW_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS})

target_link_libraries(
  bow_loop_closure_detect PRIVATE
  core_no_gui
  fbow
  ${OpenCV_LIBS}
  ${PLATFORM_LASZIP_LIB}
  vqf
  Fusion
  ${PLATFORM_MISCELLANEOUS_LIBS}
  spdlog::spdlog)
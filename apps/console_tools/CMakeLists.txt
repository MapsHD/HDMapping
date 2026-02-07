cmake_minimum_required(VERSION 4.0.0)

project(console_tools)

add_executable(matrix_mul matrix_mul.cpp)

target_include_directories(
  matrix_mul PRIVATE include ${REPOSITORY_DIRECTORY}/core/include
                     ${EIGEN3_INCLUDE_DIR})

target_link_libraries(matrix_mul PRIVATE ${PLATFORM_MISCELLANEOUS_LIBS})


add_executable(multiply_timestamps_session_point_cloud_laz multiply_timestamps_session_point_cloud_laz.cpp)

target_include_directories(
  multiply_timestamps_session_point_cloud_laz PRIVATE include ${REPOSITORY_DIRECTORY}/core/include
                     ${EIGEN3_INCLUDE_DIR}
                     ${LASZIP_INCLUDE_DIR}/LASzip/include)

target_link_libraries(multiply_timestamps_session_point_cloud_laz PRIVATE ${PLATFORM_MISCELLANEOUS_LIBS} ${PLATFORM_LASZIP_LIB})

add_executable(multiply_timestamps_session_trajectory_csv multiply_timestamps_session_trajectory_csv.cpp)

target_include_directories(
  multiply_timestamps_session_trajectory_csv PRIVATE include ${REPOSITORY_DIRECTORY}/core/include
                     ${EIGEN3_INCLUDE_DIR})

target_link_libraries(multiply_timestamps_session_trajectory_csv PRIVATE ${PLATFORM_MISCELLANEOUS_LIBS})

add_executable(laz_to_ply laz_to_ply.cpp)
target_link_libraries(laz_to_ply PRIVATE ${PLATFORM_LASZIP_LIB})
target_include_directories(laz_to_ply PRIVATE ${LASZIP_INCLUDE_DIR}/LASzip/include ${PROJECT_BINARY_DIR}/include)

add_executable(laz_to_pcd laz_to_pcd.cpp)
target_link_libraries(laz_to_pcd PRIVATE ${PLATFORM_LASZIP_LIB})
target_include_directories(laz_to_pcd PRIVATE ${LASZIP_INCLUDE_DIR}/LASzip/include ${PROJECT_BINARY_DIR}/include)
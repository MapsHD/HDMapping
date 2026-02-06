cmake_minimum_required(VERSION 3.15.0)

project(laz_to_pcd)

add_executable(laz_to_pcd laz_to_pcd.cpp)
target_link_libraries(laz_to_pcd PRIVATE ${PLATFORM_LASZIP_LIB})
target_include_directories(laz_to_pcd PRIVATE ${LASZIP_INCLUDE_DIR}/LASzip/include ${PROJECT_BINARY_DIR}/include)
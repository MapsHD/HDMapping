cmake_minimum_required(VERSION 3.15.0)

project(matrix_mul)

add_executable(matrix_mul matrix_mul.cpp)

target_include_directories(
  matrix_mul PRIVATE include ${REPOSITORY_DIRECTORY}/core/include
                     ${EIGEN3_INCLUDE_DIR})

target_link_libraries(matrix_mul PRIVATE ${PLATFORM_MISCELLANEOUS_LIBS})

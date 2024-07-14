cmake_minimum_required(VERSION 3.15.0)

project(matrix_mul)

add_executable(matrix_mul matrix_mul.cpp)

target_include_directories(
  matrix_mul PRIVATE include ${REPOSITORY_DIRECTORY}/core/include
                     ${EXTERNAL_LIBRARIES_DIRECTORY}/eigen)

target_link_libraries(mnatrix_mul PRIVATE ${PLATFORM_MISCELLANEOUS_LIBS})

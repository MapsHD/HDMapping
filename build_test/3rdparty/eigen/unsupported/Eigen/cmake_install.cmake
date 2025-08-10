# Install script for directory: D:/@Github/HDMapping085-2025-08-08_17/3rdparty/eigen/unsupported/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files/hd-mapping")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Devel" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE FILE FILES
    "D:/@Github/HDMapping085-2025-08-08_17/3rdparty/eigen/unsupported/Eigen/AdolcForward"
    "D:/@Github/HDMapping085-2025-08-08_17/3rdparty/eigen/unsupported/Eigen/AlignedVector3"
    "D:/@Github/HDMapping085-2025-08-08_17/3rdparty/eigen/unsupported/Eigen/ArpackSupport"
    "D:/@Github/HDMapping085-2025-08-08_17/3rdparty/eigen/unsupported/Eigen/AutoDiff"
    "D:/@Github/HDMapping085-2025-08-08_17/3rdparty/eigen/unsupported/Eigen/BVH"
    "D:/@Github/HDMapping085-2025-08-08_17/3rdparty/eigen/unsupported/Eigen/EulerAngles"
    "D:/@Github/HDMapping085-2025-08-08_17/3rdparty/eigen/unsupported/Eigen/FFT"
    "D:/@Github/HDMapping085-2025-08-08_17/3rdparty/eigen/unsupported/Eigen/IterativeSolvers"
    "D:/@Github/HDMapping085-2025-08-08_17/3rdparty/eigen/unsupported/Eigen/KroneckerProduct"
    "D:/@Github/HDMapping085-2025-08-08_17/3rdparty/eigen/unsupported/Eigen/LevenbergMarquardt"
    "D:/@Github/HDMapping085-2025-08-08_17/3rdparty/eigen/unsupported/Eigen/MatrixFunctions"
    "D:/@Github/HDMapping085-2025-08-08_17/3rdparty/eigen/unsupported/Eigen/MoreVectorization"
    "D:/@Github/HDMapping085-2025-08-08_17/3rdparty/eigen/unsupported/Eigen/MPRealSupport"
    "D:/@Github/HDMapping085-2025-08-08_17/3rdparty/eigen/unsupported/Eigen/NNLS"
    "D:/@Github/HDMapping085-2025-08-08_17/3rdparty/eigen/unsupported/Eigen/NonLinearOptimization"
    "D:/@Github/HDMapping085-2025-08-08_17/3rdparty/eigen/unsupported/Eigen/NumericalDiff"
    "D:/@Github/HDMapping085-2025-08-08_17/3rdparty/eigen/unsupported/Eigen/OpenGLSupport"
    "D:/@Github/HDMapping085-2025-08-08_17/3rdparty/eigen/unsupported/Eigen/Polynomials"
    "D:/@Github/HDMapping085-2025-08-08_17/3rdparty/eigen/unsupported/Eigen/Skyline"
    "D:/@Github/HDMapping085-2025-08-08_17/3rdparty/eigen/unsupported/Eigen/SparseExtra"
    "D:/@Github/HDMapping085-2025-08-08_17/3rdparty/eigen/unsupported/Eigen/SpecialFunctions"
    "D:/@Github/HDMapping085-2025-08-08_17/3rdparty/eigen/unsupported/Eigen/Splines"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Devel" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE DIRECTORY FILES "D:/@Github/HDMapping085-2025-08-08_17/3rdparty/eigen/unsupported/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("D:/@Github/HDMapping085-2025-08-08_17/build_test/3rdparty/eigen/unsupported/Eigen/CXX11/cmake_install.cmake")

endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "D:/@Github/HDMapping085-2025-08-08_17/build_test/3rdparty/eigen/unsupported/Eigen/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()

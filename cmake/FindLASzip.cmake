# - Try to find the LASzip library
# Once done, this will define
#  LASZIP_FOUND - system has LASzip
#  LASZIP_INCLUDE_DIR - the LASzip include directory
#  LASZIP_LIBRARY - the LASzip library
#  LASZIP_VERSION - the version of the LASzip library (optional)

# Find the LASzip library
find_path(LASZIP_INCLUDE_DIR
  NAMES laszip/laszip_api.h
  PATHS
    /usr/local/include
    /usr/include
    /usr/include/laszip
    /opt/local/include
    ${CMAKE_INSTALL_PREFIX}/include
)

find_library(LASZIP_LIBRARY
  NAMES liblaszip.so
  PATHS
    /usr/local/lib
    /usr/lib/x86_64-linux-gnu/
    ${CMAKE_INSTALL_PREFIX}/lib
)

# Handle the standard arguments for the package
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LASzip
  REQUIRED_VARS LASZIP_LIBRARY LASZIP_INCLUDE_DIR
)

# If LASzip is found, set the result variables
if(LASZIP_FOUND)
  set(LASZIP_LIBRARIES ${LASZIP_LIBRARY})
  set(LASZIP_INCLUDE_DIRS ${LASZIP_INCLUDE_DIR})
endif()

mark_as_advanced(LASZIP_INCLUDE_DIR LASZIP_LIBRARY)

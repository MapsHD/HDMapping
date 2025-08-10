# This file exports the Eigen3::Eigen CMake target which should be passed to the
# target_link_libraries command.


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was Eigen3Config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

####################################################################################

if (NOT TARGET Eigen3::Eigen)
  include ("${CMAKE_CURRENT_LIST_DIR}/Eigen3Targets.cmake")
endif (NOT TARGET Eigen3::Eigen)

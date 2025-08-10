#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "FreeGLUT::freeglut" for configuration "Debug"
set_property(TARGET FreeGLUT::freeglut APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(FreeGLUT::freeglut PROPERTIES
  IMPORTED_IMPLIB_DEBUG "${_IMPORT_PREFIX}/lib/freeglutd.lib"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/freeglutd.dll"
  )

list(APPEND _cmake_import_check_targets FreeGLUT::freeglut )
list(APPEND _cmake_import_check_files_for_FreeGLUT::freeglut "${_IMPORT_PREFIX}/lib/freeglutd.lib" "${_IMPORT_PREFIX}/bin/freeglutd.dll" )

# Import target "FreeGLUT::freeglut_static" for configuration "Debug"
set_property(TARGET FreeGLUT::freeglut_static APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(FreeGLUT::freeglut_static PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "C;RC"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/freeglut_staticd.lib"
  )

list(APPEND _cmake_import_check_targets FreeGLUT::freeglut_static )
list(APPEND _cmake_import_check_files_for_FreeGLUT::freeglut_static "${_IMPORT_PREFIX}/lib/freeglut_staticd.lib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

# Shim FindXercesC module for HDMapping.
#
# HDMapping vendors Apache Xerces-C as a git submodule (3rdparty/xerces-c) and
# builds it from source via add_subdirectory() in cmake/dependencies.cmake.
# libE57Format (3rdparty/libE57Format) calls `find_package( XercesC REQUIRED )`
# expecting a system-installed Xerces; this shim redirects that lookup to the
# in-tree `xerces-c` target instead so no system package is needed.
#
# cmake/ is first on CMAKE_MODULE_PATH (see top-level CMakeLists.txt), so this
# file wins over CMake's built-in FindXercesC when libE57Format is configured.

if(TARGET xerces-c)
    if(NOT TARGET XercesC::XercesC)
        # ALIAS can't point at a non-GLOBAL target from another directory on
        # older CMake, so wrap it in an INTERFACE IMPORTED target instead.
        add_library(XercesC::XercesC INTERFACE IMPORTED)
        target_link_libraries(XercesC::XercesC INTERFACE xerces-c)
    endif()

    set(XercesC_FOUND TRUE)
    set(XERCESC_FOUND TRUE)
    # Version of the vendored submodule (keep in sync with 3rdparty/xerces-c).
    set(XercesC_VERSION "3.3.0")
    set(XercesC_VERSION_STRING "3.3.0")
    set(XercesC_LIBRARIES XercesC::XercesC)
    set(XercesC_INCLUDE_DIRS "")  # carried transitively by the xerces-c target

    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(XercesC
        REQUIRED_VARS XercesC_LIBRARIES
        VERSION_VAR XercesC_VERSION)
else()
    message(FATAL_ERROR
        "FindXercesC shim: the bundled `xerces-c` target does not exist yet. "
        "add_subdirectory(3rdparty/xerces-c) must run before anything that "
        "calls find_package(XercesC) -- see cmake/dependencies.cmake.")
endif()

cmake_minimum_required(VERSION 3.15.0)

project(add_exif_to_photots)
option(BUILD_ADD_EXIF_TO_PHOTOS "Build Add Exif to photos" OFF)
if (BUILD_ADD_EXIF_TO_PHOTOS)
    include(FetchContent)

    # Fetch exiv2
    FetchContent_Declare(
            exiv2
            GIT_REPOSITORY https://github.com/Exiv2/exiv2.git
            GIT_TAG v0.28.2
    )

    set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)
    set(EXIV2_ENABLE_INIH OFF CACHE BOOL "" FORCE)
    set(EXIV2_BUILD_UNIT_TESTS OFF CACHE BOOL "" FORCE)
    set(EXIV2_BUILD_SAMPLES OFF CACHE BOOL "" FORCE)
    set(EXIV2_BUILD_EXIV2_COMMAND OFF CACHE BOOL "" FORCE)
    set(EXIV2_INSTALL ON CACHE BOOL "" FORCE)
    # Make available
    FetchContent_MakeAvailable(exiv2)

    add_executable(add_exif_to_photots add_exif_to_photots.cpp)
    target_include_directories(add_exif_to_photots PRIVATE ${CMAKE_BINARY_DIR}) # For the generated exiv2lib_export.h
    target_link_libraries(add_exif_to_photots PRIVATE exiv2lib)
endif ()
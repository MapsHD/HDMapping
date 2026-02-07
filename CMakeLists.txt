cmake_minimum_required(VERSION 4.0.0)

# Set minimum policy version to avoid compatibility issues with subprojects
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# Export compile commands for static analysis and tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(hd-mapping)

set (HDMAPPING_VERSION_MAJOR 0)
set (HDMAPPING_VERSION_MINOR 97)
set (HDMAPPING_VERSION_PATCH 0)

add_definitions(-DHDMAPPING_VERSION_MAJOR=${HDMAPPING_VERSION_MAJOR})
add_definitions(-DHDMAPPING_VERSION_MINOR=${HDMAPPING_VERSION_MINOR})
add_definitions(-DHDMAPPING_VERSION_PATCH=${HDMAPPING_VERSION_PATCH})

if (MSVC)
    add_definitions(-DNOMINMAX)
    add_definitions(-DWIN32_LEAN_AND_MEAN)
endif()

option(UTL_PROFILER_DISABLE "Disable UTL profiler (zero overhead when ON)" ON)

get_filename_component(REPOSITORY_DIRECTORY ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
set(THIRDPARTY_DIRECTORY ${REPOSITORY_DIRECTORY}/3rdparty)
set(THIRDPARTY_DIRECTORY_BINARY ${REPOSITORY_DIRECTORY}/3rdpartyBinary)
set(EXTERNAL_LIBRARIES_DIRECTORY ${REPOSITORY_DIRECTORY}/external)


# CPU Architecture optimization options
set(HD_CPU_OPTIMIZATION "AUTO" CACHE STRING "CPU optimization target (AUTO, INTEL, AMD, ARM, GENERIC)")
set_property(CACHE HD_CPU_OPTIMIZATION PROPERTY STRINGS AUTO INTEL AMD ARM GENERIC)

if (MSVC)
    add_definitions(-D_HAS_STD_BYTE=0)
    
    # Base optimizations for Visual Studio
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2 /GL")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /O2 /GL")
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /LTCG")
    set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /LTCG")
    set(CMAKE_STATIC_LINKER_FLAGS_RELEASE "${CMAKE_STATIC_LINKER_FLAGS_RELEASE} /LTCG")
    
    # Architecture-specific optimizations
    if(HD_CPU_OPTIMIZATION STREQUAL "AMD")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Oi /Ot /Oy /fp:fast")
        set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /Oi /Ot /Oy /fp:fast")
        message(STATUS "Enabling AMD-optimized build for MSVC")
    elseif(HD_CPU_OPTIMIZATION STREQUAL "INTEL")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2")
        set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /O2")
        message(STATUS "Enabling Intel-optimized build for MSVC")
    elseif(HD_CPU_OPTIMIZATION STREQUAL "ARM")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2")
        set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /O2")
        message(STATUS "Enabling ARM-optimized build for MSVC")
    elseif(HD_CPU_OPTIMIZATION STREQUAL "AUTO")
        # Auto-detect based on processor
        cmake_host_system_information(RESULT CPU_VENDOR QUERY PROCESSOR_DESCRIPTION)
        cmake_host_system_information(RESULT CPU_ARCH QUERY PROCESSOR_DESCRIPTION)
        if(CPU_ARCH MATCHES "arm" OR CPU_ARCH MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
            set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2")
            set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /O2")
            message(STATUS "Auto-detected ARM processor - enabling ARM optimizations")
        elseif(CPU_VENDOR MATCHES "AMD")
            set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Oi /Ot /Oy /fp:fast")
            set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /Oi /Ot /Oy /fp:fast")
            message(STATUS "Auto-detected AMD processor - enabling AMD optimizations")
        else()
            message(STATUS "Auto-detected Intel/Generic processor - using conservative optimizations")
        endif()
    else()
        message(STATUS "Using generic optimizations for MSVC")
    endif()
else()
    # Architecture-specific optimizations for GCC/Clang
    if(HD_CPU_OPTIMIZATION STREQUAL "AMD")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=x86-64 -mtune=generic -DNDEBUG")
        set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=x86-64 -mtune=generic -DNDEBUG")
        message(STATUS "Enabling AMD-optimized build for GCC/Clang")
    elseif(HD_CPU_OPTIMIZATION STREQUAL "INTEL")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -march=x86-64 -mtune=intel -DNDEBUG")
        set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O2 -march=x86-64 -mtune=intel -DNDEBUG")
        message(STATUS "Enabling Intel-optimized build for GCC/Clang")
    elseif(HD_CPU_OPTIMIZATION STREQUAL "ARM")
        if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
            set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=armv8-a -DNDEBUG")
            set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=armv8-a -DNDEBUG")
            message(STATUS "Enabling ARM64/AArch64 optimizations for GCC/Clang")
        else()
            set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=armv7-a -DNDEBUG")
            set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=armv7-a -DNDEBUG")
            message(STATUS "Enabling ARM32 optimizations for GCC/Clang")
        endif()
    elseif(HD_CPU_OPTIMIZATION STREQUAL "AUTO")
        # Auto-detect based on processor
        cmake_host_system_information(RESULT CPU_VENDOR QUERY PROCESSOR_DESCRIPTION)
        if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
            if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
                set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=armv8-a -DNDEBUG")
                set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=armv8-a -DNDEBUG")
                message(STATUS "Auto-detected ARM64 processor - enabling ARM optimizations")
            else()
                set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=armv7-a -DNDEBUG")
                set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=armv7-a -DNDEBUG")
                message(STATUS "Auto-detected ARM32 processor - enabling ARM optimizations")
            endif()
        elseif(CPU_VENDOR MATCHES "AMD")
            set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=x86-64 -mtune=generic -DNDEBUG")
            set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=x86-64 -mtune=generic -DNDEBUG")
            message(STATUS "Auto-detected AMD processor - enabling AMD optimizations")
        else()
            set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -march=x86-64 -mtune=intel -DNDEBUG")
            set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O2 -march=x86-64 -mtune=intel -DNDEBUG")
            message(STATUS "Auto-detected Intel/Generic processor - using Intel optimizations")
        endif()
    else()
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -march=x86-64 -DNDEBUG")
        set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O2 -march=x86-64 -DNDEBUG")
        message(STATUS "Using generic optimizations for GCC/Clang")
    endif()
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")

include_directories(shared/include)

set(OpenGL_GL_PREFERENCE GLVND)
set(CMAKE_CXX_STANDARD 20)

# SIMD optimizations based on CPU type and availability
include(CheckCXXCompilerFlag)
if (MSVC)
    if(HD_CPU_OPTIMIZATION STREQUAL "AMD_AVX512")
        check_cxx_compiler_flag("/arch:AVX512" HAS_AVX512)
        if(HAS_AVX512)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:AVX512")
            set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /arch:AVX512")
            message(STATUS "Enabling AVX-512 optimizations for AMD Zen 4+ processor (forced)")
            message(WARNING "AVX-512 support is limited to AMD Zen 4 (Ryzen 7000+) and later")
        else()
            message(WARNING "AVX-512 requested but not supported by compiler")
        endif()
    elseif(HD_CPU_OPTIMIZATION STREQUAL "AMD" OR (HD_CPU_OPTIMIZATION STREQUAL "AUTO" AND CPU_VENDOR MATCHES "AMD"))
        check_cxx_compiler_flag("/arch:AVX2" HAS_AVX2)
        if(HAS_AVX2)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:AVX2")
            set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /arch:AVX2")
            message(STATUS "Enabling AVX2 optimizations for AMD processor")
        endif() 
    elseif(HD_CPU_OPTIMIZATION STREQUAL "INTEL_AVX512")
        check_cxx_compiler_flag("/arch:AVX512" HAS_AVX512)
        if(HAS_AVX512)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:AVX512")
            set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /arch:AVX512")
            message(STATUS "Enabling AVX-512 optimizations for Intel processor (forced)")
            message(WARNING "AVX-512 may cause frequency throttling on some Intel CPUs. Use with caution.")
        else()
            message(WARNING "AVX-512 requested but not supported by compiler")
        endif()
    elseif(HD_CPU_OPTIMIZATION STREQUAL "INTEL" OR HD_CPU_OPTIMIZATION STREQUAL "AUTO" AND CPU_VENDOR MATCHES "Intel")
        check_cxx_compiler_flag("/arch:AVX2" HAS_AVX2)
        check_cxx_compiler_flag("/arch:AVX" HAS_AVX)
        
        if(HAS_AVX2)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:AVX2")
            set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /arch:AVX2")
            message(STATUS "Enabling AVX2 optimizations for Intel processor")
        elseif(HAS_AVX)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:AVX")
            set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /arch:AVX")
            message(STATUS "Enabling AVX optimizations for Intel processor")
        endif()
        
        # Additional optimizations for Intel parallel execution
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Qpar /Qpar-report:1")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /favor:INTEL64")
        message(STATUS "Enabling Intel-specific parallel optimizations")
    endif()
    
    # For ARM CPUs, check for NEON support (if building with ARM toolchain)
    if(HD_CPU_OPTIMIZATION STREQUAL "ARM" OR (HD_CPU_OPTIMIZATION STREQUAL "AUTO" AND (CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")))
        # ARM NEON is implicit in ARM64, explicit check for ARM32
        if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
            message(STATUS "ARM64 detected - NEON optimizations enabled by default")
        else()
            check_cxx_compiler_flag("/arch:NEON" HAS_NEON)
            if(HAS_NEON)
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:NEON")
                set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /arch:NEON")
                message(STATUS "Enabling NEON optimizations for ARM32 processor")
            endif()
        endif()
    endif()
else()
    # For GCC/Clang
    if(HD_CPU_OPTIMIZATION STREQUAL "AMD_AVX512")
        check_cxx_compiler_flag("-mavx512f" HAS_AVX512F)
        if(HAS_AVX512F)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx512f -mfma -mavx2")
            set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mavx512f -mavx2")
            message(STATUS "Enabling AVX-512 optimizations for AMD Zen 4+ processor (forced)")
            message(WARNING "AVX-512 support is limited to AMD Zen 4 (Ryzen 7000+) and later")
        else()
            message(WARNING "AVX-512 requested but not supported by compiler")
        endif()
    elseif(HD_CPU_OPTIMIZATION STREQUAL "AMD" OR (HD_CPU_OPTIMIZATION STREQUAL "AUTO" AND CPU_VENDOR MATCHES "AMD"))
        check_cxx_compiler_flag("-mavx" HAS_AVX)
        check_cxx_compiler_flag("-mavx2" HAS_AVX2)
        if(HAS_AVX2)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx2")
            set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mavx2")
            message(STATUS "Enabling AVX2 optimizations for AMD processor")
        elseif(HAS_AVX)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx")
            set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mavx")
            message(STATUS "Enabling AVX optimizations for AMD processor")
        endif()
    elseif(HD_CPU_OPTIMIZATION STREQUAL "INTEL_AVX512")
        check_cxx_compiler_flag("-mavx512f" HAS_AVX512F)
        if(HAS_AVX512F)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx512f -mfma -mavx2")
            set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mavx512f -mavx2")
            message(STATUS "Enabling AVX-512 optimizations for Intel processor (forced)")
            message(WARNING "AVX-512 may cause frequency throttling on some Intel CPUs. Use with caution.")
        else()
            message(WARNING "AVX-512 requested but not supported by compiler")
        endif()
    elseif(HD_CPU_OPTIMIZATION STREQUAL "INTEL" OR (HD_CPU_OPTIMIZATION STREQUAL "AUTO" AND CPU_VENDOR MATCHES "Intel"))
        check_cxx_compiler_flag("-mavx" HAS_AVX)
        check_cxx_compiler_flag("-mavx2" HAS_AVX2)
    
        if(HAS_AVX2)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx2")
            set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mavx2")
            message(STATUS "Enabling AVX2 optimizations for Intel processor")
        elseif(HAS_AVX)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx")
            set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mavx")
            message(STATUS "Enabling AVX optimizations for Intel processor")
        endif()
    endif()
    
    # For ARM CPUs, add NEON SIMD support
    if(HD_CPU_OPTIMIZATION STREQUAL "ARM" OR (HD_CPU_OPTIMIZATION STREQUAL "AUTO" AND (CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")))
        # ARM NEON optimizations
        check_cxx_compiler_flag("-mfpu=neon" HAS_NEON)
        if(HAS_NEON OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
            if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
                # ARM64 has NEON by default, add advanced SIMD optimizations
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a+simd")
                set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv8-a+simd")
                message(STATUS "Enabling Advanced SIMD (NEON) optimizations for ARM64 processor")
            else()
                # ARM32 explicit NEON
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon")
                set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfpu=neon")
                message(STATUS "Enabling NEON optimizations for ARM32 processor")
            endif()
        endif()
    endif()
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(OpenGL REQUIRED)

add_definitions(-D_USE_MATH_DEFINES)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include(cmake/platform.cmake)
include(cmake/imgui.cmake)
include(cmake/imguizmo.cmake)
include(cmake/implot.cmake)

add_subdirectory(${THIRDPARTY_DIRECTORY}/eigen)
set(EIGEN3_INCLUDE_DIR ${THIRDPARTY_DIRECTORY}/eigen)
MESSAGE(STATUS "Using bundled Eigen3 : ${EIGEN3_INCLUDE_DIR}")


option(BUILD_WITH_BUNDLED_FREEGLUT "Build with bundled FreeGlut" ON)
if (BUILD_WITH_BUNDLED_FREEGLUT)
    set(FREEGLUT_BUILD_DEMOS
            OFF
            CACHE BOOL "" FORCE)
    add_subdirectory(${THIRDPARTY_DIRECTORY}/freeglut)
    set(FREEGLUT_INCLUDE_DIR ${THIRDPARTY_DIRECTORY}/freeglut/include)
    set(FREEGLUT_LIBRARY freeglut)
    # Suppress unused variable warnings from FreeGlut
    if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(freeglut PRIVATE -Wno-unused-const-variable)
    endif()
    MESSAGE(STATUS "Using bundled FreeGlut")
else()
    find_package(GLUT REQUIRED)
    set(FREEGLUT_INCLUDE_DIR ${GLUT_INCLUDE_DIR})
    set(FREEGLUT_LIBRARY ${GLUT_glut_LIBRARY})
    MESSAGE(STATUS "FreeGlut include dir: ${FREEGLUT_INCLUDE_DIR}")
endif ()

option (BUILD_WITH_BUNDLED_GLEW "Build with bundled GLEW" ON)
if (BUILD_WITH_BUNDLED_GLEW)
    add_subdirectory(${THIRDPARTY_DIRECTORY}/glew-cmake)
    set(GLEW_INCLUDE_DIR ${THIRDPARTY_DIRECTORY}/glew-cmake/include)
    set(GLEW_LIBRARY libglew_static)

    MESSAGE(STATUS "Using bundled GLEW")
else()
    find_package(GLEW REQUIRED)
    set(GLEW_INCLUDE_DIR ${GLEW_INCLUDE_DIRS})
    set(GLEW_LIBRARY ${GLEW_LIBRARIES})
    MESSAGE(STATUS "GLEW include dir: ${GLEW_INCLUDE_DIR}, GLEW library: ${GLEW_LIBRARY}")
endif()

option(BUILD_WITH_BUNDLED_LIBLASZIP "Build with bundled Lib LASZIP" ON)
if (BUILD_WITH_BUNDLED_LIBLASZIP)
    add_subdirectory(${THIRDPARTY_DIRECTORY}/LASzip)
    set(LASZIP_INCLUDE_DIR ${THIRDPARTY_DIRECTORY})
    set(LASZIP_LIBRARY laszip)
    MESSAGE(STATUS "Using bundled LASzip")
    #suppress warnings from LASzip
#    target_compile_options(laszip PRIVATE
#            $<$<OR:$<C_COMPILER_ID:GNU>,$<C_COMPILER_ID:Clang>>:-Wno-cast-qual>
#            $<$<OR:$<C_COMPILER_ID:GNU>,$<C_COMPILER_ID:Clang>>:-Wno-maybe-uninitialized>
#            $<$<OR:$<C_COMPILER_ID:GNU>,$<C_COMPILER_ID:Clang>>:-Wno-stringop-truncation>
#            $<$<OR:$<C_COMPILER_ID:GNU>,$<C_COMPILER_ID:Clang>>:-Wno-format>
#            $<$<OR:$<C_COMPILER_ID:GNU>,$<C_COMPILER_ID:Clang>>:-Wno-format-security>
#    )

else()
    find_package(LASzip REQUIRED)
    set(LASZIP_INCLUDE_DIR ${LASZIP_INCLUDE_DIR})
    set(LASZIP_LIBRARY ${LASZIP_LIBRARY})
    set(PLATFORM_LASZIP_LIB ${LASZIP_LIBRARY})
    MESSAGE(STATUS "LASzip include dir: ${LASZIP_INCLUDE_DIR}, LASzip library: ${LASZIP_LIBRARY}")
endif ()

# Download or Search for OpenCV
include(${THIRDPARTY_DIRECTORY_BINARY}/OpenCV/CMakeLists.txt)
find_package(OpenCV REQUIRED)
MESSAGE(STATUS "OpenCV include dir: ${OpenCV_INCLUDE_DIRS}, OpenCV librarys: ${OpenCV_LIBS}")

# Option to build with bundled oneTBB
option(BUILD_WITH_BUNDLED_ONETBB "Build with bundled oneTBB" ON)
if (BUILD_WITH_BUNDLED_ONETBB)
    # Disable oneTBB components we don't need
    set(TBB_TEST OFF CACHE BOOL "" FORCE)
    set(TBB_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(TBB_STRICT OFF CACHE BOOL "" FORCE)
    set(TBBMALLOC_BUILD OFF CACHE BOOL "" FORCE)
    set(TBB_CPF OFF CACHE BOOL "" FORCE)

    add_subdirectory(${THIRDPARTY_DIRECTORY}/oneTBB)
    MESSAGE(STATUS "Using bundled oneTBB from: ${THIRDPARTY_DIRECTORY}/oneTBB")
else()
    # Find TBB from system
    find_package(TBB REQUIRED)
    MESSAGE(STATUS "Found system TBB")
endif()

# Add unordered_dense header-only library
add_subdirectory(${THIRDPARTY_DIRECTORY}/unordered_dense)
MESSAGE(STATUS "Using bundled unordered_dense from: ${THIRDPARTY_DIRECTORY}/unordered_dense")

# Add spdlog logging library
set(SPDLOG_BUILD_PIC ON CACHE BOOL "Build spdlog with position-independent code")
add_subdirectory(${THIRDPARTY_DIRECTORY}/spdlog)
MESSAGE(STATUS "Using bundled spdlog from: ${THIRDPARTY_DIRECTORY}/spdlog")

# Add UTL header-only utility library
add_subdirectory(${THIRDPARTY_DIRECTORY}/UTL)
MESSAGE(STATUS "Using bundled UTL from: ${THIRDPARTY_DIRECTORY}/UTL")

option(PYBIND "Enable pybind11 bindings" OFF)
if(PYBIND)
    message(STATUS "PYBIND is enabled: fetching pybind.")
    include(FetchContent)
    set(pybind11_SOURCE_DIR "${CMAKE_BINARY_DIR}/3rdparty/pybind11-src")
    set(pybind11_BINARY_DIR "${CMAKE_BINARY_DIR}/3rdparty/pybind11-build")
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11
        GIT_TAG        v2.12.1
        SOURCE_DIR     ${pybind11_SOURCE_DIR}
        BINARY_DIR     ${pybind11_BINARY_DIR}
    )
    FetchContent_GetProperties(pybind11)
    if(NOT pybind11_POPULATED)
        FetchContent_MakeAvailable(pybind11)
    endif()
    add_subdirectory(pybind)
else()
    message(STATUS "PYBIND is disabled.")
endif()


add_subdirectory(${THIRDPARTY_DIRECTORY}/Fusion/Fusion)

add_subdirectory(core)
add_subdirectory(core_hd_mapping)

set(CORE_LIBRARIES core core-hd-mapping)
set(GUI_LIBRARIES imgui imguizmo implot)

#add_subdirectory(apps/hd_mapper)
add_subdirectory(apps/lidar_odometry_step_1)    
add_subdirectory(apps/manual_color)
add_subdirectory(apps/console_tools)
add_subdirectory(apps/multi_session_registration)
add_subdirectory(apps/multi_view_tls_registration)
add_subdirectory(apps/split_multi_livox)
add_subdirectory(apps/precision_forestry_tools)
add_subdirectory(apps/mandeye_raw_data_viewer)
add_subdirectory(apps/compare_trajectories)
#add_subdirectory(apps/quick_start_demo)
add_subdirectory(apps/mandeye_mission_recorder_calibration)
add_subdirectory(apps/mandeye_single_session_viewer)
add_subdirectory(apps/livox_mid_360_intrinsic_calibration)
add_subdirectory(apps/single_session_manual_coloring)
add_subdirectory(apps/concatenate_multi_livox)
add_subdirectory(apps/laz_to_ply)
add_subdirectory(apps/laz_to_pcd)

 
# CPack configuration
set(CPACK_PACKAGE_NAME "hd_mapping")
set(CPACK_PACKAGE_VERSION "${HDMAPPING_VERSION_MAJOR}.${HDMAPPING_VERSION_MINOR}.${HDMAPPING_VERSION_PATCH}")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "HD Mapping debian package for Mandeye")
set(CPACK_PACKAGE_CONTACT "januszbedkowski@gmail.com")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Janusz Bedkowski <januszbedkowski@gmail.com>")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.28), libstdc++6 (>= 8.3.0), freeglut3-dev, libeigen3-dev, liblaszip-dev")
if(WIN32)
    set(CPACK_GENERATOR "ZIP")
else()
    set(CPACK_GENERATOR "DEB")
endif()

set(CPACK_DEBIAN_PACKAGE_SECTION "utils")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")

include(CPack)
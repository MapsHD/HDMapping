#pragma once
#include <nlohmann/json.hpp>

#include <HDMapping/HDMAPPING_ConfigureInfo.hpp>

//! Build configuration from HDMAPPING_ConfigureInfo.hpp, stored as "build_info" in session and project files.
inline nlohmann::json buildInfoJson()
{
    return {
        { "build_types", HDMAPPING_CONFIGURE_BUILD_TYPES },
        { "is_multiconfig", HDMAPPING_CONFIGURE_IS_MULTICONFIG },
        { "system", HDMAPPING_CONFIGURE_SYSTEM },
        { "system_version", HDMAPPING_CONFIGURE_SYSTEM_VERSION },
        { "cpu", HDMAPPING_CONFIGURE_CPU },
        { "compiler", HDMAPPING_CONFIGURE_COMPILER },
        { "compiler_version", HDMAPPING_CONFIGURE_COMPILER_VERSION },
        { "compiler_path", HDMAPPING_CONFIGURE_COMPILER_PATH },
        { "cmake_version", HDMAPPING_CONFIGURE_CMAKE_VERSION },
        { "generator", HDMAPPING_CONFIGURE_GENERATOR },
        { "project_name", HDMAPPING_CONFIGURE_PROJECT_NAME },
        { "project_version", HDMAPPING_CONFIGURE_PROJECT_VERSION },
        { "cxx_standard", HDMAPPING_CONFIGURE_CXX_STANDARD },
        { "timestamp", HDMAPPING_CONFIGURE_TIMESTAMP },
        { "git_hash", HDMAPPING_CONFIGURE_GIT_HASH },
        { "git_branch", HDMAPPING_CONFIGURE_GIT_BRANCH },
        { "git_is_dirty", HDMAPPING_CONFIGURE_GIT_IS_DIRTY },
        { "is_64_bit", HDMAPPING_CONFIGURE_IS_64_BIT },
        { "cxx_flags", HDMAPPING_CONFIGURE_CXX_FLAGS },
        { "cxx_flags_debug", HDMAPPING_CONFIGURE_CXX_FLAGS_DEBUG },
        { "cxx_flags_release", HDMAPPING_CONFIGURE_CXX_FLAGS_RELEASE },
        { "exe_linker_flags", HDMAPPING_CONFIGURE_EXE_LINKER_FLAGS },
        { "host_system", HDMAPPING_CONFIGURE_HOST_SYSTEM },
        { "host_system_version", HDMAPPING_CONFIGURE_HOST_SYSTEM_VERSION },
        { "host_cpu", HDMAPPING_CONFIGURE_HOST_CPU },
    };
}

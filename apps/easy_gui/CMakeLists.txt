cmake_minimum_required(VERSION 4.0.0)

project(easy_gui)

# Easy Forest HDMapping's own version: shown next to HDMapping's in the window title, README.txt, easy_gui.log and
# the LAZ header. Keep it equal to the easy-forest-v* release tag.
set(EASY_FOREST_VERSION "0.1.0")

# easy_gui_core -- everything except the window: pre-flight check, staging, the
# step 1 child process and its progress parsing, and the pipeline that ties
# them together. A separate static library so the doctest suite in tests/ can
# link it without raylib.
#
# Lidar odometry itself is NOT compiled in: it runs as a child process
# (lidar_odometry_step_1 <input> <params.toml> <output>), for crash isolation
# and a real cancel -- see src/child_process.h. Only two step 1 sources are
# compiled here, both by relative path like the other apps do:
#   - lidar_odometry_utils.cpp for MLvxCalib (the pre-flight check mirrors
#     load_data's calibration checks with step 1's own parser),
#   - toml_io.cpp to write the parameter file with step 1's own TomlIO.
# toml_io.cpp also reads the parameters back from the session for the export.
# The export (src/export.cpp) loads the step 1 session with core's Session in
# load_cache_mode (poses + trajectories exactly as step 2 sees them, no points)
# and streams the points itself, so core_no_gui is linked below.
# WITH_GUI=0 is PUBLIC because LidarOdometryParams changes layout under
# WITH_GUI (clear_color), so every TU that sees it must agree.
add_library(easy_gui_core STATIC
    src/batch.cpp src/batch.h
    src/child_process.cpp src/child_process.h
    src/export.cpp src/export.h
    src/forestry_preset.cpp src/forestry_preset.h
    src/las14_writer.cpp src/las14_writer.h
    src/pipeline.cpp src/pipeline.h
    src/preflight.cpp src/preflight.h
    src/progress_parser.cpp src/progress_parser.h
    src/settings.h
    src/sleep_blocker.cpp src/sleep_blocker.h
    src/staging.cpp src/staging.h
    src/version.h
    ../lidar_odometry_step_1/lidar_odometry_utils.cpp
    ../lidar_odometry_step_1/toml_io.cpp
)

target_compile_definitions(easy_gui_core
    PUBLIC WITH_GUI=0 EASY_FOREST_VERSION="${EASY_FOREST_VERSION}"
    PRIVATE HDMAPPING_BINARY_PREFIX="${HDMAPPING_BINARY_PREFIX}" UTL_PROFILER_DISABLE)

target_include_directories(easy_gui_core
    PUBLIC
        src
        ${REPOSITORY_DIRECTORY}/apps/lidar_odometry_step_1
        ${REPOSITORY_DIRECTORY}/core/include
        ${THIRDPARTY_DIRECTORY}
        ${EIGEN3_INCLUDE_DIR}
        ${THIRDPARTY_DIRECTORY}/json/include
        ${THIRDPARTY_DIRECTORY}/tomlplusplus/include
        ${LASZIP_INCLUDE_DIR}/LASzip/include
        ${THIRDPARTY_DIRECTORY}/observation_equations/codes
        ${THIRDPARTY_DIRECTORY}/vqf/vqf/cpp
        ${THIRDPARTY_DIRECTORY}/Fusion/Fusion)

# core_math provides the Core/ndt.h + hash_utils symbols lidar_odometry_utils.cpp
# needs (the same reason step 1's own tests link core_no_gui); TBB and laszip for
# its load_point_cloud()/decimate(), which the pre-flight check never calls.
target_link_libraries(easy_gui_core PUBLIC
    core_no_gui
    core_math
    vqf
    Fusion
    unordered_dense::unordered_dense
    spdlog::spdlog
    UTL::include
    ${PLATFORM_LASZIP_LIB}
    ${PLATFORM_MISCELLANEOUS_LIBS})

# Batch mode keeps the computer awake (src/sleep_blocker.cpp): IOKit power assertions on macOS.
if(APPLE)
    target_link_libraries(easy_gui_core PUBLIC "-framework IOKit" "-framework CoreFoundation")
endif()

if(MSVC)
    target_compile_definitions(easy_gui_core PUBLIC _USE_MATH_DEFINES NOMINMAX)
    target_compile_options(easy_gui_core PRIVATE /bigobj)
endif()

# The window. raylib + rlImGui like step 2; core_pfd for the native folder picker.
# imgui_stdlib (std::string InputText, for the batch table's output names) is compiled here: imgui_raylib does not
# include it, and it must come from the same vendored ImGui.
add_executable(easy_gui src/easy_gui_main.cpp ${THIRDPARTY_DIRECTORY}/imgui/misc/cpp/imgui_stdlib.cpp)

target_link_libraries(easy_gui PRIVATE
    easy_gui_core
    core_pfd
    raylib_widgets
    raylib
    imgui_raylib
    rlimgui)

# The step 1 program is part of Easy GUI at run time (it is started as a child
# process from the same directory), so building Easy GUI builds it too.
add_dependencies(easy_gui lidar_odometry_step_1)

if(WIN32)
    # A windowed program: no console window behind the GUI. main() stays the entry point (the WIN32 subsystem would
    # otherwise expect WinMain); --headless/--export-only print into the console they were started from
    # (attachToParentConsole). The manifest makes UTF-8 the process code page, see src/easy_gui.manifest.
    set_target_properties(easy_gui PROPERTIES WIN32_EXECUTABLE ON)
    if(MSVC)
        target_link_options(easy_gui PRIVATE /ENTRY:mainCRTStartup)
    endif()
    target_sources(easy_gui PRIVATE src/easy_gui.manifest)

    add_custom_command(
        TARGET easy_gui
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:easy_gui> $<TARGET_FILE_DIR:easy_gui>
        COMMAND_EXPAND_LISTS)
endif()

hdmapping_install_app(easy_gui)

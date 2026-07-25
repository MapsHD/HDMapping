include_guard()

# raylib + Dear ImGui + rlImGui, for the raylib-based step2 viewer
# (apps/multi_view_tls_registration_raylib). Mirrors the integration pattern
# established in the sibling mandeye-colors project.
#
# NOTE: cmake/imgui.cmake already defines a target named `imgui`, statically
# linked with the GLUT/OpenGL2 backend baked in for the legacy GLUT apps.
# That target cannot be reused here (rlImGui needs a plain ImGui core with no
# backend baked in), so this file builds a separate `imgui_raylib` target.

include(FetchContent)

set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES    OFF CACHE BOOL "" FORCE)
if(WIN32)
    # Makes GLFW's Win32 backend export NvOptimusEnablement/
    # AmdPowerXpressRequestHighPerformance (see glfw/src/win32_init.c), the
    # standard hint NVIDIA Optimus/AMD PowerXpress drivers read from an EXE's
    # exports to prefer the discrete GPU on hybrid-graphics laptops. Win32-only
    # (GLFW_USE_HYBRID_HPG is a no-op on X11/Wayland/macOS) -- Linux hybrid
    # offload is a runtime env-var choice instead (__NV_PRIME_RENDER_OFFLOAD=1
    # __GLX_VENDOR_LIBRARY_NAME=nvidia, or prime-run), not a build-time one.
    set(GLFW_USE_HYBRID_HPG ON CACHE BOOL "" FORCE)
endif()
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG        5.5
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(raylib)

# Built from the already-vendored 3rdparty/imgui tree (same source cmake/
# imgui.cmake's `imgui` target uses, just without the GLUT/OpenGL2 backend
# files -- rlImGui provides its own backend) rather than fetching a separate
# copy. This matters beyond avoiding a redundant fetch: 3rdparty/imgui is on
# the "docking" branch (has ImGui::DockSpace/DockBuilder*/
# SetNextWindowViewport, IMGUI_HAS_DOCK) -- gui.cpp's ShowMainDockSpace()
# uses those -- whereas a plain tagged release (e.g. v1.92.8) is built from
# "master", which doesn't include docking/viewports.
set(IMGUI_RAYLIB_DIR ${THIRDPARTY_DIRECTORY}/imgui)
add_library(imgui_raylib STATIC
    ${IMGUI_RAYLIB_DIR}/imgui.cpp
    ${IMGUI_RAYLIB_DIR}/imgui_draw.cpp
    ${IMGUI_RAYLIB_DIR}/imgui_tables.cpp
    ${IMGUI_RAYLIB_DIR}/imgui_widgets.cpp
    ${IMGUI_RAYLIB_DIR}/imgui_demo.cpp
)
target_include_directories(imgui_raylib PUBLIC ${IMGUI_RAYLIB_DIR})
# NOTE: deliberately NOT defining IMGUI_DISABLE_OBSOLETE_FUNCTIONS (unlike
# mandeye-colors) -- core's WITH_GUI=1 sources (manual_pose_graph_loop_closure.cpp
# etc, shared unchanged with the legacy GLUT app) call a few of ImGui's
# "obsolete" functions (e.g. SetWindowFontScale), which that flag compiles
# out entirely (declaration included), breaking the link against core.a.

# Pinned to a specific commit (rather than mandeye-colors' floating `main`)
# for build reproducibility.
FetchContent_Declare(
    rlimgui_src
    GIT_REPOSITORY https://github.com/raylib-extras/rlImGui.git
    GIT_TAG        ef129d1858373b6fe332c45f85a1dfc1421bebae
)
FetchContent_MakeAvailable(rlimgui_src)

add_library(rlimgui STATIC ${rlimgui_src_SOURCE_DIR}/rlImGui.cpp)
target_include_directories(rlimgui PUBLIC ${rlimgui_src_SOURCE_DIR})
target_link_libraries(rlimgui PUBLIC raylib imgui_raylib)

# ImGuizmo -- reuses the already-vendored 3rdparty/ImGuizmo sources (pure
# ImGui immediate-mode drawing via ImDrawList, no GL/GLUT calls of its own),
# but built as a SEPARATE static lib against imgui_raylib's headers rather
# than reusing the existing `imguizmo` target (cmake/imguizmo.cmake), which
# is compiled against the vendored 3rdparty/imgui (a different version) --
# mixing the two ImGui ABIs in one binary would be unsafe.
set(IMGUIZMO_RAYLIB_DIR ${THIRDPARTY_DIRECTORY}/ImGuizmo)
add_library(imguizmo_raylib STATIC ${IMGUIZMO_RAYLIB_DIR}/ImGuizmo.cpp)
target_include_directories(imguizmo_raylib PUBLIC ${IMGUIZMO_RAYLIB_DIR})
target_link_libraries(imguizmo_raylib PUBLIC imgui_raylib)

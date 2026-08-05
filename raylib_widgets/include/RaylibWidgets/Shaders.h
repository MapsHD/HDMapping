#pragma once

// Shared GLSL fragment-shader snippets, string-spliced into each app's own
// otherwise-distinct point-cloud shaders. Header-only (no .cpp, nothing to
// link) -- consumers just need this directory on their include path, which
// core_raylib gets via a plain target_include_directories (not a full
// raylib_widgets link, to avoid pulling imgui_raylib into core_raylib as a
// new transitive dependency for what's just a compile-time string constant).
namespace raylib_widgets {

// Standard "jet" colormap (blue -> cyan -> yellow -> red), t in [0,1]. Was
// byte-for-byte duplicated in core/src/raylib_render.cpp's ScanRenderer,
// apps/camera_lidar_calibration/Renderer.cpp's two point shaders, and
// apps/camera_lidar_trajectory_viewer/TrajectoryViewer.cpp's point shader.
// Splice into a fragment shader's source via string concatenation, e.g.:
//   static const std::string kMyFS = std::string(R"(#version 330
//   ...uniforms/varyings...
//   )") + raylib_widgets::kJetColormapGLSL + R"(
//   void main() { ... }
//   )";
inline constexpr const char* kJetColormapGLSL = R"(
vec3 jet(float t) {
    t = clamp(t, 0.0, 1.0);
    return clamp(vec3(1.5 - abs(4.0*t - 3.0),
                      1.5 - abs(4.0*t - 2.0),
                      1.5 - abs(4.0*t - 1.0)), 0.0, 1.0);
}
)";

} // namespace raylib_widgets

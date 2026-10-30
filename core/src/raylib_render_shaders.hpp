#pragma once

// GLSL source for ScanRenderer's point shader. Split out of raylib_render.cpp
// to keep the .cpp free of embedded shader text; included only there.
#include <RaylibWidgets/Shaders.h>

#include <string>

namespace raylib_render_shaders
{
    // vertexIntensity is per-point LAS/LAZ intensity, normalized to [0,1] per
    // scan at upload time (see ScanRenderer::rebuild). vertexPosition is
    // already world-space (rebuild() applies pc.m_pose before uploading), so
    // it doubles as the Elevation/Distance color modes' input with no extra
    // per-vertex data needed. colorMode selects between the flat per-scan
    // pointColor (0, the original app's only mode) and the ScanColorMode
    // gradients (1/2/3), matching the enum's Intensity/Elevation/Distance
    // ordering exactly (see ScanRenderer::draw()'s static_cast below).
    // model displaces the cached world-space points to the pose actually shown
    // (identity for a plain draw; a pose delta for drawCachedWithTransform() or
    // useInitialPose). It is applied before fragWorldPos, not folded into mvp, so
    // the intersection slab and the Elevation/Distance gradients follow the
    // displaced points instead of the pose they were cached at.
    inline constexpr const char* kPointVS = R"(
#version 330
in vec3 vertexPosition;
in float vertexIntensity;
uniform mat4 mvp;
uniform mat4 model;
uniform float pointSize;
out float fragIntensity;
out vec3 fragWorldPos;
void main()
{
    vec4 worldPos = model * vec4(vertexPosition, 1.0);
    gl_Position = mvp * worldPos;
    gl_PointSize = pointSize;
    fragIntensity = vertexIntensity;
    fragWorldPos = worldPos.xyz;
}
)";

    // jet() is shared (raylib_widgets/Shaders.h) with camera_lidar_calibration's
    // and camera_lidar_trajectory_viewer's point shaders -- was byte-for-byte
    // duplicated here.
    inline const std::string kPointFS = std::string(R"(
#version 330
uniform vec4 pointColor;
uniform int colorMode;
uniform int colormap;
uniform float elevMin;
uniform float elevMax;
uniform vec3 distCenter;
uniform float distMax;
// Intersection-slab gating: was PointCloud::render()'s (core/src/point_cloud.cpp,
// legacy immediate-mode GL) per-point xz/yz/xy_intersection check, ported here
// as a fragment discard instead of a CPU-side render_point bool. xzOn/yzOn/xyOn
// are 0/1 (bool uniforms aren't portable pre-4.x); intersectionWidth is the
// same half-width in world units on both sides of each cutting plane through
// the world origin. With all three off, every point draws (matches the
// original's "no intersection mode active -> always render" fallback).
uniform int xzOn;
uniform int yzOn;
uniform int xyOn;
uniform float intersectionWidth;
uniform int clipPlaneCount;
uniform vec4 clipPlanes[6];
in float fragIntensity;
in vec3 fragWorldPos;
out vec4 finalColor;
)") + raylib_widgets::kColormapsGLSL +
        R"(
void main()
{
    if (xzOn != 0 || yzOn != 0 || xyOn != 0)
    {
        bool inSlab = false;
        if (xzOn != 0 && abs(fragWorldPos.y) < intersectionWidth) inSlab = true;
        if (yzOn != 0 && abs(fragWorldPos.x) < intersectionWidth) inSlab = true;
        if (xyOn != 0 && abs(fragWorldPos.z) < intersectionWidth) inSlab = true;
        if (!inSlab) discard;
    }

    for (int i = 0; i < clipPlaneCount; ++i)
    {
        if (dot(clipPlanes[i].xyz, fragWorldPos) + clipPlanes[i].w < 0.0) discard;
    }

    if (colorMode == 1)
    {
        finalColor = vec4(applyColormap(colormap, fragIntensity), pointColor.a);
    }
    else if (colorMode == 2)
    {
        float range = max(elevMax - elevMin, 1e-6);
        finalColor = vec4(applyColormap(colormap, (fragWorldPos.z - elevMin) / range), pointColor.a);
    }
    else if (colorMode == 3)
    {
        float d = length(fragWorldPos - distCenter);
        finalColor = vec4(applyColormap(colormap, d / max(distMax, 1e-6)), pointColor.a);
    }
    else if (colorMode == 4)
    {
        // Flat color shaded by intensity. sqrt lifts the typically low-skewed intensity distribution;
        // the 0.35 floor keeps low-intensity points' hue visible.
        finalColor = vec4(pointColor.rgb * mix(0.35, 1.0, sqrt(fragIntensity)), pointColor.a);
    }
    else
    {
        finalColor = pointColor;
    }
}
)";
} // namespace raylib_render_shaders

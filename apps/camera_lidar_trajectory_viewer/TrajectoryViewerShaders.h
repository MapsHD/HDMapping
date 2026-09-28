#pragma once

// GLSL source for the trajectory viewer's point shader. Split out of
// TrajectoryViewer.cpp to keep the .cpp free of embedded shader text;
// included only there.
#include <RaylibWidgets/Shaders.h>

#include <string>

namespace trajectory_viewer_shaders
{
    // colorPacked: float bits = 0x00RRGGBB; colorMode: 0=jet intensity, 1=RGB, 2=camera id, 3=in ROI/mask,
    // 4=jet local depth (LiDAR range)
    inline constexpr const char* kVS = R"(
#version 330
// GLSL 330 has no 16-bit types: the integer attributes arrive widened to uint/int,
// and must be bound with glVertexAttribIPointer (rlSetVertexAttribute always
// converts to float).
layout(location = 0) in vec3  pos;
layout(location = 1) in float colorPacked;
layout(location = 2) in uint  lidarIntensity;  // GL_UNSIGNED_SHORT: intensity scaled from 0 to 65535
layout(location = 4) in int   lidarDepth;      // GL_SHORT: distance from the LiDAR's center in cm; < 0 = unknown
layout(location = 3) in int   colorCameraId;   // GL_INT: global image index that colored this point;
                                               // -1 = projects into no image, -2 = rejected by ROI/mask

uniform mat4  mvp;
uniform float pointSize;
uniform int   drawDecim;
uniform float minRange;  // m: hide points the LiDAR saw closer than this
uniform float maxRange;  // m: hide points the LiDAR saw farther than this; <= 0 = no limit
out float fragIntensity;
out float fragDepth;   // m; < 0 = unknown
out vec4 vertColor;
flat out float fragColorCameraId;
flat out float fragInRoi;
void main() {
    float range = float(lidarDepth) * 0.01;
    bool outOfRange = lidarDepth >= 0 && (range < minRange || (maxRange > 0.0 && range > maxRange));
    if ((drawDecim > 1 && (gl_VertexID % drawDecim) != 0) || outOfRange) {
        gl_Position  = vec4(2.0, 2.0, 2.0, 1.0);
        gl_PointSize = 0.0;
        return;
    }
    gl_Position  = mvp * vec4(pos, 1.0);
    gl_PointSize = pointSize;
    uint p = floatBitsToUint(colorPacked);
    float r = float((p >> 16) & 0xFFu) / 255.0;
    float g = float((p >>  8) & 0xFFu) / 255.0;
    float b = float( p        & 0xFFu) / 255.0;
    fragIntensity = float(lidarIntensity) / 65535.0;
    fragDepth = lidarDepth >= 0 ? range : -1.0;
    vertColor = vec4(r, g, b, 1.0);
    fragColorCameraId = float(colorCameraId);
    // kFS's in-ROI states: 1 = kept, 0 = rejected, -1 = projects into no image
    fragInRoi = colorCameraId >= 0 ? 1.0 : (colorCameraId == -1 ? -1.0 : 0.0);
}
)";

    inline const std::string kFS = std::string(R"(
#version 330
in float fragIntensity;
in float fragDepth;
in vec4 vertColor;
flat in float fragColorCameraId;
flat in float fragInRoi;
uniform int colorMode;
uniform int selectedCamera;   // -1 = show all, else keep only points from this image
uniform float depthColorMax;  // m: colorMode 4 maps 0..this onto the jet colormap
out vec4 finalColor;
)") + raylib_widgets::kJetColormapGLSL +
        R"(
vec3 hsv2rgb(vec3 c) {
    vec4 K = vec4(1.0, 2.0/3.0, 1.0/3.0, 3.0);
    vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);
    return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);
}
// deterministic, well-spread color per integer camera id
vec3 idColor(float idf) {
    float id = floor(idf + 0.5);
    float hue = fract(id * 0.61803398875); // golden ratio
    return hsv2rgb(vec3(hue, 0.85, 1.0));
}
void main() {
    if (selectedCamera >= 0)
    {
        if (selectedCamera != int(fragColorCameraId))
            discard; // do not draw
    }

    if (colorMode == 1)
    {
        if (fragColorCameraId < 0.0)
            discard; // not colored by any image — draw only colored points
        finalColor = vertColor;
    }
    else if (colorMode == 2)
    {
        if (fragColorCameraId < 0.0)
            discard; // not colored by any image — draw only colored points
        finalColor = vec4(idColor(fragColorCameraId), 1.0);
    }
    else if (colorMode == 3)
    {
        // ROI/mask membership: green = a pixel the ROI and the image mask both
        // keep, red = projects into an image but is rejected by one of them,
        // dim gray = projects into no image (spatial context).
        if (fragInRoi < 0.0)
            finalColor = vec4(0.28, 0.28, 0.28, 1.0);
        else
            finalColor = (fragInRoi > 0.5) ? vec4(0.15, 0.9, 0.2, 1.0)
                                           : vec4(0.9, 0.15, 0.15, 1.0);
    }
    else if (colorMode == 4)
    {
        // local depth: distance from the LiDAR when captured; gray = no timestamp
        if (fragDepth < 0.0)
            finalColor = vec4(0.28, 0.28, 0.28, 1.0);
        else
            finalColor = vec4(jet(clamp(fragDepth / max(depthColorMax, 0.01), 0.0, 1.0)), 1.0);
    }
    else finalColor = vec4(jet(fragIntensity), 1.0);
}
)";
} // namespace trajectory_viewer_shaders

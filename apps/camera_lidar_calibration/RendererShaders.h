#pragma once

// GLSL source for Renderer's point/projection shaders. Split out of
// Renderer.cpp to keep the .cpp free of embedded shader text; included only
// there.
#include <RaylibWidgets/Shaders.h>

#include <string>

namespace renderer_shaders
{
    // ── GPU point cloud shaders ──────────────────────────────────────────────────
    // Explicit attribute locations so one VAO works with both programs:
    // location 0 = position (raylib coords), location 1 = intensity.
    inline constexpr const char* kPointVS = R"(
#version 330
layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in float vertexIntensity;
uniform mat4 mvp;
uniform float pointSize;
uniform int drawDecim;     // draw only every Nth point; 1 = draw all
uniform mat4 lidarToCam;   // extrinsics (for RGB mode)
uniform vec4 K;            // fx, fy, cx, cy
uniform vec2 imgSize;
out vec3 fragPos;
out float fragIntensity;
out vec2 fragUV;
out float fragCamDepth;
void main() {
    if (drawDecim > 1 && (gl_VertexID % drawDecim) != 0) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        gl_PointSize = 0.0;
        return;
    }
    fragPos = vertexPosition;
    fragIntensity = vertexIntensity;
    gl_Position = mvp * vec4(vertexPosition, 1.0);
    gl_PointSize = pointSize;

    // Project into the camera image for RGB sampling (rectified → pinhole)
    vec3 lidar = vec3(vertexPosition.x, -vertexPosition.z, vertexPosition.y);
    vec3 pc = (lidarToCam * vec4(lidar, 1.0)).xyz;
    fragCamDepth = pc.z;
    vec2 uv = (K.xy * (pc.xy / max(pc.z, 1e-6)) + K.zw) / imgSize;
    fragUV = uv;
}
)";

    inline const std::string kPointFS = std::string(R"(
#version 330
in vec3 fragPos;
in float fragIntensity;
in vec2 fragUV;
in float fragCamDepth;
uniform int colorMode;      // 0 = distance, 1 = intensity, 2 = height, 3 = camera RGB
uniform vec2 heightRange;   // min/max of raylib Y (lidar Z)
uniform float maxDist;
uniform float opacity;
uniform sampler2D imageTex;
out vec4 finalColor;
)") + raylib_widgets::kJetColormapGLSL +
        R"(
void main() {
    if (colorMode == 3) {
        bool seen = fragCamDepth > 0.0
            && fragUV.x >= 0.0 && fragUV.x <= 1.0
            && fragUV.y >= 0.0 && fragUV.y <= 1.0;
        // points the camera cannot see stay gray — shows the camera FOV
        vec3 c = seen ? texture(imageTex, fragUV).rgb : vec3(0.25);
        finalColor = vec4(c, opacity);
        return;
    }
    float t;
    if (colorMode == 1)
        t = fragIntensity;
    else if (colorMode == 2)
        t = (fragPos.y - heightRange.x) / max(heightRange.y - heightRange.x, 1e-6);
    else
        t = length(fragPos) / max(maxDist, 1e-6);
    finalColor = vec4(jet(t), opacity);
}
)";

    // Projects lidar points directly onto the image plane. Position attribute is
    // in raylib coords, converted back to lidar frame here. With w = z_cam the
    // hardware clip rejects points behind the camera; optional rational+tangential
    // distortion handles non-rectified images (pass zeros when rectified).
    inline constexpr const char* kProjVS = R"(
#version 330
layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in float vertexIntensity;
uniform mat4 lidarToCam;   // extrinsics
uniform vec4 K;            // fx, fy, cx, cy
uniform vec2 imgSize;
uniform vec3 kRad1;        // k1 k2 k3
uniform vec3 kRad2;        // k4 k5 k6
uniform vec2 pTan;         // p1 p2
uniform float pointSize;
uniform int drawDecim;     // draw only every Nth point; 1 = draw all
out float fragDepth;
out float fragIntensity;
void main() {
    if (drawDecim > 1 && (gl_VertexID % drawDecim) != 0) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        gl_PointSize = 0.0;
        return;
    }
    // raylib coords -> lidar: x = rx, y = -rz, z = ry
    vec3 lidar = vec3(vertexPosition.x, -vertexPosition.z, vertexPosition.y);
    vec3 pc = (lidarToCam * vec4(lidar, 1.0)).xyz;
    fragDepth = pc.z;
    fragIntensity = vertexIntensity;

    vec2 n = pc.xy / max(pc.z, 1e-6);
    float r2 = dot(n, n);
    float radial = (1.0 + kRad1.x*r2 + kRad1.y*r2*r2 + kRad1.z*r2*r2*r2)
                 / (1.0 + kRad2.x*r2 + kRad2.y*r2*r2 + kRad2.z*r2*r2*r2);
    vec2 d = n * radial
           + vec2(2.0*pTan.x*n.x*n.y + pTan.y*(r2 + 2.0*n.x*n.x),
                  pTan.x*(r2 + 2.0*n.y*n.y) + 2.0*pTan.y*n.x*n.y);
    vec2 uv = K.xy * d + K.zw;                     // pixel coords

    // pixel -> clip space (y down, like raylib's render-texture ortho)
    gl_Position = vec4((2.0*uv.x/imgSize.x - 1.0) * pc.z,
                       -(2.0*uv.y/imgSize.y - 1.0) * pc.z,
                       0.0,
                       pc.z);
    gl_PointSize = pointSize;
}
)";

    inline const std::string kProjFS = std::string(R"(
#version 330
in float fragDepth;
in float fragIntensity;
uniform vec2 depthRange;
uniform float opacity;
uniform int colorMode;
out vec4 finalColor;
)") + raylib_widgets::kJetColormapGLSL +
        R"(
void main() {
    if (fragDepth < depthRange.x || fragDepth > depthRange.y) discard;
    float t = (colorMode == 1)
        ? fragIntensity
        : (fragDepth - depthRange.x) / max(depthRange.y - depthRange.x, 1e-6);
    finalColor = vec4(jet(t), opacity);
}
)";
} // namespace renderer_shaders

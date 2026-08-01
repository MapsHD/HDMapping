#include "Renderer.h"
#include "rlgl.h"
#include "raymath.h"
// glad function pointers are compiled into raylib; the header only declares them
#include "external/glad.h"
#include <cmath>
#include <algorithm>
#include <vector>

// ── Jet colormap ─────────────────────────────────────────────────────────────
Color jetColor(float t) {
    t = std::max(0.f, std::min(1.f, t));
    float r = std::max(0.f, std::min(1.f, 1.5f - std::abs(4.f*t - 3.f)));
    float g = std::max(0.f, std::min(1.f, 1.5f - std::abs(4.f*t - 2.f)));
    float b = std::max(0.f, std::min(1.f, 1.5f - std::abs(4.f*t - 1.f)));
    return Color{
        static_cast<unsigned char>(r * 255),
        static_cast<unsigned char>(g * 255),
        static_cast<unsigned char>(b * 255),
        255
    };
}

// ── OrbitCamera ───────────────────────────────────────────────────────────────
Camera3D OrbitCamera::toRaylib() const {
    float az  = azimuth   * (float)DEG2RAD;
    float el  = elevation * (float)DEG2RAD;
    Vector3 pos = {
        target.x + distance * std::cos(el) * std::sin(az),
        target.y + distance * std::sin(el),
        target.z + distance * std::cos(el) * std::cos(az)
    };
    Camera3D cam;
    cam.position   = pos;
    cam.target     = target;
    cam.up         = {0.f, 1.f, 0.f};
    cam.fovy       = 45.f;
    cam.projection = CAMERA_PERSPECTIVE;
    return cam;
}

void OrbitCamera::update(bool active) {
    if (!active) return;

    // Left-drag → orbit
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        Vector2 d = GetMouseDelta();
        azimuth   -= d.x * 0.4f;
        elevation += d.y * 0.4f;
        elevation  = std::max(-89.f, std::min(89.f, elevation));
    }
    // Right-drag → pan
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        Camera3D cam = toRaylib();
        Vector3 fwd   = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
        Vector3 right = Vector3Normalize(Vector3CrossProduct(fwd, cam.up));
        Vector3 up    = Vector3CrossProduct(right, fwd);
        Vector2 d     = GetMouseDelta();
        float   speed = distance * 0.002f;
        target = Vector3Add(target, Vector3Scale(right, -d.x * speed));
        target = Vector3Add(target, Vector3Scale(up,     d.y * speed));
    }
    // Scroll → zoom
    float wheel = GetMouseWheelMove();
    if (wheel != 0.f) {
        distance -= wheel * distance * 0.1f;
        distance  = std::max(0.5f, distance);
    }
}

// World-frame convention: E.rx/ry/rz = camera orientation in world (R_wc, ZYX Euler).
// E.tx/ty/tz = camera position in world.  p_cam = R_wc^T * (p_lidar - C).
static Matrix buildLidarToCamMatrix(const Extrinsics& E) {
    Eigen::Matrix3f R = eulerZYXtoMat3(E.rx, E.ry, E.rz);
    Eigen::Vector3f ti = -(R.transpose() * Eigen::Vector3f(E.tx, E.ty, E.tz));
    // Raylib Matrix struct fields: m0,m4,m8,m12 / m1,m5,m9,m13 / m2,m6,m10,m14 / m3,m7,m11,m15
    // We store R^T with translation ti (lidar→cam transform).
    return Matrix{
        R(0,0), R(1,0), R(2,0), ti(0),
        R(0,1), R(1,1), R(2,1), ti(1),
        R(0,2), R(1,2), R(2,2), ti(2),
        0.f,    0.f,    0.f,    1.f
    };
}

// ── Renderer ──────────────────────────────────────────────────────────────────
void Renderer::init(int imgW, int imgH) {
    if (imageTexValid)
        UnloadRenderTexture(imageTex);
    texW = imgW;
    texH = imgH;
    imageTex     = LoadRenderTexture(imgW, imgH);
    imageTexValid = true;
}

void Renderer::shutdown() {
    if (imageTexValid) {
        UnloadRenderTexture(imageTex);
        imageTexValid = false;
    }
    unloadCloudGPU();
    if (shaderValid) {
        UnloadShader(pointShader);
        shaderValid = false;
    }
}

// ── GPU point cloud shaders ──────────────────────────────────────────────────
// Explicit attribute locations so one VAO works with both programs:
// location 0 = position (raylib coords), location 1 = intensity.
static const char* kPointVS = R"(
#version 330
layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in float vertexIntensity;
uniform mat4 mvp;
uniform float pointSize;
uniform mat4 lidarToCam;   // extrinsics (for RGB mode)
uniform vec4 K;            // fx, fy, cx, cy
uniform vec2 imgSize;
out vec3 fragPos;
out float fragIntensity;
out vec2 fragUV;
out float fragCamDepth;
void main() {
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

static const char* kPointFS = R"(
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

vec3 jet(float t) {
    t = clamp(t, 0.0, 1.0);
    return clamp(vec3(1.5 - abs(4.0*t - 3.0),
                      1.5 - abs(4.0*t - 2.0),
                      1.5 - abs(4.0*t - 1.0)), 0.0, 1.0);
}

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
static const char* kProjVS = R"(
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
out float fragDepth;
out float fragIntensity;
void main() {
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

static const char* kProjFS = R"(
#version 330
in float fragDepth;
in float fragIntensity;
uniform vec2 depthRange;
uniform float opacity;
uniform int colorMode;
out vec4 finalColor;

vec3 jet(float t) {
    t = clamp(t, 0.0, 1.0);
    return clamp(vec3(1.5 - abs(4.0*t - 3.0),
                      1.5 - abs(4.0*t - 2.0),
                      1.5 - abs(4.0*t - 1.0)), 0.0, 1.0);
}

void main() {
    if (fragDepth < depthRange.x || fragDepth > depthRange.y) discard;
    float t = (colorMode == 1)
        ? fragIntensity
        : (fragDepth - depthRange.x) / max(depthRange.y - depthRange.x, 1e-6);
    finalColor = vec4(jet(t), opacity);
}
)";

void Renderer::initPointShader() {
    pointShader = LoadShaderFromMemory(kPointVS, kPointFS);
    shaderValid = pointShader.id > 0;
    if (!shaderValid) {
        TraceLog(LOG_ERROR, "Point cloud shader failed to compile");
    } else {
        locMVP         = rlGetLocationUniform(pointShader.id, "mvp");
        locPointSize   = rlGetLocationUniform(pointShader.id, "pointSize");
        locColorMode   = rlGetLocationUniform(pointShader.id, "colorMode");
        locHeightRange = rlGetLocationUniform(pointShader.id, "heightRange");
        locMaxDist     = rlGetLocationUniform(pointShader.id, "maxDist");
        locOpacity     = rlGetLocationUniform(pointShader.id, "opacity");
        locCamXform    = rlGetLocationUniform(pointShader.id, "lidarToCam");
        locCamK        = rlGetLocationUniform(pointShader.id, "K");
        locCamImgSize  = rlGetLocationUniform(pointShader.id, "imgSize");
        locCamTex      = rlGetLocationUniform(pointShader.id, "imageTex");
    }

    projShader = LoadShaderFromMemory(kProjVS, kProjFS);
    projShaderValid = projShader.id > 0;
    if (!projShaderValid) {
        TraceLog(LOG_ERROR, "Projection shader failed to compile");
    } else {
        locPrjXform      = rlGetLocationUniform(projShader.id, "lidarToCam");
        locPrjK          = rlGetLocationUniform(projShader.id, "K");
        locPrjImgSize    = rlGetLocationUniform(projShader.id, "imgSize");
        locPrjRad1       = rlGetLocationUniform(projShader.id, "kRad1");
        locPrjRad2       = rlGetLocationUniform(projShader.id, "kRad2");
        locPrjTan        = rlGetLocationUniform(projShader.id, "pTan");
        locPrjDepthRange = rlGetLocationUniform(projShader.id, "depthRange");
        locPrjOpacity    = rlGetLocationUniform(projShader.id, "opacity");
        locPrjPointSize  = rlGetLocationUniform(projShader.id, "pointSize");
        locPrjColorMode  = rlGetLocationUniform(projShader.id, "colorMode");
    }

    // Allow gl_PointSize from the vertex shader (core profile requires this)
    glEnable(GL_PROGRAM_POINT_SIZE);
}

void Renderer::uploadCloud(const PointCloud& cloud) {
    unloadCloudGPU();
    if (cloud.empty() || !shaderValid) return;

    // Interleaved: x, y, z (raylib coords), intensity
    std::vector<float> data;
    data.reserve(cloud.points.size() * 4);
    for (const auto& p : cloud.points) {
        // LiDAR coords → raylib: X=x, Y=z (up), Z=-y
        data.push_back(p.x);
        data.push_back(p.z);
        data.push_back(-p.y);
        data.push_back(p.intensity);
    }

    cloudVAO = rlLoadVertexArray();
    rlEnableVertexArray(cloudVAO);
    cloudVBO = rlLoadVertexBuffer(data.data(),
                                  static_cast<int>(data.size() * sizeof(float)),
                                  false);
    const int stride = 4 * sizeof(float);
    // locations fixed by layout() qualifiers in both shaders
    rlSetVertexAttribute(0, 3, RL_FLOAT, false, stride, 0);
    rlEnableVertexAttribute(0);
    rlSetVertexAttribute(1, 1, RL_FLOAT, false, stride, 3 * sizeof(float));
    rlEnableVertexAttribute(1);
    rlDisableVertexArray();

    cloudCount = static_cast<int>(cloud.points.size());
}

void Renderer::unloadCloudGPU() {
    if (cloudVAO) { rlUnloadVertexArray(cloudVAO);  cloudVAO = 0; }
    if (cloudVBO) { rlUnloadVertexBuffer(cloudVBO); cloudVBO = 0; }
    cloudCount = 0;
}

void Renderer::renderImageOverlay(const Texture2D& img, int imgW, int imgH,
                                  const Intrinsics& K, const Extrinsics& E,
                                  bool applyDistortion,
                                  const VisualizationParams& vp) {
    if (!imageTexValid) return;

    BeginTextureMode(imageTex);
    ClearBackground(BLACK);

    DrawTexturePro(img,
        Rectangle{0, 0, (float)imgW, (float)imgH},
        Rectangle{0, 0, (float)texW, (float)texH},
        Vector2{0, 0}, 0.f, WHITE);

    if (cloudCount > 0 && projShaderValid) {
        rlDrawRenderBatchActive();  // flush the image quad before raw GL draw

        Matrix xform = buildLidarToCamMatrix(E);

        float k[4]     = {K.fx, K.fy, K.cx, K.cy};
        float imgSize[2] = {(float)texW, (float)texH};
        float rad1[3]  = {0.f, 0.f, 0.f};
        float rad2[3]  = {0.f, 0.f, 0.f};
        float tan2[2]  = {0.f, 0.f};
        if (applyDistortion) {
            rad1[0] = K.k1; rad1[1] = K.k2; rad1[2] = K.k3;
            rad2[0] = K.k4; rad2[1] = K.k5; rad2[2] = K.k6;
            tan2[0] = K.p1; tan2[1] = K.p2;
        }
        float depthRange[2] = {vp.depthMin, vp.depthMax};

        rlEnableShader(projShader.id);
        rlSetUniformMatrix(locPrjXform, xform);
        rlSetUniform(locPrjK,          k,             RL_SHADER_UNIFORM_VEC4,  1);
        rlSetUniform(locPrjImgSize,    imgSize,       RL_SHADER_UNIFORM_VEC2,  1);
        rlSetUniform(locPrjRad1,       rad1,          RL_SHADER_UNIFORM_VEC3,  1);
        rlSetUniform(locPrjRad2,       rad2,          RL_SHADER_UNIFORM_VEC3,  1);
        rlSetUniform(locPrjTan,        tan2,          RL_SHADER_UNIFORM_VEC2,  1);
        rlSetUniform(locPrjDepthRange, depthRange,    RL_SHADER_UNIFORM_VEC2,  1);
        rlSetUniform(locPrjOpacity,    &vp.opacity,   RL_SHADER_UNIFORM_FLOAT, 1);
        rlSetUniform(locPrjPointSize,  &vp.pointSize, RL_SHADER_UNIFORM_FLOAT, 1);
        rlSetUniform(locPrjColorMode,  &vp.colorMode, RL_SHADER_UNIFORM_INT,   1);

        rlEnableVertexArray(cloudVAO);
        glDrawArrays(GL_POINTS, 0, cloudCount);
        rlDisableVertexArray();
        rlDisableShader();
    }

    EndTextureMode();
}

void Renderer::draw3DCloud(const PointCloud& cloud, const VisualizationParams& vp,
                           const Intrinsics& K, const Extrinsics& E,
                           const Texture2D& image, bool hasImage,
                           int imgW, int imgH) {
    if (cloudCount == 0 || !shaderValid) return;

    // Flush whatever raylib has batched so far (grid, lines) before raw GL draw
    rlDrawRenderBatchActive();

    Matrix mvp = MatrixMultiply(rlGetMatrixModelview(), rlGetMatrixProjection());

    // Furthest cloud corner from the LiDAR origin — normalizes distance coloring
    float mx = std::max(std::fabs(cloud.minX), std::fabs(cloud.maxX));
    float my = std::max(std::fabs(cloud.minY), std::fabs(cloud.maxY));
    float mz = std::max(std::fabs(cloud.minZ), std::fabs(cloud.maxZ));
    float maxDist = std::sqrt(mx*mx + my*my + mz*mz);

    // heightRange is in raylib Y, which carries lidar Z
    float heightRange[2] = {cloud.minZ, cloud.maxZ};

    int colorMode = vp.colorMode;
    if (colorMode == 3 && !hasImage)
        colorMode = 0;  // no image to sample — fall back to distance

    Matrix camXform = buildLidarToCamMatrix(E);
    float k[4]       = {K.fx, K.fy, K.cx, K.cy};
    float imgSize[2] = {(float)std::max(imgW, 1), (float)std::max(imgH, 1)};

    rlEnableShader(pointShader.id);
    rlSetUniformMatrix(locMVP, mvp);
    rlSetUniform(locPointSize,   &vp.pointSize, RL_SHADER_UNIFORM_FLOAT, 1);
    rlSetUniform(locColorMode,   &colorMode,    RL_SHADER_UNIFORM_INT,   1);
    rlSetUniform(locHeightRange, heightRange,   RL_SHADER_UNIFORM_VEC2,  1);
    rlSetUniform(locMaxDist,     &maxDist,      RL_SHADER_UNIFORM_FLOAT, 1);
    rlSetUniform(locOpacity,     &vp.opacity,   RL_SHADER_UNIFORM_FLOAT, 1);
    rlSetUniformMatrix(locCamXform, camXform);
    rlSetUniform(locCamK,       k,       RL_SHADER_UNIFORM_VEC4, 1);
    rlSetUniform(locCamImgSize, imgSize, RL_SHADER_UNIFORM_VEC2, 1);

    if (colorMode == 3) {
        rlActiveTextureSlot(0);
        rlEnableTexture(image.id);
        int slot = 0;
        rlSetUniform(locCamTex, &slot, RL_SHADER_UNIFORM_INT, 1);
    }

    rlEnableVertexArray(cloudVAO);
    glDrawArrays(GL_POINTS, 0, cloudCount);
    rlDisableVertexArray();
    rlDisableShader();
}

void Renderer::drawCameraFrustum(const Intrinsics& K, const Extrinsics& E,
                                 int imgW, int imgH, float scale) {
    // World-frame convention: R_wc = camera orientation in world, C = camera position in world
    Eigen::Matrix3f R = eulerZYXtoMat3(E.rx, E.ry, E.rz);

    // Camera position in LiDAR frame is directly (E.tx, E.ty, E.tz)
    Vector3 origin = {E.tx, E.tz, -E.ty}; // LiDAR→raylib

    // Four image corners in camera frame, at depth=scale
    float corners[4][2] = {
        {(0.f         - K.cx) / K.fx, (0.f         - K.cy) / K.fy},
        {(float(imgW) - K.cx) / K.fx, (0.f         - K.cy) / K.fy},
        {(float(imgW) - K.cx) / K.fx, (float(imgH) - K.cy) / K.fy},
        {(0.f         - K.cx) / K.fx, (float(imgH) - K.cy) / K.fy},
    };

    // Transform corners: p_lidar = R_wc * pc_cam + C
    Eigen::Vector3f C(E.tx, E.ty, E.tz);
    auto toWorld = [&](float xn, float yn) -> Vector3 {
        Eigen::Vector3f pl = R * Eigen::Vector3f(xn * scale, yn * scale, scale) + C;
        return {pl.x(), pl.z(), -pl.y()};
    };
    Vector3 w[4];
    for (int i = 0; i < 4; i++)
        w[i] = toWorld(corners[i][0], corners[i][1]);

    Color fc = YELLOW;
    DrawLine3D(origin, w[0], fc);
    DrawLine3D(origin, w[1], fc);
    DrawLine3D(origin, w[2], fc);
    DrawLine3D(origin, w[3], fc);
    DrawLine3D(w[0], w[1], fc);
    DrawLine3D(w[1], w[2], fc);
    DrawLine3D(w[2], w[3], fc);
    DrawLine3D(w[3], w[0], fc);
}

void Renderer::drawAxes(float len) {
    DrawLine3D({0,0,0}, {len, 0, 0},  RED);    // X
    DrawLine3D({0,0,0}, {0, len, 0},  GREEN);  // Y (= LiDAR Z = up)
    DrawLine3D({0,0,0}, {0, 0, -len}, BLUE);   // Z (= LiDAR Y)
}

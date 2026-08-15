#include "Renderer.h"
#include "raymath.h"
#include "rlgl.h"
// glad function pointers are compiled into raylib; the header only declares them
#include "RendererShaders.h"
#include "external/glad.h"
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

// ── Jet colormap ─────────────────────────────────────────────────────────────
Color jetColor(float t)
{
    t = std::max(0.f, std::min(1.f, t));
    float r = std::max(0.f, std::min(1.f, 1.5f - std::abs(4.f * t - 3.f)));
    float g = std::max(0.f, std::min(1.f, 1.5f - std::abs(4.f * t - 2.f)));
    float b = std::max(0.f, std::min(1.f, 1.5f - std::abs(4.f * t - 1.f)));
    return Color{ static_cast<unsigned char>(r * 255), static_cast<unsigned char>(g * 255), static_cast<unsigned char>(b * 255), 255 };
}

// OrbitCamera::toRaylib()/update() now live in raylib_widgets/src/OrbitCamera.cpp
// (Renderer.h aliases raylib_widgets::OrbitCamera) -- this used to duplicate
// those definitions, which became a duplicate-symbol link error once
// raylib_widgets/CMakeLists.txt started actually compiling that .cpp.

// World-frame convention: E.om/fi/ka = camera orientation in world (R_wc = Rx*Ry*Rz).
// E.tx/ty/tz = camera position in world.  p_cam = R_wc^T * (p_lidar - C).
static Matrix buildLidarToCamMatrix(const Extrinsics& E)
{
    Eigen::Matrix3f R = omFiKaToMat3(E.om, E.fi, E.ka);
    Eigen::Vector3f ti = -(R.transpose() * Eigen::Vector3f(E.tx, E.ty, E.tz));
    // Raylib Matrix struct fields: m0,m4,m8,m12 / m1,m5,m9,m13 / m2,m6,m10,m14 / m3,m7,m11,m15
    // We store R^T with translation ti (lidar→cam transform).
    return Matrix{
        R(0, 0), R(1, 0), R(2, 0), ti(0), R(0, 1), R(1, 1), R(2, 1), ti(1), R(0, 2), R(1, 2), R(2, 2), ti(2), 0.f, 0.f, 0.f, 1.f
    };
}

// ── Renderer ──────────────────────────────────────────────────────────────────
void Renderer::init(int imgW, int imgH)
{
    if (imageTexValid)
        UnloadRenderTexture(imageTex);
    texW = imgW;
    texH = imgH;
    imageTex = LoadRenderTexture(imgW, imgH);
    imageTexValid = true;
}

void Renderer::shutdown()
{
    if (imageTexValid)
    {
        UnloadRenderTexture(imageTex);
        imageTexValid = false;
    }
    unloadCloudGPU();
    if (shaderValid)
    {
        UnloadShader(pointShader);
        shaderValid = false;
    }
}

using renderer_shaders::kPointFS;
using renderer_shaders::kPointVS;
using renderer_shaders::kProjFS;
using renderer_shaders::kProjVS;

void Renderer::initPointShader()
{
    pointShader = LoadShaderFromMemory(kPointVS, kPointFS.c_str());
    shaderValid = pointShader.id > 0;
    if (!shaderValid)
    {
        TraceLog(LOG_ERROR, "Point cloud shader failed to compile");
    }
    else
    {
        locMVP = rlGetLocationUniform(pointShader.id, "mvp");
        locPointSize = rlGetLocationUniform(pointShader.id, "pointSize");
        locDecim = rlGetLocationUniform(pointShader.id, "drawDecim");
        locColorMode = rlGetLocationUniform(pointShader.id, "colorMode");
        locHeightRange = rlGetLocationUniform(pointShader.id, "heightRange");
        locMaxDist = rlGetLocationUniform(pointShader.id, "maxDist");
        locOpacity = rlGetLocationUniform(pointShader.id, "opacity");
        locCamXform = rlGetLocationUniform(pointShader.id, "lidarToCam");
        locCamK = rlGetLocationUniform(pointShader.id, "K");
        locCamImgSize = rlGetLocationUniform(pointShader.id, "imgSize");
        locCamTex = rlGetLocationUniform(pointShader.id, "imageTex");
    }

    projShader = LoadShaderFromMemory(kProjVS, kProjFS.c_str());
    projShaderValid = projShader.id > 0;
    if (!projShaderValid)
    {
        TraceLog(LOG_ERROR, "Projection shader failed to compile");
    }
    else
    {
        locPrjXform = rlGetLocationUniform(projShader.id, "lidarToCam");
        locPrjK = rlGetLocationUniform(projShader.id, "K");
        locPrjImgSize = rlGetLocationUniform(projShader.id, "imgSize");
        locPrjRad1 = rlGetLocationUniform(projShader.id, "kRad1");
        locPrjRad2 = rlGetLocationUniform(projShader.id, "kRad2");
        locPrjTan = rlGetLocationUniform(projShader.id, "pTan");
        locPrjDepthRange = rlGetLocationUniform(projShader.id, "depthRange");
        locPrjOpacity = rlGetLocationUniform(projShader.id, "opacity");
        locPrjPointSize = rlGetLocationUniform(projShader.id, "pointSize");
        locPrjColorMode = rlGetLocationUniform(projShader.id, "colorMode");
        locPrjDecim = rlGetLocationUniform(projShader.id, "drawDecim");
    }

    // Allow gl_PointSize from the vertex shader (core profile requires this)
    glEnable(GL_PROGRAM_POINT_SIZE);
}

void Renderer::uploadCloud(const PointCloud& cloud)
{
    unloadCloudGPU();
    if (cloud.empty() || !shaderValid)
        return;

    // Interleaved: x, y, z (raylib coords), intensity
    std::vector<float> data;
    data.reserve(cloud.points.size() * 4);
    for (const auto& p : cloud.points)
    {
        // LiDAR coords → raylib: X=x, Y=z (up), Z=-y
        data.push_back(p.x);
        data.push_back(p.z);
        data.push_back(-p.y);
        data.push_back(p.intensity);
    }

    cloudVAO = rlLoadVertexArray();
    rlEnableVertexArray(cloudVAO);
    cloudVBO = rlLoadVertexBuffer(data.data(), static_cast<int>(data.size() * sizeof(float)), false);
    const int stride = 4 * sizeof(float);
    // locations fixed by layout() qualifiers in both shaders
    rlSetVertexAttribute(0, 3, RL_FLOAT, false, stride, 0);
    rlEnableVertexAttribute(0);
    rlSetVertexAttribute(1, 1, RL_FLOAT, false, stride, 3 * sizeof(float));
    rlEnableVertexAttribute(1);
    rlDisableVertexArray();

    cloudCount = static_cast<int>(cloud.points.size());
}

void Renderer::unloadCloudGPU()
{
    if (cloudVAO)
    {
        rlUnloadVertexArray(cloudVAO);
        cloudVAO = 0;
    }
    if (cloudVBO)
    {
        rlUnloadVertexBuffer(cloudVBO);
        cloudVBO = 0;
    }
    cloudCount = 0;
}

void Renderer::renderImageOverlay(
    const Texture2D& img,
    int imgW,
    int imgH,
    const Intrinsics& K,
    const Extrinsics& E,
    bool applyDistortion,
    const VisualizationParams& vp,
    bool showOverlay)
{
    if (!imageTexValid)
        return;

    BeginTextureMode(imageTex);
    ClearBackground(BLACK);

    DrawTexturePro(
        img, Rectangle{ 0, 0, (float)imgW, (float)imgH }, Rectangle{ 0, 0, (float)texW, (float)texH }, Vector2{ 0, 0 }, 0.f, WHITE);

    if (showOverlay && cloudCount > 0 && projShaderValid)
    {
        rlDrawRenderBatchActive(); // flush the image quad before raw GL draw

        Matrix xform = buildLidarToCamMatrix(E);

        float k[4] = { K.fx, K.fy, K.cx, K.cy };
        float imgSize[2] = { (float)texW, (float)texH };
        float rad1[3] = { 0.f, 0.f, 0.f };
        float rad2[3] = { 0.f, 0.f, 0.f };
        float tan2[2] = { 0.f, 0.f };
        if (applyDistortion)
        {
            rad1[0] = K.k1;
            rad1[1] = K.k2;
            rad1[2] = K.k3;
            rad2[0] = K.k4;
            rad2[1] = K.k5;
            rad2[2] = K.k6;
            tan2[0] = K.p1;
            tan2[1] = K.p2;
        }
        float depthRange[2] = { vp.depthMin, vp.depthMax };

        rlEnableShader(projShader.id);
        rlSetUniformMatrix(locPrjXform, xform);
        rlSetUniform(locPrjK, k, RL_SHADER_UNIFORM_VEC4, 1);
        rlSetUniform(locPrjImgSize, imgSize, RL_SHADER_UNIFORM_VEC2, 1);
        rlSetUniform(locPrjRad1, rad1, RL_SHADER_UNIFORM_VEC3, 1);
        rlSetUniform(locPrjRad2, rad2, RL_SHADER_UNIFORM_VEC3, 1);
        rlSetUniform(locPrjTan, tan2, RL_SHADER_UNIFORM_VEC2, 1);
        rlSetUniform(locPrjDepthRange, depthRange, RL_SHADER_UNIFORM_VEC2, 1);
        rlSetUniform(locPrjOpacity, &vp.opacity, RL_SHADER_UNIFORM_FLOAT, 1);
        rlSetUniform(locPrjPointSize, &vp.pointSize, RL_SHADER_UNIFORM_FLOAT, 1);
        rlSetUniform(locPrjColorMode, &vp.colorMode, RL_SHADER_UNIFORM_INT, 1);
        rlSetUniform(locPrjDecim, &vp.drawDecim, RL_SHADER_UNIFORM_INT, 1);

        rlEnableVertexArray(cloudVAO);
        glDrawArrays(GL_POINTS, 0, cloudCount);
        rlDisableVertexArray();
        rlDisableShader();
    }

    EndTextureMode();
}

void Renderer::draw3DCloud(
    const PointCloud& cloud,
    const VisualizationParams& vp,
    const Intrinsics& K,
    const Extrinsics& E,
    const Texture2D& image,
    bool hasImage,
    int imgW,
    int imgH)
{
    if (cloudCount == 0 || !shaderValid)
        return;

    // Flush whatever raylib has batched so far (grid, lines) before raw GL draw
    rlDrawRenderBatchActive();

    Matrix mvp = MatrixMultiply(rlGetMatrixModelview(), rlGetMatrixProjection());

    // Furthest cloud corner from the LiDAR origin — normalizes distance coloring
    float mx = std::max(std::fabs(cloud.minX), std::fabs(cloud.maxX));
    float my = std::max(std::fabs(cloud.minY), std::fabs(cloud.maxY));
    float mz = std::max(std::fabs(cloud.minZ), std::fabs(cloud.maxZ));
    float maxDist = std::sqrt(mx * mx + my * my + mz * mz);

    // heightRange is in raylib Y, which carries lidar Z
    float heightRange[2] = { cloud.minZ, cloud.maxZ };

    int colorMode = vp.colorMode;
    if (colorMode == 3 && !hasImage)
        colorMode = 0; // no image to sample — fall back to distance

    Matrix camXform = buildLidarToCamMatrix(E);
    float k[4] = { K.fx, K.fy, K.cx, K.cy };
    float imgSize[2] = { (float)std::max(imgW, 1), (float)std::max(imgH, 1) };

    rlEnableShader(pointShader.id);
    rlSetUniformMatrix(locMVP, mvp);
    rlSetUniform(locPointSize, &vp.pointSize, RL_SHADER_UNIFORM_FLOAT, 1);
    rlSetUniform(locDecim, &vp.drawDecim, RL_SHADER_UNIFORM_INT, 1);
    rlSetUniform(locColorMode, &colorMode, RL_SHADER_UNIFORM_INT, 1);
    rlSetUniform(locHeightRange, heightRange, RL_SHADER_UNIFORM_VEC2, 1);
    rlSetUniform(locMaxDist, &maxDist, RL_SHADER_UNIFORM_FLOAT, 1);
    rlSetUniform(locOpacity, &vp.opacity, RL_SHADER_UNIFORM_FLOAT, 1);
    rlSetUniformMatrix(locCamXform, camXform);
    rlSetUniform(locCamK, k, RL_SHADER_UNIFORM_VEC4, 1);
    rlSetUniform(locCamImgSize, imgSize, RL_SHADER_UNIFORM_VEC2, 1);

    if (colorMode == 3)
    {
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

void Renderer::drawCameraFrustum(const Intrinsics& K, const Extrinsics& E, int imgW, int imgH, float scale)
{
    // World-frame convention: R_wc = camera orientation in world, C = camera position in world
    Eigen::Matrix3f R = omFiKaToMat3(E.om, E.fi, E.ka);

    // Camera position in LiDAR frame is directly (E.tx, E.ty, E.tz)
    Vector3 origin = { E.tx, E.tz, -E.ty }; // LiDAR→raylib

    // Four image corners in camera frame, at depth=scale
    float corners[4][2] = {
        { (0.f - K.cx) / K.fx, (0.f - K.cy) / K.fy },
        { (float(imgW) - K.cx) / K.fx, (0.f - K.cy) / K.fy },
        { (float(imgW) - K.cx) / K.fx, (float(imgH) - K.cy) / K.fy },
        { (0.f - K.cx) / K.fx, (float(imgH) - K.cy) / K.fy },
    };

    // Transform corners: p_lidar = R_wc * pc_cam + C
    Eigen::Vector3f C(E.tx, E.ty, E.tz);
    auto toWorld = [&](float xn, float yn) -> Vector3
    {
        Eigen::Vector3f pl = R * Eigen::Vector3f(xn * scale, yn * scale, scale) + C;
        return { pl.x(), pl.z(), -pl.y() };
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

void Renderer::drawAxes(float len)
{
    DrawLine3D({ 0, 0, 0 }, { len, 0, 0 }, RED); // X
    DrawLine3D({ 0, 0, 0 }, { 0, len, 0 }, GREEN); // Y (= LiDAR Z = up)
    DrawLine3D({ 0, 0, 0 }, { 0, 0, -len }, BLUE); // Z (= LiDAR Y)
}

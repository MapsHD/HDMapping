#include "Renderer.h"
#include "raymath.h"
#include "rlgl.h"
// glad function pointers are compiled into raylib; the header only declares them
#include "RendererShaders.h"
#include "external/glad.h"
#include <RaylibWidgets/PointBufferParts.h>
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
        locCamModel = rlGetLocationUniform(pointShader.id, "model");
        locCamXi = rlGetLocationUniform(pointShader.id, "xi");
        locCamRad1 = rlGetLocationUniform(pointShader.id, "kRad1");
        locCamRad2 = rlGetLocationUniform(pointShader.id, "kRad2");
        locCamTan = rlGetLocationUniform(pointShader.id, "pTan");
        locCamThetaMax = rlGetLocationUniform(pointShader.id, "thetaMax");
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
        locPrjModel = rlGetLocationUniform(projShader.id, "model");
        locPrjXi = rlGetLocationUniform(projShader.id, "xi");
        locPrjThetaMax = rlGetLocationUniform(projShader.id, "thetaMax");
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

    // locations fixed by layout() qualifiers in both shaders
    cloudCount = cloud.points.size();
    cloudParts = raylib_widgets::uploadPointBufferParts(data.data(), cloudCount, { 3, 1 });
}

void Renderer::unloadCloudGPU()
{
    raylib_widgets::unloadPointBufferParts(cloudParts);
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
    ImageOverlay overlay)
{
    if (!imageTexValid)
        return;

    BeginTextureMode(imageTex);
    ClearBackground(BLACK);

    const bool intensityImage = overlay == ImageOverlay::LidarIntensity;
    if (!intensityImage)
        DrawTexturePro(
            img, Rectangle{ 0, 0, (float)imgW, (float)imgH }, Rectangle{ 0, 0, (float)texW, (float)texH }, Vector2{ 0, 0 }, 0.f, WHITE);

    if (overlay != ImageOverlay::None && cloudCount > 0 && projShaderValid)
    {
        rlDrawRenderBatchActive(); // flush the image quad before raw GL draw

        Matrix xform = buildLidarToCamMatrix(E);

        float k[4] = { K.fx, K.fy, K.cx, K.cy };
        float imgSize[2] = { (float)texW, (float)texH };
        float rad1[3] = { 0.f, 0.f, 0.f };
        float rad2[3] = { 0.f, 0.f, 0.f };
        float tan2[2] = { 0.f, 0.f };
        // model/xi only take effect when applyDistortion is set too, same as
        // rad1/rad2/tan2 below -- applyDistortion==false means "treat as
        // already rectified" regardless of model (kept exactly as before
        // for Pinhole; Mei and Fisheye in practice always have
        // applyDistortion==true, since AppState::rebuildImageTexture never
        // rectifies them).
        int model = 0;
        float xiVal = 0.f;
        float thetaMax = 0.f;
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
            if (K.model == CameraModel::Mei)
            {
                model = 2;
                xiVal = K.xi;
            }
            else if (K.model == CameraModel::Fisheye)
            {
                model = 3;
                thetaMax = calib::fisheyeMaxTheta(K);
            }
        }
        float depthRange[2] = { vp.depthMin, vp.depthMax };
        // The intensity image is the points alone: opaque, coloured by
        // intensity whatever the colour mode, nearest point on top.
        const int colorMode = intensityImage ? 1 : vp.colorMode;
        const float opacity = intensityImage ? 1.f : vp.opacity;
        if (intensityImage)
            rlEnableDepthTest();

        rlEnableShader(projShader.id);
        rlSetUniformMatrix(locPrjXform, xform);
        rlSetUniform(locPrjK, k, RL_SHADER_UNIFORM_VEC4, 1);
        rlSetUniform(locPrjImgSize, imgSize, RL_SHADER_UNIFORM_VEC2, 1);
        rlSetUniform(locPrjRad1, rad1, RL_SHADER_UNIFORM_VEC3, 1);
        rlSetUniform(locPrjRad2, rad2, RL_SHADER_UNIFORM_VEC3, 1);
        rlSetUniform(locPrjTan, tan2, RL_SHADER_UNIFORM_VEC2, 1);
        rlSetUniform(locPrjModel, &model, RL_SHADER_UNIFORM_INT, 1);
        rlSetUniform(locPrjXi, &xiVal, RL_SHADER_UNIFORM_FLOAT, 1);
        rlSetUniform(locPrjThetaMax, &thetaMax, RL_SHADER_UNIFORM_FLOAT, 1);
        rlSetUniform(locPrjDepthRange, depthRange, RL_SHADER_UNIFORM_VEC2, 1);
        rlSetUniform(locPrjOpacity, &opacity, RL_SHADER_UNIFORM_FLOAT, 1);
        rlSetUniform(locPrjPointSize, &vp.pointSize, RL_SHADER_UNIFORM_FLOAT, 1);
        rlSetUniform(locPrjColorMode, &colorMode, RL_SHADER_UNIFORM_INT, 1);
        rlSetUniform(locPrjDecim, &vp.drawDecim, RL_SHADER_UNIFORM_INT, 1);

        raylib_widgets::drawPointBufferParts(cloudParts);
        rlDisableShader();
        if (intensityImage)
            rlDisableDepthTest();
    }

    EndTextureMode();
}

bool Renderer::isPointDrawn(size_t index, int drawDecim)
{
    // Mirrors the shaders' gl_VertexID % drawDecim, and gl_VertexID restarts
    // in every buffer part.
    return drawDecim <= 1 || (index % raylib_widgets::kMaxVerticesPerPart) % static_cast<size_t>(drawDecim) == 0;
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
    // Camera RGB sampling always applies Mei's or Fisheye's own distortion
    // (unlike the Pinhole path, their displayed image is never rectified --
    // see AppState::rebuildImageTexture and kPointVS's branches for them).
    int model = (K.model == CameraModel::Mei) ? 2 : (K.model == CameraModel::Fisheye) ? 3 : 0;
    float xi = K.xi;
    float rad1[3] = { K.k1, K.k2, K.k3 };
    float rad2[3] = { K.k4, K.k5, K.k6 };
    float tan2[2] = { K.p1, K.p2 };
    float thetaMax = (K.model == CameraModel::Fisheye) ? calib::fisheyeMaxTheta(K) : 0.f;

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
    rlSetUniform(locCamModel, &model, RL_SHADER_UNIFORM_INT, 1);
    rlSetUniform(locCamXi, &xi, RL_SHADER_UNIFORM_FLOAT, 1);
    rlSetUniform(locCamRad1, rad1, RL_SHADER_UNIFORM_VEC3, 1);
    rlSetUniform(locCamRad2, rad2, RL_SHADER_UNIFORM_VEC3, 1);
    rlSetUniform(locCamTan, tan2, RL_SHADER_UNIFORM_VEC2, 1);
    rlSetUniform(locCamThetaMax, &thetaMax, RL_SHADER_UNIFORM_FLOAT, 1);
    rlSetUniform(locCamImgSize, imgSize, RL_SHADER_UNIFORM_VEC2, 1);

    if (colorMode == 3)
    {
        rlActiveTextureSlot(0);
        rlEnableTexture(image.id);
        int slot = 0;
        rlSetUniform(locCamTex, &slot, RL_SHADER_UNIFORM_INT, 1);
    }

    raylib_widgets::drawPointBufferParts(cloudParts);
    rlDisableShader();
}

void Renderer::drawCameraFrustum(const Intrinsics& K, const Extrinsics& E, int imgW, int imgH, float scale)
{
    // World-frame convention: R_wc = camera orientation in world, C = camera position in world
    Eigen::Matrix3f R = omFiKaToMat3(E.om, E.fi, E.ka);

    // Camera position in LiDAR frame is directly (E.tx, E.ty, E.tz)
    Vector3 origin = { E.tx, E.tz, -E.ty }; // LiDAR→raylib

    if (K.model != CameraModel::Pinhole)
    {
        // A rectangular pyramid built from fx/fy/cx/cy/imgW/imgH (below)
        // assumes a narrow rectilinear FOV, which misrepresents a Mei or
        // equidistant fisheye's much wider one -- draw a position marker + camera
        // forward/right/up axis triad instead, same fallback
        // camera_lidar_trajectory_viewer uses for CameraModel::Mei.
        auto toWorld = [&](const Eigen::Vector3f& axis_c) -> Vector3
        {
            Eigen::Vector3f pl = R * (axis_c * scale * 0.5f) + Eigen::Vector3f(E.tx, E.ty, E.tz);
            return { pl.x(), pl.z(), -pl.y() };
        };
        DrawSphereWires(origin, scale * 0.08f, 8, 8, YELLOW);
        DrawLine3D(origin, toWorld(Eigen::Vector3f(0.f, 0.f, 1.f)), BLUE); // camera forward (Z)
        DrawLine3D(origin, toWorld(Eigen::Vector3f(1.f, 0.f, 0.f)), RED); // camera right (X)
        DrawLine3D(origin, toWorld(Eigen::Vector3f(0.f, -1.f, 0.f)), GREEN); // camera up (-Y: camera Y is down)
        return;
    }

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

// ── Sensor models ─────────────────────────────────────────────────────────────
// Dimensions in millimetres, read off the Livox Mid-360 and Insta360 X5 CAD
// models.
namespace
{
    //! Pushes a local frame posed at (R, t) in the LiDAR frame onto the
    //! matrix stack, with local lengths in millimetres. Pair with
    //! rlPopMatrix().
    //! @param R local axes in the LiDAR frame
    //! @param t local origin in the LiDAR frame, metres
    void pushPoseMm(const Eigen::Matrix3f& R, const Eigen::Vector3f& t)
    {
        Eigen::Matrix3f lidarToRaylib; // (x, y, z) -> (x, z, -y)
        lidarToRaylib << 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, -1.f, 0.f;
        const Eigen::Matrix3f M = lidarToRaylib * R * 0.001f;
        const Eigen::Vector3f T = lidarToRaylib * t;
        const Matrix m = { M(0, 0), M(0, 1), M(0, 2), T(0), M(1, 0), M(1, 1), M(1, 2), T(1),
                           M(2, 0), M(2, 1), M(2, 2), T(2), 0.f,     0.f,     0.f,     1.f };
        rlPushMatrix();
        rlMultMatrixf(MatrixToFloat(m));
    }

    void box(Vector3 lo, Vector3 hi, Color fill, Color edge)
    {
        const Vector3 centre = Vector3Scale(Vector3Add(lo, hi), 0.5f);
        const Vector3 size = Vector3Subtract(hi, lo);
        DrawCubeV(centre, size, fill);
        DrawCubeWiresV(centre, size, edge);
    }

    void circleZ(float z, float r, Color color)
    {
        constexpr int kSegments = 32;
        for (int i = 0; i < kSegments; ++i)
        {
            const float a0 = 2.f * PI * i / kSegments, a1 = 2.f * PI * (i + 1) / kSegments;
            DrawLine3D({ r * cosf(a0), r * sinf(a0), z }, { r * cosf(a1), r * sinf(a1), z }, color);
        }
    }

    //! Solid cone frustum along local z, outlined at both ends.
    void coneZ(float z0, float z1, float r0, float r1, Color fill, Color edge)
    {
        DrawCylinderEx({ 0.f, 0.f, z0 }, { 0.f, 0.f, z1 }, r0, r1, 32, fill);
        circleZ(z0, r0, edge);
        circleZ(z1, r1, edge);
    }

    //! In the optical frame: the X5's optical centre sits midway between its
    //! two lenses, with the body running down the image.
    void drawInsta360X5()
    {
        const Color body = { 70, 70, 75, 255 }, bodyEdge = { 150, 150, 155, 255 };
        const Color glass = { 25, 25, 30, 255 };

        box({ -23.f, -21.5f, -12.5f }, { 23.f, 103.f, 12.5f }, body, bodyEdge);
        for (const float side : { 1.f, -1.f })
        {
            coneZ(side * 12.5f, side * 15.f, 17.f, 17.f, body, side > 0.f ? YELLOW : bodyEdge);
            DrawSphereEx({ 0.f, 0.f, side * 8.f }, 12.5f, 12, 24, glass);
        }
    }

    //! In the optical frame: a Raspberry Pi format camera board (IMX519-style
    //! autofocus module). The sensor sits ~4.3 mm (the lens' focal length)
    //! behind the optical centre; the lens is 9.5 mm below the board's top
    //! edge, the ribbon connector on the back at the bottom.
    void drawPcbCamera()
    {
        const Color pcb = { 20, 110, 60, 255 }, pcbEdge = { 140, 200, 160, 255 };
        const Color module = { 35, 35, 40, 255 }, moduleEdge = { 130, 130, 135, 255 };
        const Color pad = { 200, 170, 80, 255 }, connector = { 60, 60, 60, 255 };
        const Color glass = { 15, 15, 20, 255 };
        constexpr float kSensorZ = -4.3f, kPcbT = 1.f;

        box({ -12.5f, -9.5f, kSensorZ - kPcbT }, { 12.5f, 14.5f, kSensorZ }, pcb, pcbEdge);
        for (const float x : { -10.5f, 10.5f })
            for (const float y : { -7.5f, 5.f })
                DrawCylinderEx({ x, y, kSensorZ - kPcbT - 0.05f }, { x, y, kSensorZ + 0.05f }, 1.2f, 1.2f, 12, pad);
        box({ -10.5f, 10.f, kSensorZ - kPcbT - 3.f }, { 10.5f, 14.f, kSensorZ - kPcbT }, connector, moduleEdge);
        box({ -4.25f, -4.25f, kSensorZ }, { 4.25f, 4.25f, 1.5f }, module, moduleEdge);
        coneZ(1.5f, 2.2f, 2.5f, 2.5f, glass, YELLOW);
    }
} // namespace

void Renderer::drawLidarModel()
{
    const Color body = { 150, 150, 155, 255 }, bodyEdge = { 70, 70, 75, 255 };
    const Color connector = { 95, 95, 100, 255 };
    const Color window = { 40, 40, 45, 255 }, windowEdge = { 120, 120, 125, 255 };

    pushPoseMm(Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero());
    box({ -32.5f, -32.5f, -26.f }, { 32.5f, 32.5f, 1.f }, body, bodyEdge);
    box({ -40.f, -10.f, -21.f }, { -32.5f, 10.f, -4.f }, connector, bodyEdge);
    coneZ(1.f, 4.f, 31.f, 31.f, body, bodyEdge);
    coneZ(4.f, 14.f, 25.5f, 25.5f, window, windowEdge);
    coneZ(14.f, 34.f, 25.5f, 15.f, window, windowEdge);
    rlPopMatrix();
}

void Renderer::drawCameraModel(const Extrinsics& E, CameraBody body)
{
    // Local frame = the optical frame (x right, y down, z forward), origin at
    // the optical centre.
    pushPoseMm(omFiKaToMat3(E.om, E.fi, E.ka), Eigen::Vector3f(E.tx, E.ty, E.tz));
    switch (body)
    {
    case CameraBody::Insta360X5:
        drawInsta360X5();
        break;
    case CameraBody::PcbCamera:
        drawPcbCamera();
        break;
    }
    rlPopMatrix();
}

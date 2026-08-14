#pragma once
#include "raylib.h"
#include <CalibCore/Camera.h>
#include <CalibCore/PointCloud.h>
#include <RaylibWidgets/OrbitCamera.h>
#include <vector>

using namespace calib;
using raylib_widgets::OrbitCamera;

struct VisualizationParams
{
    float pointSize = 2.f;
    float depthMin = 0.f;
    float depthMax = 50.f;
    float opacity = 1.f;
    int colorMode = 0; // 0=depth(jet), 1=intensity, 2=height(z), 3=Camera RGB
    // Draw only every Nth point (GPU-side, like camera_lidar_trajectory_viewer's
    // "Draw decimation") -- 1 = draw all points.
    int drawDecim = 1;
};

Color jetColor(float t); // t in [0,1]

class Renderer
{
public:
    RenderTexture2D imageTex = {}; // image + 2D projection overlay
    bool imageTexValid = false;

    void init(int imgW, int imgH);
    void shutdown();

    // Compile the GPU point shader. Requires an active OpenGL context.
    void initPointShader();

    // Upload point cloud to a GPU vertex buffer (interleaved x,y,z,intensity,
    // already in raylib coords). Replaces any previous buffer.
    void uploadCloud(const PointCloud& cloud);
    void unloadCloudGPU();

    // Render image + GPU-projected point overlay into imageTex.
    // If the displayed image is rectified, pass applyDistortion=false.
    // showOverlay=false draws just the plain image (e.g. while picking, so
    // projected points don't obscure the pixel the user is aiming for).
    void renderImageOverlay(
        const Texture2D& img,
        int imgW,
        int imgH,
        const Intrinsics& K,
        const Extrinsics& E,
        bool applyDistortion,
        const VisualizationParams& vp,
        bool showOverlay = true);

    // Draw 3D point cloud into current BeginMode3D context (GPU shader path).
    // For colorMode 3 (camera RGB) pass the displayed image texture and the
    // calibration; hasImage=false falls back to distance coloring.
    void draw3DCloud(
        const PointCloud& cloud,
        const VisualizationParams& vp,
        const Intrinsics& K,
        const Extrinsics& E,
        const Texture2D& image,
        bool hasImage,
        int imgW,
        int imgH);

    // Draw camera frustum as lines in current BeginMode3D context
    void drawCameraFrustum(const Intrinsics& K, const Extrinsics& E, int imgW, int imgH, float scale = 3.f);

    // Draw world axes at origin
    void drawAxes(float len = 2.f);

private:
    int texW = 0, texH = 0;

    // GPU point cloud (VAO shared by both shaders via fixed attrib locations)
    Shader pointShader = {};
    bool shaderValid = false;
    unsigned int cloudVAO = 0;
    unsigned int cloudVBO = 0;
    int cloudCount = 0;
    // 3D view shader uniforms
    int locMVP = -1, locColorMode = -1, locHeightRange = -1;
    int locMaxDist = -1, locOpacity = -1, locPointSize = -1, locDecim = -1;
    int locCamXform = -1, locCamK = -1, locCamImgSize = -1, locCamTex = -1;

    // 2D image-projection shader
    Shader projShader = {};
    bool projShaderValid = false;
    int locPrjXform = -1, locPrjK = -1, locPrjImgSize = -1;
    int locPrjRad1 = -1, locPrjRad2 = -1, locPrjTan = -1;
    int locPrjDepthRange = -1, locPrjOpacity = -1;
    int locPrjPointSize = -1, locPrjColorMode = -1, locPrjDecim = -1;
};

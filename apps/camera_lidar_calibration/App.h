#pragma once
#include "Renderer.h"
#include "UI.h"
#include "raylib.h"
#include <CalibCore/Camera.h>
#include <CalibCore/PointCloud.h>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

using namespace calib;

// One manually-picked correspondence between a LiDAR point and the image
// pixel it should project to. u/v are in the (undistorted) displayed-image
// pixel frame; px/py/pz are in the raw LiDAR frame.
struct CorrespondencePair
{
    float u = 0.f, v = 0.f;
    float px = 0.f, py = 0.f, pz = 0.f;
};

struct AppState
{
    // ── loaded data ──────────────────────────────────────────────────────────
    PointCloud cloud;
    cv::Mat originalImage; // RGB, as loaded from disk
    Texture2D imageTexture = {}; // displayed (rectified if possible)
    bool imageLoaded = false;
    bool imageRectified = false;
    bool intrinsicsLoaded = false; // from file (defaults are guesses)
    int imageW = 0, imageH = 0;
    std::string imagePath;
    std::vector<std::string> cloudPaths;
    // `cloud.points` converted to raylib world-space (x, z, -y), 1:1 index
    // match with `cloud.points` -- kept in sync by loadCloud()/addCloud() so
    // 3D point picking (raylib_widgets::pickNearestPoint) doesn't have to
    // re-convert the whole cloud every frame.
    std::vector<Vector3> cloudPointsRaylib;

    // ── calibration params ───────────────────────────────────────────────────
    Intrinsics intrinsics;
    Extrinsics extrinsics;

    // ── visualization ─────────────────────────────────────────────────────────
    VisualizationParams vizParams;
    bool showCompassRuler = true;

    // ── 3D camera ─────────────────────────────────────────────────────────────
    OrbitCamera orbit;

    // ── sub-systems ───────────────────────────────────────────────────────────
    Renderer renderer;
    UI ui;

    // ── misc ──────────────────────────────────────────────────────────────────
    std::string statusMsg;

    // ── LiDAR↔image correspondence picking ──────────────────────────────────
    // Hold Shift and click in either the Image View or the 3D View to set
    // the pending pick for that side (each click overwrites the previous
    // pending pick for that side, in either order); once both sides are set
    // the pair is completed automatically.
    std::vector<CorrespondencePair> pairs;
    CorrespondencePair pendingPair;
    bool pendingHasImage = false;
    bool pendingHasCloud = false;
    double lastSolveRmsPixels = -1.0; // < 0 = no solve run yet
    // Row selected in the Correspondences panel list (-1 = none); drawn as
    // a large cross in both the 3D view and the image view.
    int selectedPairIndex = -1;
    // When set, "Solve Extrinsics from Pairs" refines orientation only and
    // leaves tx/ty/tz exactly as they are -- for when the camera's position
    // relative to the LiDAR is already known (e.g. measured by hand) and
    // only orientation needs calibrating from the picked pairs. Also
    // disables the tx/ty/tz drag sliders in the Extrinsics panel so they
    // can't be nudged by accident while locked.
    bool lockTranslation = false;

    void setPendingImagePoint(float u, float v); // completes the pair if a cloud point is already pending
    void setPendingCloudPoint(float px, float py, float pz); // completes the pair if an image point is already pending
    void clearPending(); // discards the in-progress pending pick, if any
    void removePair(int index);
    bool solvePairs(); // solves extrinsics from `pairs`, updates statusMsg

    // ── operations ────────────────────────────────────────────────────────────
    void loadImage(const char* path);
    void loadCloud(const char* path); // clear + load
    void addCloud(const char* path); // merge into existing cloud
    void loadIntrinsics(const char* path);
    void loadCalibration(const char* path); // full JSON (intrinsics + extrinsics)
    void saveCalibration(const char* path);
    // (Re)build the displayed texture: undistorts with current intrinsics
    // when they were loaded from a file, otherwise shows the raw image.
    void rebuildImageTexture();
};

class App
{
public:
    // Call before run() to auto-load files after window init
    void preloadImage(const char* path)
    {
        pendingImage = path;
    }
    void preloadCloud(const char* path)
    {
        pendingClouds.push_back(path);
    }
    void preloadIntrinsics(const char* path)
    {
        pendingIntrinsics = path;
    }
    void preloadCalibration(const char* path)
    {
        pendingCalibration = path;
    }

    void run();

private:
    AppState state;
    std::string pendingImage;
    std::vector<std::string> pendingClouds;
    std::string pendingIntrinsics;
    std::string pendingCalibration;

    void update();
    void draw();
};

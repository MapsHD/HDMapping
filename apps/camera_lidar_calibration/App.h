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

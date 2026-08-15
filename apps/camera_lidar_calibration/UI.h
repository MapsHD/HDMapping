#pragma once
#include "Renderer.h"
#include <CalibCore/Camera.h>
#include <CalibCore/PointCloud.h>
#include <functional>
#include <string>

struct AppState;

class UI
{
public:
    // Called once per frame inside rlImGuiBegin()/rlImGuiEnd()
    void draw(AppState& state);

private:
    char imagePathBuf[512] = {};
    char cloudPathBuf[512] = {};
    char intrPathBuf[512] = {};
    char savePath[512] = "calibration.json";

    // 2D image view zoom -- pan is delegated entirely to ImGui's own child
    // scroll offset (see drawImageView), so there is no offX/offY to track
    // or clamp ourselves.
    float imgZoom = 1.f; // 1 = actual size (1 image px = 1 screen px)
    int lastImgW = 0, lastImgH = 0; // detects a newly-loaded image, to reset imgZoom

    void drawImageView(AppState& state);
    void panelMenuBar(AppState& state);
    void panelFiles(AppState& state);
    void panelIntrinsics(AppState& state);
    void panelExtrinsics(AppState& state);
    void panelVisualization(AppState& state);
    void panelCorrespondences(AppState& state);
    void panelStatus(const AppState& state);

    // File actions -- shared by the File menu items and their keyboard
    // shortcuts (handleShortcuts).
    void actionOpenImage(AppState& state);
    void actionOpenPointCloud(AppState& state);
    void actionAddPointCloud(AppState& state);
    void actionOpenIntrinsics(AppState& state);
    void actionOpenCalibration(AppState& state);
    void actionSaveCalibration(AppState& state);
    void handleShortcuts(AppState& state);
};

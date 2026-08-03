#pragma once
#include <CalibCore/Camera.h>
#include "Renderer.h"
#include <CalibCore/PointCloud.h>
#include <string>
#include <functional>

struct AppState;

class UI {
public:
    // Called once per frame inside rlImGuiBegin()/rlImGuiEnd()
    void draw(AppState& state);

private:
    char imagePathBuf[512] = {};
    char cloudPathBuf[512] = {};
    char intrPathBuf[512]  = {};
    char savePath[512]     = "calibration.json";

    // 2D image view pan/zoom state
    float zoom2D = 1.f;          // 1 = fit to window
    float offX = 0.f, offY = 0.f; // image coords of top-left visible pixel
    int   viewImgW = 0, viewImgH = 0;

    void drawImageView(AppState& state);
    void panelMenuBar(AppState& state);
    void panelFiles(AppState& state);
    void panelIntrinsics(AppState& state);
    void panelExtrinsics(AppState& state);
    void panelVisualization(AppState& state);
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

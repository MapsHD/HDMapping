#pragma once
#include "Renderer.h"
#include "UI.h"
#include "raylib.h"
#include <CalibCore/Camera.h>
#include <CalibCore/CameraCalibrationSolver.h>
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
    // Which camera `intrinsics` describe, when the file said so. Replaced
    // whenever the intrinsics are -- an untracked source (an OpenCV YAML, the
    // flat intrinsics JSON) clears it rather than leaving the previous
    // camera's serial attached to someone else's numbers.
    CameraIdentity cameraId;

    // Lidar id from status side car to laz
    std::string lidarId;

    // Resolution `intrinsics` are currently valid for: the calibration file's
    // own width/height, else whatever image was loaded at the time. 0 =
    // unknown. autoScaleIntrinsicsToImage() keeps this in sync, so it names
    // the size the *current* intrinsics apply to, not the file's original.
    int intrinsicsW = 0, intrinsicsH = 0;

    // ── visualization ─────────────────────────────────────────────────────────
    VisualizationParams vizParams;
    bool showCompassRuler = true;
    //! Simplified real-size sensor models in the 3D view: a Mid-360 at the
    //! LiDAR origin, cameraBody at the camera pose.
    bool showLidarModel = true;
    bool showCameraModel = true;
    CameraBody cameraBody = CameraBody::Insta360X5;

    //! Width of the Controls panel on the right, px; dragging its left edge changes it.
    float panelW = 340.f;

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
    //! Pair whose Remove button is hovered (-1 = none), marked in red in both views
    //! so it is clear which pair the click removes. Set by UI::draw each frame.
    int hoveredRemoveIndex = -1;
    // When set, "Solve Extrinsics from Pairs" refines orientation only and
    // leaves tx/ty/tz exactly as they are -- for when the camera's position
    // relative to the LiDAR is already known (e.g. measured by hand) and
    // only orientation needs calibrating from the picked pairs. Also
    // disables the tx/ty/tz drag sliders in the Extrinsics panel so they
    // can't be nudged by accident while locked.
    bool lockTranslation = false;
    //! Rotation counterpart of lockTranslation: the solve refines tx/ty/tz
    //! only and the om/fi/ka sliders are disabled.
    bool lockRotation = false;
    //! Soft constraints for "Solve Extrinsics from Pairs": each enabled one
    //! adds an observation equation pulling that parameter towards its
    //! target, e.g. to keep the camera height (tz) from drifting when the
    //! pairs barely constrain it. Indexed as calib::kPriorTx..kPriorKa.
    calib::ExtrinsicsPriors priors = defaultPriors();
    //! Mei intrinsics "Solve Extrinsics from Pairs" refines too; ignored for
    //! other models.
    calib::MeiIntrinsicsRefinement refineIntrinsics;
    //! @return the intrinsics the solve refines, e.g. "fx, fy, xi"; empty when none
    std::string refinedIntrinsicsLabel() const;

    /**
     * @brief Priors all disabled, with a sigma of 1 cm for tx/ty/tz and
     *        0.5 deg for om/fi/ka.
     * @return the default priors
     */
    static calib::ExtrinsicsPriors defaultPriors();

    void setPendingImagePoint(float u, float v); // completes the pair if a cloud point is already pending
    void setPendingCloudPoint(float px, float py, float pz); // completes the pair if an image point is already pending
    void clearPending(); // discards the in-progress pending pick, if any
    //! Makes the LiDAR point drawn at pixel (u, v) of the Image View's
    //! intensity image (Shift+Alt) the pending cloud point: the nearest one
    //! whose splat covers the pixel, else the closest within a few pixels.
    //! @return false, with the reason in statusMsg, when there is none
    bool pickLidarPointAtPixel(float u, float v);
    //! `intrinsics` as the displayed image needs them: distortion zeroed when
    //! the image is rectified.
    Intrinsics displayIntrinsics() const;
    void removePair(int index);
    bool solvePairs(); // solves extrinsics from `pairs`, updates statusMsg
    //! Writes `pairs` to a JSON file, with the size and rectification of the
    //! image their u/v were picked in.
    void savePairs(const char* path);
    //! Replaces `pairs` with those in a savePairs() file, rescaling u/v when the
    //! loaded image's size differs from the file's.
    void loadPairs(const char* path);
    //! @return whether `path` is a savePairs() file, i.e. a .json with a "pairs" array
    static bool isPairsFile(const char* path);
    //! File Ctrl+S saves the pairs to: --pairs, else the last pairs file loaded or
    //! saved; empty = ask for one.
    std::string pairsPath;

    // ── operations ────────────────────────────────────────────────────────────
    //! Points the 3D view at the LiDAR origin and the camera pose, close enough
    //! to see the sensor models, keeping the current viewing angle.
    void focusOnSensors();
    void loadImage(const char* path);
    //! Loads the image in `dir` taken closest to the middle of the loaded
    //! clouds' point times (calib::closestImage); leaves the image as it is,
    //! saying why in statusMsg, when the clouds or images carry no times.
    void loadImageClosestToCloudTime(const char* dir);
    void loadCloud(const char* path); // clear + load
    void addCloud(const char* path); // merge into existing cloud
    void loadIntrinsics(const char* path);
    void loadCalibration(const char* path); // full JSON (intrinsics + extrinsics)
    void saveCalibration(const char* path);
    // (Re)build the displayed texture: undistorts with current intrinsics
    // when they were loaded from a file, otherwise shows the raw image.
    void rebuildImageTexture();
    // Rescales `intrinsics` to the current imageW/imageH when intrinsicsW/H
    // names a different resolution, so a calibration and an image of
    // different sizes just work instead of silently mis-projecting. Called
    // after whichever of the two loads comes second. No-op (returns "") if
    // either size is unknown or they match. Caller owns rebuildImageTexture().
    std::string autoScaleIntrinsicsToImage();
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
    //! After the clouds load, replace the image with the one in `dir` taken
    //! closest to their time (AppState::loadImageClosestToCloudTime).
    void preloadImageClosestToClouds(const char* dir)
    {
        pendingImageDirByTime = dir;
    }
    //! Picked pairs file: loaded after everything else, as from a drop, when it
    //! exists; Ctrl+S saves the pairs to it either way.
    void preloadPairs(const char* path)
    {
        pendingPairs = path;
    }

    void run();

private:
    AppState state;
    std::string pendingImage;
    std::vector<std::string> pendingClouds;
    std::string pendingIntrinsics;
    std::string pendingCalibration;
    std::string pendingImageDirByTime;
    std::string pendingPairs;

    void update();
    void draw();
};

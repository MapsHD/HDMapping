#include "App.h"
#include "imgui.h"
#include "raymath.h"
#include "rlImGui.h"
#include <CalibCore/CameraCalibrationSolver.h>
#include <HDMapping/Version.hpp>
#include <RaylibWidgets/CompassRuler.h>
#include <RaylibWidgets/PointPicking.h>
#include <RaylibWidgets/WindowFit.h>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

// ── AppState::rebuildImageTexture ─────────────────────────────────────────────
void AppState::rebuildImageTexture()
{
    if (originalImage.empty())
        return;

    cv::Mat display = originalImage;
    imageRectified = false;

    if (intrinsicsLoaded)
    {
        cv::Mat K = (cv::Mat_<double>(3, 3) << intrinsics.fx, 0, intrinsics.cx, 0, intrinsics.fy, intrinsics.cy, 0, 0, 1);
        // OpenCV distCoeffs order: k1 k2 p1 p2 k3 k4 k5 k6 (rational model)
        cv::Mat D =
            (cv::Mat_<double>(1, 8) << intrinsics.k1,
             intrinsics.k2,
             intrinsics.p1,
             intrinsics.p2,
             intrinsics.k3,
             intrinsics.k4,
             intrinsics.k5,
             intrinsics.k6);

        cv::Mat map1, map2;
        cv::initUndistortRectifyMap(K, D, cv::Mat(), K, originalImage.size(), CV_16SC2, map1, map2);
        cv::Mat rectified;
        cv::remap(originalImage, rectified, map1, map2, cv::INTER_LINEAR);
        display = rectified;
        imageRectified = true;
    }

    if (imageLoaded)
        UnloadTexture(imageTexture);
    Image rimg = {};
    rimg.data = display.data;
    rimg.width = display.cols;
    rimg.height = display.rows;
    rimg.mipmaps = 1;
    rimg.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8;
    imageTexture = LoadTextureFromImage(rimg); // copies pixels to GPU
    imageLoaded = true;
}

// ── AppState correspondence picking ───────────────────────────────────────────
void AppState::setPendingImagePoint(float u, float v)
{
    pendingPair.u = u;
    pendingPair.v = v;
    pendingHasImage = true;

    if (pendingHasCloud)
    {
        pairs.push_back(pendingPair);
        pendingPair = CorrespondencePair{};
        pendingHasImage = pendingHasCloud = false;
        statusMsg = "Pair " + std::to_string(pairs.size()) + " added";
    }
    else
    {
        statusMsg = "Image point set -- Shift+click the matching point in the 3D View";
    }
}

void AppState::setPendingCloudPoint(float px, float py, float pz)
{
    pendingPair.px = px;
    pendingPair.py = py;
    pendingPair.pz = pz;
    pendingHasCloud = true;

    if (pendingHasImage)
    {
        pairs.push_back(pendingPair);
        pendingPair = CorrespondencePair{};
        pendingHasImage = pendingHasCloud = false;
        statusMsg = "Pair " + std::to_string(pairs.size()) + " added";
    }
    else
    {
        statusMsg = "3D point set -- Shift+click the matching pixel in the Image View";
    }
}

void AppState::clearPending()
{
    pendingPair = CorrespondencePair{};
    pendingHasImage = pendingHasCloud = false;
    statusMsg = "Pending pick cleared";
}

void AppState::removePair(int index)
{
    if (index < 0 || index >= static_cast<int>(pairs.size()))
        return;
    pairs.erase(pairs.begin() + index);
    if (selectedPairIndex == index)
        selectedPairIndex = -1;
    else if (selectedPairIndex > index)
        selectedPairIndex--;
}

bool AppState::solvePairs()
{
    if (pairs.size() < 3)
    {
        statusMsg = "Need at least 3 pairs to solve";
        return false;
    }

    std::vector<calib::PointPixelCorrespondence> corr;
    corr.reserve(pairs.size());
    for (const auto& p : pairs)
    {
        calib::PointPixelCorrespondence c;
        c.p = Eigen::Vector3d(p.px, p.py, p.pz);
        c.u = p.u;
        c.v = p.v;
        corr.push_back(c);
    }

    double rms = -1.0;
    bool ok = calib::solveExtrinsicsFromCorrespondences(corr, intrinsics, extrinsics, &rms, lockTranslation);
    if (!ok)
    {
        statusMsg = "Solve failed (degenerate correspondences)";
        return false;
    }

    lastSolveRmsPixels = rms;
    statusMsg =
        lockTranslation ? "Solved rotation (translation locked): RMS reprojection error " : "Solved extrinsics: RMS reprojection error ";
    statusMsg += std::to_string(rms) + " px";
    if (!imageRectified && intrinsicsLoaded == false)
        statusMsg += " (no intrinsics loaded -- distortion assumed zero)";
    return true;
}

// ── AppState::loadImage ───────────────────────────────────────────────────────
void AppState::loadImage(const char* path)
{
    cv::Mat bgr = cv::imread(path, cv::IMREAD_COLOR);
    if (bgr.empty())
    {
        statusMsg = std::string("Failed to load image: ") + path;
        return;
    }
    cv::cvtColor(bgr, originalImage, cv::COLOR_BGR2RGB);
    imageW = originalImage.cols;
    imageH = originalImage.rows;
    imagePath = path;
    rebuildImageTexture();
    renderer.init(imageW, imageH);
    statusMsg = imageRectified ? "Image loaded and rectified" : "Image loaded (raw)";
}

// ── AppState::loadCloud ───────────────────────────────────────────────────────
static void centerOrbitOnCloud(AppState& s)
{
    s.orbit.target = { (s.cloud.minX + s.cloud.maxX) * 0.5f, (s.cloud.minZ + s.cloud.maxZ) * 0.5f, -(s.cloud.minY + s.cloud.maxY) * 0.5f };
    float span = std::max({ s.cloud.maxX - s.cloud.minX, s.cloud.maxY - s.cloud.minY, s.cloud.maxZ - s.cloud.minZ });
    s.orbit.distance = span * 0.8f;
}

// Keeps AppState::cloudPointsRaylib (used by 3D point picking) 1:1 in sync
// with AppState::cloud.points -- same LiDAR (x,y,z) -> raylib (x,z,-y)
// convention as Renderer::uploadCloud.
static void rebuildCloudPointsRaylib(AppState& s)
{
    s.cloudPointsRaylib.clear();
    s.cloudPointsRaylib.reserve(s.cloud.points.size());
    for (const auto& p : s.cloud.points)
        s.cloudPointsRaylib.push_back(Vector3{ p.x, p.z, -p.y });
}

void AppState::loadCloud(const char* path)
{
    if (!cloud.load(path))
    {
        statusMsg = std::string("Failed to load cloud: ") + path;
        return;
    }
    cloudPaths = { path };
    renderer.uploadCloud(cloud);
    rebuildCloudPointsRaylib(*this);
    centerOrbitOnCloud(*this);
    statusMsg = "";
}

void AppState::addCloud(const char* path)
{
    PointCloud extra;
    if (!extra.load(path))
    {
        statusMsg = std::string("Failed to load: ") + path;
        return;
    }
    // merge bounding box
    if (cloud.empty())
    {
        cloud = std::move(extra);
    }
    else
    {
        cloud.points.insert(cloud.points.end(), extra.points.begin(), extra.points.end());
        cloud.minX = std::min(cloud.minX, extra.minX);
        cloud.maxX = std::max(cloud.maxX, extra.maxX);
        cloud.minY = std::min(cloud.minY, extra.minY);
        cloud.maxY = std::max(cloud.maxY, extra.maxY);
        cloud.minZ = std::min(cloud.minZ, extra.minZ);
        cloud.maxZ = std::max(cloud.maxZ, extra.maxZ);
    }
    cloudPaths.push_back(path);
    renderer.uploadCloud(cloud);
    rebuildCloudPointsRaylib(*this);
    centerOrbitOnCloud(*this);
    statusMsg = "";
}

// ── OpenCV YAML intrinsics parser ─────────────────────────────────────────────
// Handles both styles produced by OpenCV/ROS calibration tools:
//   data: [a, b, c]        (flow, may span lines until ']')
//   data:                  (block)
//   - a
//   - b
// Distortion order is OpenCV distCoeffs: k1 k2 p1 p2 k3 [k4 k5 k6 ...]
static void extractNumbers(const std::string& s, std::vector<double>& out)
{
    const char* p = s.c_str();
    while (*p)
    {
        if ((*p >= '0' && *p <= '9') || *p == '-' || *p == '+' || *p == '.')
        {
            char* end = nullptr;
            double v = std::strtod(p, &end);
            if (end != p)
            {
                out.push_back(v);
                p = end;
                continue;
            }
        }
        ++p;
    }
}

static bool parseOpenCVYaml(const char* path, Intrinsics& K, int& imgW, int& imgH, std::string& err)
{
    std::ifstream f(path);
    if (!f)
    {
        err = "cannot open file";
        return false;
    }

    std::vector<double> camMat, dist;
    std::vector<double>* active = nullptr; // section whose data we collect
    std::vector<double>* collecting = nullptr;
    bool inFlow = false;

    std::string line;
    while (std::getline(f, line))
    {
        std::string trimmed = line;
        trimmed.erase(0, trimmed.find_first_not_of(" \t"));

        if (inFlow)
        {
            extractNumbers(trimmed, *collecting);
            if (trimmed.find(']') != std::string::npos)
            {
                inFlow = false;
                collecting = nullptr;
            }
            continue;
        }

        bool topLevel = !line.empty() && line[0] != ' ' && line[0] != '\t' && line[0] != '-';
        if (topLevel)
        {
            collecting = nullptr;
            if (trimmed.rfind("camera_matrix:", 0) == 0)
                active = &camMat;
            else if (trimmed.rfind("distortion_coefficients:", 0) == 0)
                active = &dist;
            else
            {
                active = nullptr;
                if (trimmed.rfind("image_width:", 0) == 0)
                    imgW = std::atoi(trimmed.c_str() + 12);
                else if (trimmed.rfind("image_height:", 0) == 0)
                    imgH = std::atoi(trimmed.c_str() + 13);
            }
            continue;
        }

        if (active && trimmed.rfind("data:", 0) == 0)
        {
            auto bracket = trimmed.find('[');
            if (bracket != std::string::npos)
            {
                extractNumbers(trimmed.substr(bracket), *active);
                if (trimmed.find(']') == std::string::npos)
                {
                    collecting = active;
                    inFlow = true;
                }
            }
            else
            {
                collecting = active; // block list follows
            }
            continue;
        }

        if (collecting)
        {
            if (trimmed.rfind("- ", 0) == 0 || trimmed.rfind("-", 0) == 0)
                extractNumbers(trimmed, *collecting);
            else
                collecting = nullptr; // rows:/cols: or another key ends the list
        }
    }

    if (camMat.size() < 9)
    {
        err = "camera_matrix needs 9 values";
        return false;
    }

    // Row-major 3x3: [fx 0 cx; 0 fy cy; 0 0 1]
    K.fx = static_cast<float>(camMat[0]);
    K.cx = static_cast<float>(camMat[2]);
    K.fy = static_cast<float>(camMat[4]);
    K.cy = static_cast<float>(camMat[5]);

    auto d = [&](size_t i)
    {
        return i < dist.size() ? static_cast<float>(dist[i]) : 0.f;
    };
    K.k1 = d(0);
    K.k2 = d(1);
    K.p1 = d(2);
    K.p2 = d(3);
    K.k3 = d(4);
    K.k4 = d(5);
    K.k5 = d(6);
    K.k6 = d(7);
    return true;
}

// ── AppState::loadIntrinsics ──────────────────────────────────────────────────
void AppState::loadIntrinsics(const char* path)
{
    std::string p = path;
    auto dot = p.rfind('.');
    std::string ext = (dot != std::string::npos) ? p.substr(dot + 1) : "";
    for (auto& c : ext)
        c = static_cast<char>(tolower(c));

    if (ext == "yml" || ext == "yaml")
    {
        int imgW = 0, imgH = 0;
        std::string err;
        if (!parseOpenCVYaml(path, intrinsics, imgW, imgH, err))
        {
            statusMsg = std::string("YAML error: ") + err + " (" + path + ")";
            return;
        }
        intrinsicsLoaded = true;
        rebuildImageTexture(); // re-rectify with the new coefficients
        statusMsg = "Intrinsics loaded";
        if (imageRectified)
            statusMsg += ", image rectified";
        if (imgW > 0)
        {
            statusMsg += " (camera " + std::to_string(imgW) + "x" + std::to_string(imgH) + ")";
            if (imageLoaded && (imgW != imageW || imgH != imageH))
                statusMsg += " WARNING: image is " + std::to_string(imageW) + "x" + std::to_string(imageH);
        }
        return;
    }

    std::ifstream f(path);
    if (!f)
    {
        statusMsg = std::string("Cannot open: ") + path;
        return;
    }
    nlohmann::json j;
    f >> j;
    intrinsics.fx = j.value("fx", intrinsics.fx);
    intrinsics.fy = j.value("fy", intrinsics.fy);
    intrinsics.cx = j.value("cx", intrinsics.cx);
    intrinsics.cy = j.value("cy", intrinsics.cy);
    intrinsics.k1 = j.value("k1", 0.f);
    intrinsics.k2 = j.value("k2", 0.f);
    intrinsics.k3 = j.value("k3", 0.f);
    intrinsics.k4 = j.value("k4", 0.f);
    intrinsics.k5 = j.value("k5", 0.f);
    intrinsics.k6 = j.value("k6", 0.f);
    intrinsics.p1 = j.value("p1", 0.f);
    intrinsics.p2 = j.value("p2", 0.f);
    intrinsicsLoaded = true;
    rebuildImageTexture();
    statusMsg = "Intrinsics loaded.";
}

// ── AppState::loadCalibration ─────────────────────────────────────────────────
void AppState::loadCalibration(const char* path)
{
    std::ifstream f(path);
    if (!f)
    {
        statusMsg = std::string("Cannot open: ") + path;
        return;
    }
    nlohmann::json j;
    try
    {
        f >> j;
    } catch (...)
    {
        statusMsg = std::string("JSON parse error: ") + path;
        return;
    }

    bool gotIntrinsics = false, gotExtrinsics = false;

    if (j.contains("intrinsics"))
    {
        auto& ji = j["intrinsics"];
        intrinsics.fx = ji.value("fx", intrinsics.fx);
        intrinsics.fy = ji.value("fy", intrinsics.fy);
        intrinsics.cx = ji.value("cx", intrinsics.cx);
        intrinsics.cy = ji.value("cy", intrinsics.cy);
        intrinsics.k1 = ji.value("k1", 0.f);
        intrinsics.k2 = ji.value("k2", 0.f);
        intrinsics.k3 = ji.value("k3", 0.f);
        intrinsics.k4 = ji.value("k4", 0.f);
        intrinsics.k5 = ji.value("k5", 0.f);
        intrinsics.k6 = ji.value("k6", 0.f);
        intrinsics.p1 = ji.value("p1", 0.f);
        intrinsics.p2 = ji.value("p2", 0.f);
        intrinsicsLoaded = true;
        gotIntrinsics = true;
    }

    if (j.contains("extrinsics"))
    {
        auto& je = j["extrinsics"];
        // camera_position_in_world_xyz: [tx, ty, tz]
        if (je.contains("camera_position_in_world_xyz") && je["camera_position_in_world_xyz"].size() >= 3)
        {
            auto& pos = je["camera_position_in_world_xyz"];
            extrinsics.tx = pos[0].get<float>();
            extrinsics.ty = pos[1].get<float>();
            extrinsics.tz = pos[2].get<float>();
        }
        // camera_rotation_matrix_in_world: 3x3 rows of R_wc. The file's only
        // rotation representation (convention-independent, portable) --
        // decomposed into calib::Extrinsics' own om/fi/ka for internal
        // use (UI sliders, solver initial guess).
        if (je.contains("camera_rotation_matrix_in_world") && je["camera_rotation_matrix_in_world"].size() >= 3)
        {
            auto& m = je["camera_rotation_matrix_in_world"];
            Eigen::Matrix3f R;
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    R(r, c) = m[r][c].get<float>();
            calib::omFiKaFromMat3(R, extrinsics.om, extrinsics.fi, extrinsics.ka);
        }
        gotExtrinsics = true;
    }

    if (!gotIntrinsics && !gotExtrinsics)
    {
        statusMsg = std::string("No intrinsics/extrinsics found in: ") + path;
        return;
    }

    if (gotIntrinsics)
        rebuildImageTexture();

    statusMsg = "Loaded";
    if (gotIntrinsics)
        statusMsg += " intrinsics";
    if (gotIntrinsics && gotExtrinsics)
        statusMsg += " +";
    if (gotExtrinsics)
        statusMsg += " extrinsics";
    statusMsg += std::string(" from ") + path;
}

// ── AppState::saveCalibration ─────────────────────────────────────────────────
void AppState::saveCalibration(const char* path)
{
    // World-frame convention: R = R_wc (camera orientation in world, om/fi/ka)
    // C = camera position in world.  T_lidar_to_cam = [R_wc^T | -R_wc^T*C]
    Eigen::Matrix3f R = omFiKaToMat3(extrinsics.om, extrinsics.fi, extrinsics.ka);
    Eigen::Vector3f C(extrinsics.tx, extrinsics.ty, extrinsics.tz);
    Eigen::Vector3f ti = -(R.transpose() * C); // translation of T_lidar_to_camera

    nlohmann::json j;
    j["intrinsics"] = { { "fx", intrinsics.fx }, { "fy", intrinsics.fy }, { "cx", intrinsics.cx }, { "cy", intrinsics.cy },
                        { "k1", intrinsics.k1 }, { "k2", intrinsics.k2 }, { "k3", intrinsics.k3 }, { "k4", intrinsics.k4 },
                        { "k5", intrinsics.k5 }, { "k6", intrinsics.k6 }, { "p1", intrinsics.p1 }, { "p2", intrinsics.p2 } };
    // Rotation is stored as a matrix only -- convention-independent (no
    // Euler/Tait-Bryan angle order or units to document/misread) and
    // directly portable to any external tool. camera_rotation_matrix_in_world
    // is the source of truth on load; see AppState::loadCalibration.
    j["extrinsics"]["camera_position_in_world_xyz"] = { C.x(), C.y(), C.z() };
    j["extrinsics"]["camera_rotation_matrix_in_world"] = { { R(0, 0), R(0, 1), R(0, 2) },
                                                           { R(1, 0), R(1, 1), R(1, 2) },
                                                           { R(2, 0), R(2, 1), R(2, 2) } };
    j["extrinsics"]["T_lidar_to_camera_4x4"] = {
        { R(0, 0), R(1, 0), R(2, 0), ti(0) }, { R(0, 1), R(1, 1), R(2, 1), ti(1) }, { R(0, 2), R(1, 2), R(2, 2), ti(2) }, { 0, 0, 0, 1 }
    };

    std::ofstream f(path);
    if (!f)
    {
        statusMsg = std::string("Cannot write: ") + path;
        return;
    }
    f << j.dump(4);
    statusMsg = std::string("Saved to ") + path;
    printf("Calibration saved to %s\n", path);
}

// ── App::run ──────────────────────────────────────────────────────────────────
void App::run()
{
    const int W = 1400, H = 900;
    // MSAA was costing real frame time once the 3D view grew to fill the
    // whole window (was previously confined to half-height) -- point-cloud
    // rendering is fill-rate-bound, and 4x MSAA multiplies per-pixel
    // shading/blending cost across that now-doubled area. Point clouds are
    // rendered as flat-shaded point sprites (no polygon edges to smooth),
    // so the visual benefit was marginal anyway.
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(W, H, ("LiDAR-Camera Calibration " HDMAPPING_VERSION_STRING));
    raylib_widgets::fitWindowToScreen();
    // The 340px-wide side panel is fixed-width; below this the 3D/image
    // views and the panel start overlapping instead of scrolling.
    SetWindowMinSize(900, 500);
    SetTargetFPS(60);

    rlImGuiSetup(true); // dark theme

    state.renderer.initPointShader();

    // Load files passed as command-line arguments
    if (!pendingImage.empty())
        state.loadImage(pendingImage.c_str());
    for (size_t i = 0; i < pendingClouds.size(); ++i)
        (i == 0 ? state.loadCloud(pendingClouds[i].c_str()) : state.addCloud(pendingClouds[i].c_str()));
    if (!pendingIntrinsics.empty())
        state.loadIntrinsics(pendingIntrinsics.c_str());
    if (!pendingCalibration.empty())
        state.loadCalibration(pendingCalibration.c_str());

    while (!WindowShouldClose())
    {
        update();
        draw();
    }

    if (state.imageLoaded)
        UnloadTexture(state.imageTexture);
    state.renderer.shutdown();
    rlImGuiShutdown();
    CloseWindow();
}

// Shift (either side) is the modifier that dedicates the mouse to point
// picking in both the Image View (UI::drawImageView) and the 3D View
// (updateAndDrawPicking3D below) -- mirrors the CTRL-to-pick convention
// core/src/control_points.cpp already uses elsewhere in this codebase.
static bool shiftHeld()
{
    return IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
}

// ── App::update ───────────────────────────────────────────────────────────────
void App::update()
{
    bool imguiWantMouse = ImGui::GetIO().WantCaptureMouse;
    // Freeze orbiting while Shift is held so LMB is dedicated to picking
    // instead of being ambiguous between "start an orbit drag" and "click
    // to pick".
    bool allowOrbit = !imguiWantMouse && !shiftHeld();
    state.orbit.update(allowOrbit);
}

// ── 3D point picking + correspondence markers ───────────────────────────────
// Draws small markers for already-picked pairs and, while Shift is held over
// the 3D view, a live "nearest point under cursor" candidate -- the 3D-side
// counterpart to the image view's zoom loupe: a precision aid the user sees
// before committing a click. Must be called inside BeginMode3D/EndMode3D.
//
// Index labels for the markers are 2D text (DrawText), which must NOT be
// called while inside BeginMode3D/EndMode3D -- while a 3D projection matrix
// is active, DrawText's screen-pixel quads would get warped through it
// instead of rendering flat. So this only *collects* (screenPos, label)
// pairs here (GetWorldToScreen just needs the camera, not an active 3D
// pass); the caller draws them with plain DrawText() after EndMode3D().
static void updateAndDrawPicking3D(
    AppState& state, const Camera3D& cam3d, Rectangle viewport3D, std::vector<std::pair<Vector2, int>>& outLabels)
{
    Vector3 camFwd = Vector3Normalize(Vector3Subtract(cam3d.target, cam3d.position));

    for (size_t i = 0; i < state.pairs.size(); i++)
    {
        const auto& p = state.pairs[i];
        Vector3 wp = { p.px, p.pz, -p.py };
        DrawSphere(wp, 0.05f, YELLOW);
        DrawLine3D(Vector3{ wp.x - 0.1f, wp.y, wp.z }, Vector3{ wp.x + 0.1f, wp.y, wp.z }, YELLOW);
        DrawLine3D(Vector3{ wp.x, wp.y - 0.1f, wp.z }, Vector3{ wp.x, wp.y + 0.1f, wp.z }, YELLOW);

        if (Vector3DotProduct(Vector3Subtract(wp, cam3d.position), camFwd) > 0.f)
        {
            Vector2 s = GetWorldToScreen(wp, cam3d);
            if (s.x >= viewport3D.x && s.x <= viewport3D.x + viewport3D.width && s.y >= viewport3D.y &&
                s.y <= viewport3D.y + viewport3D.height)
                outLabels.emplace_back(s, (int)i);
        }
    }

    // Large 3-axis cross for the pair selected in the Correspondences
    // panel, so it's easy to spot in the (usually much busier) 3D view.
    if (state.selectedPairIndex >= 0 && state.selectedPairIndex < (int)state.pairs.size())
    {
        const auto& p = state.pairs[state.selectedPairIndex];
        Vector3 wp = { p.px, p.pz, -p.py };
        const float armLen = 0.5f;
        DrawLine3D(Vector3{ wp.x - armLen, wp.y, wp.z }, Vector3{ wp.x + armLen, wp.y, wp.z }, MAGENTA);
        DrawLine3D(Vector3{ wp.x, wp.y - armLen, wp.z }, Vector3{ wp.x, wp.y + armLen, wp.z }, MAGENTA);
        DrawLine3D(Vector3{ wp.x, wp.y, wp.z - armLen }, Vector3{ wp.x, wp.y, wp.z + armLen }, MAGENTA);
    }

    if (state.pendingHasCloud)
    {
        Vector3 wp = { state.pendingPair.px, state.pendingPair.pz, -state.pendingPair.py };
        DrawSphere(wp, 0.07f, ORANGE);
    }

    if (!shiftHeld() || ImGui::GetIO().WantCaptureMouse)
        return;

    Vector2 mouse = GetMousePosition();
    if (mouse.x < viewport3D.x || mouse.x > viewport3D.x + viewport3D.width || mouse.y < viewport3D.y ||
        mouse.y > viewport3D.y + viewport3D.height)
        return;

    // Plain (not viewport-scoped) ray: BeginMode3D derives its projection's
    // aspect ratio from the *full* window framebuffer regardless of the
    // BeginScissorMode() sub-region this 3D view is confined to (see
    // PointPicking.h), so the ray must be built the same way -- matching
    // OrbitCamera::pickGroundPlaneTarget's existing convention.
    Ray ray = GetScreenToWorldRay(mouse, cam3d);

    size_t hitIndex = 0;
    bool hit = raylib_widgets::pickNearestPoint(
        state.cloudPointsRaylib.data(), state.cloudPointsRaylib.size(), ray, cam3d.fovy, (float)GetScreenHeight(), 12.f, hitIndex);
    if (hit)
    {
        const auto& hp = state.cloud.points[hitIndex];
        Vector3 wp = state.cloudPointsRaylib[hitIndex];
        DrawSphere(wp, 0.08f, LIME);

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
            state.setPendingCloudPoint(hp.x, hp.y, hp.z);
    }
    else if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
    {
        state.statusMsg = "No point found near cursor, try again";
    }
}

// ── App::draw ─────────────────────────────────────────────────────────────────
// The 3D view fills the whole background (no more fixed top/bottom split
// with the image view) -- the image is now a normal floating ImGui window
// (see UI::draw) that the user can move/resize/overlap on top of it, like
// any other panel.
void App::draw()
{
    float menuBarH = ImGui::GetFrameHeight();
    float panelW = 340.f;
    float viewW = (float)GetScreenWidth() - panelW;
    float viewH = (float)GetScreenHeight() - menuBarH;

    // ── Image + projection overlay (GPU, into render texture)
    // Hide the projected-point overlay while Shift is held (picking mode)
    // so it doesn't obscure the pixel the user is aiming for.
    if (state.imageLoaded)
        state.renderer.renderImageOverlay(
            state.imageTexture,
            state.imageW,
            state.imageH,
            state.intrinsics,
            state.extrinsics,
            !state.imageRectified,
            state.vizParams,
            !shiftHeld());

    // ── 3D scene renders as the raylib background, filling the whole view
    BeginDrawing();
    ClearBackground(Color{ 30, 30, 30, 255 });

    // Note: raylib scissor is in screen coords (y-down)
    BeginScissorMode(0, (int)menuBarH, (int)viewW, (int)viewH);

    Camera3D cam3d = state.orbit.toRaylib();
    BeginMode3D(cam3d);

    state.renderer.draw3DCloud(
        state.cloud,
        state.vizParams,
        state.intrinsics,
        state.extrinsics,
        state.imageTexture,
        state.imageLoaded,
        state.imageW,
        state.imageH);
    if (state.imageLoaded)
        state.renderer.drawCameraFrustum(state.intrinsics, state.extrinsics, state.imageW, state.imageH);
    state.renderer.drawAxes(2.f);

    // Grid on ground plane
    DrawGrid(20, 1.f);

    Rectangle viewport3D = { 0.f, menuBarH, viewW, viewH };
    std::vector<std::pair<Vector2, int>> pickLabels;
    updateAndDrawPicking3D(state, cam3d, viewport3D, pickLabels);

    EndMode3D();
    EndScissorMode();

    // Index labels for the pair markers -- 2D text, drawn after EndMode3D
    // (see updateAndDrawPicking3D's comment for why).
    for (const auto& [screenPos, index] : pickLabels)
        DrawText(std::to_string(index).c_str(), (int)screenPos.x + 8, (int)screenPos.y - 8, 14, YELLOW);

    if (state.showCompassRuler)
    {
        Vector3 fwd = Vector3Normalize(Vector3Subtract(cam3d.target, cam3d.position));
        Vector3 right = Vector3Normalize(Vector3CrossProduct(fwd, cam3d.up));
        Vector3 up = Vector3CrossProduct(right, fwd);
        raylib_widgets::drawCompassRuler(right, up, state.orbit.distance, LIGHTGRAY);
    }

    // ── 3D label
    if (shiftHeld())
        DrawText("Shift+click to pick a 3D point (green = candidate)", 8, (int)menuBarH + 4, 14, YELLOW);
    else
        DrawText("3D View  [LMB: orbit | RMB: pan | Scroll: zoom | Shift: pick]", 8, (int)menuBarH + 4, 14, LIGHTGRAY);

    DrawFPS((int)viewW - 90, (int)menuBarH + 4);

    // ── ImGui on top ─────────────────────────────────────────────────────────
    rlImGuiBegin();
    state.ui.draw(state);
    rlImGuiEnd();

    EndDrawing();
}

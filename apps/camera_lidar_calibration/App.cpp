#include "App.h"
#include "rlImGui.h"
#include "imgui.h"
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <fstream>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <vector>

// ── AppState::rebuildImageTexture ─────────────────────────────────────────────
void AppState::rebuildImageTexture() {
    if (originalImage.empty()) return;

    cv::Mat display = originalImage;
    imageRectified = false;

    if (intrinsicsLoaded) {
        cv::Mat K = (cv::Mat_<double>(3, 3) <<
            intrinsics.fx, 0, intrinsics.cx,
            0, intrinsics.fy, intrinsics.cy,
            0, 0, 1);
        // OpenCV distCoeffs order: k1 k2 p1 p2 k3 k4 k5 k6 (rational model)
        cv::Mat D = (cv::Mat_<double>(1, 8) <<
            intrinsics.k1, intrinsics.k2, intrinsics.p1, intrinsics.p2,
            intrinsics.k3, intrinsics.k4, intrinsics.k5, intrinsics.k6);

        cv::Mat map1, map2;
        cv::initUndistortRectifyMap(K, D, cv::Mat(), K,
                                    originalImage.size(), CV_16SC2, map1, map2);
        cv::Mat rectified;
        cv::remap(originalImage, rectified, map1, map2, cv::INTER_LINEAR);
        display = rectified;
        imageRectified = true;
    }

    if (imageLoaded) UnloadTexture(imageTexture);
    Image rimg = {};
    rimg.data    = display.data;
    rimg.width   = display.cols;
    rimg.height  = display.rows;
    rimg.mipmaps = 1;
    rimg.format  = PIXELFORMAT_UNCOMPRESSED_R8G8B8;
    imageTexture = LoadTextureFromImage(rimg);  // copies pixels to GPU
    imageLoaded  = true;
}

// ── AppState::loadImage ───────────────────────────────────────────────────────
void AppState::loadImage(const char* path) {
    cv::Mat bgr = cv::imread(path, cv::IMREAD_COLOR);
    if (bgr.empty()) {
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
static void centerOrbitOnCloud(AppState& s) {
    s.orbit.target = {
        (s.cloud.minX + s.cloud.maxX) * 0.5f,
        (s.cloud.minZ + s.cloud.maxZ) * 0.5f,
        -(s.cloud.minY + s.cloud.maxY) * 0.5f
    };
    float span = std::max({s.cloud.maxX - s.cloud.minX,
                           s.cloud.maxY - s.cloud.minY,
                           s.cloud.maxZ - s.cloud.minZ});
    s.orbit.distance = span * 0.8f;
}

void AppState::loadCloud(const char* path) {
    if (!cloud.load(path)) {
        statusMsg = std::string("Failed to load cloud: ") + path;
        return;
    }
    cloudPaths = {path};
    renderer.uploadCloud(cloud);
    centerOrbitOnCloud(*this);
    statusMsg = "";
}

void AppState::addCloud(const char* path) {
    PointCloud extra;
    if (!extra.load(path)) {
        statusMsg = std::string("Failed to load: ") + path;
        return;
    }
    // merge bounding box
    if (cloud.empty()) {
        cloud = std::move(extra);
    } else {
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
static void extractNumbers(const std::string& s, std::vector<double>& out) {
    const char* p = s.c_str();
    while (*p) {
        if ((*p >= '0' && *p <= '9') || *p == '-' || *p == '+' || *p == '.') {
            char* end = nullptr;
            double v = std::strtod(p, &end);
            if (end != p) { out.push_back(v); p = end; continue; }
        }
        ++p;
    }
}

static bool parseOpenCVYaml(const char* path, Intrinsics& K,
                            int& imgW, int& imgH, std::string& err) {
    std::ifstream f(path);
    if (!f) { err = "cannot open file"; return false; }

    std::vector<double> camMat, dist;
    std::vector<double>* active = nullptr;  // section whose data we collect
    std::vector<double>* collecting = nullptr;
    bool inFlow = false;

    std::string line;
    while (std::getline(f, line)) {
        std::string trimmed = line;
        trimmed.erase(0, trimmed.find_first_not_of(" \t"));

        if (inFlow) {
            extractNumbers(trimmed, *collecting);
            if (trimmed.find(']') != std::string::npos) { inFlow = false; collecting = nullptr; }
            continue;
        }

        bool topLevel = !line.empty() && line[0] != ' ' && line[0] != '\t' && line[0] != '-';
        if (topLevel) {
            collecting = nullptr;
            if      (trimmed.rfind("camera_matrix:", 0) == 0)           active = &camMat;
            else if (trimmed.rfind("distortion_coefficients:", 0) == 0) active = &dist;
            else {
                active = nullptr;
                if (trimmed.rfind("image_width:", 0) == 0)
                    imgW = std::atoi(trimmed.c_str() + 12);
                else if (trimmed.rfind("image_height:", 0) == 0)
                    imgH = std::atoi(trimmed.c_str() + 13);
            }
            continue;
        }

        if (active && trimmed.rfind("data:", 0) == 0) {
            auto bracket = trimmed.find('[');
            if (bracket != std::string::npos) {
                extractNumbers(trimmed.substr(bracket), *active);
                if (trimmed.find(']') == std::string::npos) {
                    collecting = active;
                    inFlow = true;
                }
            } else {
                collecting = active;  // block list follows
            }
            continue;
        }

        if (collecting) {
            if (trimmed.rfind("- ", 0) == 0 || trimmed.rfind("-", 0) == 0)
                extractNumbers(trimmed, *collecting);
            else
                collecting = nullptr;  // rows:/cols: or another key ends the list
        }
    }

    if (camMat.size() < 9) { err = "camera_matrix needs 9 values"; return false; }

    // Row-major 3x3: [fx 0 cx; 0 fy cy; 0 0 1]
    K.fx = static_cast<float>(camMat[0]);
    K.cx = static_cast<float>(camMat[2]);
    K.fy = static_cast<float>(camMat[4]);
    K.cy = static_cast<float>(camMat[5]);

    auto d = [&](size_t i) { return i < dist.size() ? static_cast<float>(dist[i]) : 0.f; };
    K.k1 = d(0); K.k2 = d(1);
    K.p1 = d(2); K.p2 = d(3);
    K.k3 = d(4);
    K.k4 = d(5); K.k5 = d(6); K.k6 = d(7);
    return true;
}

// ── AppState::loadIntrinsics ──────────────────────────────────────────────────
void AppState::loadIntrinsics(const char* path) {
    std::string p = path;
    auto dot = p.rfind('.');
    std::string ext = (dot != std::string::npos) ? p.substr(dot + 1) : "";
    for (auto& c : ext) c = static_cast<char>(tolower(c));

    if (ext == "yml" || ext == "yaml") {
        int imgW = 0, imgH = 0;
        std::string err;
        if (!parseOpenCVYaml(path, intrinsics, imgW, imgH, err)) {
            statusMsg = std::string("YAML error: ") + err + " (" + path + ")";
            return;
        }
        intrinsicsLoaded = true;
        rebuildImageTexture();  // re-rectify with the new coefficients
        statusMsg = "Intrinsics loaded";
        if (imageRectified) statusMsg += ", image rectified";
        if (imgW > 0) {
            statusMsg += " (camera " + std::to_string(imgW) + "x" + std::to_string(imgH) + ")";
            if (imageLoaded && (imgW != imageW || imgH != imageH))
                statusMsg += " WARNING: image is " + std::to_string(imageW) + "x" + std::to_string(imageH);
        }
        return;
    }

    std::ifstream f(path);
    if (!f) { statusMsg = std::string("Cannot open: ") + path; return; }
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
void AppState::loadCalibration(const char* path) {
    std::ifstream f(path);
    if (!f) { statusMsg = std::string("Cannot open: ") + path; return; }
    nlohmann::json j;
    try { f >> j; }
    catch (...) { statusMsg = std::string("JSON parse error: ") + path; return; }

    bool gotIntrinsics = false, gotExtrinsics = false;

    if (j.contains("intrinsics")) {
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

    if (j.contains("extrinsics")) {
        auto& je = j["extrinsics"];
        // camera_position_in_world_xyz: [tx, ty, tz]
        if (je.contains("camera_position_in_world_xyz") &&
            je["camera_position_in_world_xyz"].size() >= 3) {
            auto& pos = je["camera_position_in_world_xyz"];
            extrinsics.tx = pos[0].get<float>();
            extrinsics.ty = pos[1].get<float>();
            extrinsics.tz = pos[2].get<float>();
        }
        // camera_rotation_in_world_euler_zyx_deg: [rz, ry, rx]
        if (je.contains("camera_rotation_in_world_euler_zyx_deg") &&
            je["camera_rotation_in_world_euler_zyx_deg"].size() >= 3) {
            auto& rot = je["camera_rotation_in_world_euler_zyx_deg"];
            extrinsics.rz = rot[0].get<float>();
            extrinsics.ry = rot[1].get<float>();
            extrinsics.rx = rot[2].get<float>();
        }
        gotExtrinsics = true;
    }

    if (!gotIntrinsics && !gotExtrinsics) {
        statusMsg = std::string("No intrinsics/extrinsics found in: ") + path;
        return;
    }

    if (gotIntrinsics)
        rebuildImageTexture();

    statusMsg = "Loaded";
    if (gotIntrinsics) statusMsg += " intrinsics";
    if (gotIntrinsics && gotExtrinsics) statusMsg += " +";
    if (gotExtrinsics) statusMsg += " extrinsics";
    statusMsg += std::string(" from ") + path;
}

// ── AppState::saveCalibration ─────────────────────────────────────────────────
void AppState::saveCalibration(const char* path) {
    // World-frame convention: R = R_wc (camera orientation in world, ZYX Euler)
    // C = camera position in world.  T_lidar_to_cam = [R_wc^T | -R_wc^T*C]
    Eigen::Matrix3f R  = eulerZYXtoMat3(extrinsics.rx, extrinsics.ry, extrinsics.rz);
    Eigen::Vector3f C(extrinsics.tx, extrinsics.ty, extrinsics.tz);
    Eigen::Vector3f ti = -(R.transpose() * C);  // translation of T_lidar_to_camera

    nlohmann::json j;
    j["intrinsics"] = {
        {"fx", intrinsics.fx}, {"fy", intrinsics.fy},
        {"cx", intrinsics.cx}, {"cy", intrinsics.cy},
        {"k1", intrinsics.k1}, {"k2", intrinsics.k2}, {"k3", intrinsics.k3},
        {"k4", intrinsics.k4}, {"k5", intrinsics.k5}, {"k6", intrinsics.k6},
        {"p1", intrinsics.p1}, {"p2", intrinsics.p2}
    };
    j["extrinsics"]["camera_rotation_in_world_euler_zyx_deg"] = {extrinsics.rz, extrinsics.ry, extrinsics.rx};
    j["extrinsics"]["camera_position_in_world_xyz"] = {C.x(), C.y(), C.z()};
    j["extrinsics"]["camera_rotation_matrix_in_world"] = {
        {R(0,0), R(0,1), R(0,2)},
        {R(1,0), R(1,1), R(1,2)},
        {R(2,0), R(2,1), R(2,2)}
    };
    j["extrinsics"]["T_lidar_to_camera_4x4"] = {
        {R(0,0), R(1,0), R(2,0), ti(0)},
        {R(0,1), R(1,1), R(2,1), ti(1)},
        {R(0,2), R(1,2), R(2,2), ti(2)},
        {0, 0, 0, 1}
    };

    std::ofstream f(path);
    if (!f) { statusMsg = std::string("Cannot write: ") + path; return; }
    f << j.dump(4);
    statusMsg = std::string("Saved to ") + path;
    printf("Calibration saved to %s\n", path);
}

// ── App::run ──────────────────────────────────────────────────────────────────
void App::run() {
    const int W = 1400, H = 900;
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(W, H, "LiDAR-Camera Calibration");
    SetTargetFPS(60);

    rlImGuiSetup(true); // dark theme

    state.renderer.initPointShader();

    // Load files passed as command-line arguments
    if (!pendingImage.empty())       state.loadImage(pendingImage.c_str());
    for (size_t i = 0; i < pendingClouds.size(); ++i)
        (i == 0 ? state.loadCloud(pendingClouds[i].c_str())
                : state.addCloud(pendingClouds[i].c_str()));
    if (!pendingIntrinsics.empty())  state.loadIntrinsics(pendingIntrinsics.c_str());
    if (!pendingCalibration.empty()) state.loadCalibration(pendingCalibration.c_str());

    while (!WindowShouldClose()) {
        update();
        draw();
    }

    if (state.imageLoaded)
        UnloadTexture(state.imageTexture);
    state.renderer.shutdown();
    rlImGuiShutdown();
    CloseWindow();
}

// ── App::update ───────────────────────────────────────────────────────────────
void App::update() {
    bool imguiWantMouse = ImGui::GetIO().WantCaptureMouse;
    state.orbit.update(!imguiWantMouse);
}

// ── App::draw ─────────────────────────────────────────────────────────────────
void App::draw() {
    float panelW = 340.f;
    float viewW  = (float)GetScreenWidth() - panelW;
    float viewH  = (float)GetScreenHeight();
    float view3DY = viewH * 0.5f; // 3D starts at middle

    // ── Image + projection overlay (GPU, into render texture)
    if (state.imageLoaded)
        state.renderer.renderImageOverlay(state.imageTexture,
                                          state.imageW, state.imageH,
                                          state.intrinsics, state.extrinsics,
                                          !state.imageRectified,
                                          state.vizParams);

    // ── 3D scene renders in the bottom-left area (as raylib background)
    BeginDrawing();
    ClearBackground(Color{30, 30, 30, 255});

    // Clipping for 3D region (bottom-left)
    // Note: raylib scissor is in screen coords (y-down)
    BeginScissorMode(0, (int)view3DY, (int)viewW, (int)(viewH - view3DY));

    Camera3D cam3d = state.orbit.toRaylib();
    BeginMode3D(cam3d);

    state.renderer.draw3DCloud(state.cloud, state.vizParams,
                               state.intrinsics, state.extrinsics,
                               state.imageTexture, state.imageLoaded,
                               state.imageW, state.imageH);
    if (state.imageLoaded)
        state.renderer.drawCameraFrustum(state.intrinsics, state.extrinsics,
                                         state.imageW, state.imageH);
    state.renderer.drawAxes(2.f);

    // Grid on ground plane
    DrawGrid(20, 1.f);

    EndMode3D();
    EndScissorMode();

    // ── 3D label
    DrawText("3D View  [LMB: orbit | RMB: pan | Scroll: zoom]",
             8, (int)view3DY + 4, 14, LIGHTGRAY);

    // ── Divider line
    DrawLineEx(Vector2{0, view3DY}, Vector2{viewW, view3DY}, 1.f, GRAY);

    // ── ImGui on top ─────────────────────────────────────────────────────────
    rlImGuiBegin();
    state.ui.draw(state);
    rlImGuiEnd();

    EndDrawing();
}

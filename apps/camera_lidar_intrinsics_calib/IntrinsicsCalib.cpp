#include "imgui.h"
#include "raylib.h"
#include "rlImGui.h"
#include <CalibCore/CliArgs.h>
#include <CalibCore/FileDialog.h>
#include <cstdio>
#include <nlohmann/json.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
// findChessboardCorners/drawChessboardCorners and the CALIB_CB_* flags moved
// out of calib3d.hpp into objdetect.hpp in OpenCV 5.
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <opencv2/objdetect.hpp>
#include <string>
#include <thread>
#include <vector>

using namespace calib;
namespace fs = std::filesystem;

// Copies `path` into `buf` (truncating to fit), for wiring a native-dialog
// result back into the same fixed-size char[] the matching text field edits.
static void setBuf(char* buf, size_t bufSize, const std::string& path)
{
    if (path.empty())
        return;
    std::strncpy(buf, path.c_str(), bufSize - 1);
    buf[bufSize - 1] = '\0';
}

// Shrinks/repositions the just-created window so it fits within the current
// monitor's usable area. Without this, a fixed-size window can be taller
// than the screen once the OS menu bar + title bar are accounted for,
// silently pushing the top of the window off-screen behind the menu bar
// instead of erroring or scrolling.
static void fitWindowToScreen()
{
    int monitor = GetCurrentMonitor();
    // GetMonitorWidth/Height return the monitor's native PIXEL resolution
    // (GLFW's glfwGetVideoMode), while GetScreenWidth/Height, SetWindowSize
    // and SetWindowPosition all operate in logical points -- on a 2x Retina
    // display that's a 2x unit mismatch. Divide by the DPI scale to bring
    // the monitor size into the same points space everything else uses;
    // without this, SetWindowPosition computes an X centered on a monitor
    // twice too wide, pushing most of the window off the right edge of the
    // actual (points-sized) screen.
    Vector2 dpi = GetWindowScaleDPI();
    if (dpi.x <= 0.f)
        dpi.x = 1.f;
    if (dpi.y <= 0.f)
        dpi.y = 1.f;
    int monW = (int)(GetMonitorWidth(monitor) / dpi.x);
    int monH = (int)(GetMonitorHeight(monitor) / dpi.y);
    if (monW <= 0 || monH <= 0)
        return; // monitor info unavailable, leave as-is

    const int marginW = 40; // side breathing room
    const int marginH = 100; // OS menu bar + window title bar headroom

    int w = std::min(GetScreenWidth(), monW - marginW);
    int h = std::min(GetScreenHeight(), monH - marginH);
    if (w != GetScreenWidth() || h != GetScreenHeight())
        SetWindowSize(w, h);

    SetWindowPosition(std::max(0, (monW - w) / 2), 30);

    // SetWindowSize/SetWindowPosition only update GLFW's window state; raylib's
    // cached mouse/window geometry (what rlImGui reads into io.MousePos every
    // frame) isn't refreshed until the next PollInputEvents(), which otherwise
    // wouldn't happen until the first EndDrawing() -- after rlImGuiSetup() has
    // already run. Without this, every click lands offset from the cursor by
    // however far this function just moved/resized the window.
    PollInputEvents();
}

// ── per-image state ───────────────────────────────────────────────────────────
struct CalibImage
{
    std::string path;
    cv::Mat rgb;
    std::vector<cv::Point2f> corners;
    bool detected = false;
    bool processed = false;
};

// ── application state ─────────────────────────────────────────────────────────
struct State
{
    // board parameters
    int boardCols = 10; // inner corner count
    int boardRows = 7;
    float squareMm = 25.f;

    // loaded images
    char dirBuf[512] = {};
    std::vector<CalibImage> images;
    int currentIdx = 0;

    // display texture (current image + drawn corners)
    Texture2D tex = {};
    bool texOk = false;
    int texIdx = -1; // which image is on GPU

    // calibration results
    bool calibrated = false;
    double rmsError = 0.0;
    cv::Mat K, D;
    cv::Size imageSize;

    // output
    char outPath[512] = "intrinsics.json";
    std::string statusMsg;

    // background detection
    std::thread detectThread;
    std::atomic<int> detectProgress{ -1 }; // -1=idle, [0,N)=index in progress, N=done
    std::atomic<bool> detectStop{ false };
    int detectTotal = 0;

    bool isDetecting() const
    {
        int p = detectProgress.load();
        return p >= 0 && p < detectTotal;
    }
};

// ── helpers ───────────────────────────────────────────────────────────────────
static void loadDir(State& s)
{
    s.images.clear();
    s.calibrated = false;
    s.currentIdx = 0;
    s.texIdx = -1;

    fs::path dir(s.dirBuf);
    if (!fs::is_directory(dir))
    {
        s.statusMsg = "Not a directory: " + std::string(s.dirBuf);
        return;
    }

    const std::vector<std::string> exts = { ".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".tif" };
    std::vector<std::string> paths;
    for (auto& e : fs::directory_iterator(dir))
    {
        if (!e.is_regular_file())
            continue;
        std::string ext = e.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        for (auto& x : exts)
            if (ext == x)
            {
                paths.push_back(e.path().string());
                break;
            }
    }
    std::sort(paths.begin(), paths.end());

    for (auto& p : paths)
    {
        cv::Mat bgr = cv::imread(p, cv::IMREAD_COLOR);
        if (bgr.empty())
            continue;
        CalibImage ci;
        ci.path = p;
        cv::cvtColor(bgr, ci.rgb, cv::COLOR_BGR2RGB);
        s.images.push_back(std::move(ci));
    }
    s.statusMsg = "Loaded " + std::to_string(s.images.size()) + " images";
}

static void detectAll(State& s)
{
    if (s.isDetecting())
        return; // already running
    if (s.images.empty())
        return;

    // reset processed flags so previous results are not stale
    for (auto& ci : s.images)
        ci.processed = false;

    s.detectTotal = (int)s.images.size();
    s.detectStop.store(false);
    s.detectProgress.store(0);
    s.statusMsg.clear();

    // snapshot board params for the thread
    int cols = s.boardCols, rows = s.boardRows;

    if (s.detectThread.joinable())
        s.detectThread.join();
    s.detectThread = std::thread(
        [&s, cols, rows]()
        {
            cv::Size pat(cols, rows);
            const int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;
            // target width for detection — large enough to see corners, small enough to be fast
            const float TARGET_W = 1500.f;

            for (int i = 0; i < (int)s.images.size(); i++)
            {
                if (s.detectStop.load())
                    break;
                s.detectProgress.store(i);
                auto& ci = s.images[i];

                cv::Mat gray;
                cv::cvtColor(ci.rgb, gray, cv::COLOR_RGB2GRAY);

                // downsample for detection
                float scale = (gray.cols > TARGET_W) ? TARGET_W / gray.cols : 1.f;
                cv::Mat small;
                if (scale < 1.f)
                    cv::resize(gray, small, cv::Size(), scale, scale, cv::INTER_AREA);
                else
                    small = gray;

                bool found = cv::findChessboardCorners(small, pat, ci.corners, flags);
                if (found)
                {
                    // scale corners back to full resolution
                    if (scale < 1.f)
                        for (auto& pt : ci.corners)
                            pt *= (1.f / scale);
                    // subpix refinement on full-resolution image
                    // scale the search window proportionally to the image width
                    int win = std::max(11, (int)(11.f / scale) | 1); // keep odd
                    cv::cornerSubPix(
                        gray,
                        ci.corners,
                        cv::Size(win, win),
                        cv::Size(-1, -1),
                        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 50, 0.0001));
                }
                ci.detected = found;
                ci.processed = true;
            }
            s.detectProgress.store(s.detectTotal);
        });
}

static void runCalibration(State& s)
{
    std::vector<cv::Point3f> objPts;
    objPts.reserve(s.boardCols * s.boardRows);
    for (int r = 0; r < s.boardRows; r++)
        for (int c = 0; c < s.boardCols; c++)
            objPts.push_back(cv::Point3f(c * s.squareMm, r * s.squareMm, 0.f));

    std::vector<std::vector<cv::Point3f>> allObj;
    std::vector<std::vector<cv::Point2f>> allImg;
    for (auto& ci : s.images)
    {
        if (!ci.detected)
            continue;
        allObj.push_back(objPts);
        allImg.push_back(ci.corners);
        s.imageSize = cv::Size(ci.rgb.cols, ci.rgb.rows);
    }

    if ((int)allObj.size() < 4)
    {
        s.statusMsg = "Need at least 4 images with detected corners";
        return;
    }

    s.K = cv::Mat::eye(3, 3, CV_64F);
    s.D = cv::Mat::zeros(8, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;

    s.rmsError = cv::calibrateCamera(allObj, allImg, s.imageSize, s.K, s.D, rvecs, tvecs, cv::CALIB_RATIONAL_MODEL);
    s.calibrated = true;
    s.statusMsg = "RMS: " + std::to_string(s.rmsError).substr(0, 5) + " px  (" + std::to_string(allObj.size()) + " images)";
}

static void saveJson(const State& s)
{
    if (!s.calibrated)
        return;
    double fx = s.K.at<double>(0, 0);
    double fy = s.K.at<double>(1, 1);
    double cx = s.K.at<double>(0, 2);
    double cy = s.K.at<double>(1, 2);
    // CALIB_RATIONAL_MODEL dist order: k1 k2 p1 p2 k3 k4 k5 k6
    auto d = [&](int i)
    {
        return i < s.D.rows ? s.D.at<double>(i) : 0.0;
    };

    nlohmann::json j;
    j["intrinsics"] = { { "fx", fx },   { "fy", fy },   { "cx", cx },   { "cy", cy },   { "k1", d(0) }, { "k2", d(1) },
                        { "p1", d(2) }, { "p2", d(3) }, { "k3", d(4) }, { "k4", d(5) }, { "k5", d(6) }, { "k6", d(7) } };
    j["image_size"] = { s.imageSize.width, s.imageSize.height };
    j["rms_error"] = s.rmsError;

    std::ofstream f(s.outPath);
    if (f)
        f << j.dump(4);
}

// Upload current image (with corners drawn) to a raylib texture.
static void refreshTex(State& s)
{
    if (s.images.empty())
        return;
    s.currentIdx = std::max(0, std::min(s.currentIdx, (int)s.images.size() - 1));
    if (s.currentIdx == s.texIdx)
        return;

    auto& ci = s.images[s.currentIdx];
    cv::Mat display = ci.rgb.clone();

    if (ci.processed)
    {
        cv::Mat tmp;
        cv::cvtColor(display, tmp, cv::COLOR_RGB2BGR);
        cv::drawChessboardCorners(tmp, cv::Size(s.boardCols, s.boardRows), ci.corners, ci.detected);
        cv::cvtColor(tmp, display, cv::COLOR_BGR2RGB);
    }

    if (s.texOk)
        UnloadTexture(s.tex);
    Image img = {};
    img.data = display.data;
    img.width = display.cols;
    img.height = display.rows;
    img.mipmaps = 1;
    img.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8;
    s.tex = LoadTextureFromImage(img);
    s.texOk = true;
    s.texIdx = s.currentIdx;
}

// ── entry point ───────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    CliArgs args = parseArgs(argc, argv);
    if (args.help)
    {
        printUsage("IntrinsicsCalib", "Camera intrinsics calibration from a folder of images", { cliopt::CAMERA_DIR });
        return 0;
    }
    if (!args.valid)
    {
        std::fprintf(stderr, "%s\n\n", args.error.c_str());
        printUsage("IntrinsicsCalib", "Camera intrinsics calibration from a folder of images", { cliopt::CAMERA_DIR }, /*toStderr=*/true);
        return 1;
    }

    State state;
    // --camera_dir, or the first positional, selects the image folder.
    std::string dir =
        args.has("camera_dir") ? args.get("camera_dir") : (!args.positional.empty() ? args.positional.front() : std::string{});
    if (!dir.empty())
    {
        strncpy(state.dirBuf, dir.c_str(), sizeof(state.dirBuf) - 1);
        loadDir(state);
    }

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(1280, 800, "Intrinsics Calibration");
    fitWindowToScreen();
    // PANEL_W below (330) is fixed; below this the image view and panel
    // start overlapping instead of scrolling.
    SetWindowMinSize(800, 500);
    SetTargetFPS(60);
    rlImGuiSetup(true);

    const float PANEL_W = 330.f;

    while (!WindowShouldClose())
    {
        refreshTex(state);

        BeginDrawing();
        ClearBackground(Color{ 30, 30, 30, 255 });

        // ── image view (left area) ────────────────────────────────────────────
        if (state.texOk)
        {
            float aw = GetScreenWidth() - PANEL_W;
            float ah = GetScreenHeight();
            float sx = aw / state.tex.width;
            float sy = ah / state.tex.height;
            float sc = std::min(sx, sy);
            float dw = state.tex.width * sc;
            float dh = state.tex.height * sc;
            DrawTexturePro(
                state.tex,
                { 0, 0, (float)state.tex.width, (float)state.tex.height },
                { (aw - dw) * 0.5f, (ah - dh) * 0.5f, dw, dh },
                { 0, 0 },
                0.f,
                WHITE);
        }

        // ── ImGui panel (right) ───────────────────────────────────────────────
        rlImGuiBegin();
        ImGuiIO& io = ImGui::GetIO();
        ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - PANEL_W, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(PANEL_W, io.DisplaySize.y), ImGuiCond_Always);
        ImGui::Begin(
            "Controls",
            nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse);

        ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.f, 1.f), "Intrinsics Calibration");
        ImGui::Separator();

        // ── board ─────────────────────────────────────────────────────────────
        if (ImGui::CollapsingHeader("Checkerboard", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::PushItemWidth(-100.f);
            ImGui::InputInt("Inner cols", &state.boardCols);
            ImGui::InputInt("Inner rows", &state.boardRows);
            ImGui::DragFloat("Square mm", &state.squareMm, 0.5f, 1.f, 500.f, "%.1f");
            state.boardCols = std::max(2, state.boardCols);
            state.boardRows = std::max(2, state.boardRows);
            ImGui::PopItemWidth();
        }

        // ── images ────────────────────────────────────────────────────────────
        if (ImGui::CollapsingHeader("Images", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::PushItemWidth(-1);
            ImGui::Text("Image directory:");
            ImGui::InputText("##dir", state.dirBuf, sizeof(state.dirBuf));
            if (ImGui::Button("Browse...##dir", ImVec2(-1, 0)))
                setBuf(state.dirBuf, sizeof(state.dirBuf), calib::fd::SelectFolder("Select checkerboard image directory"));
            if (ImGui::Button("Load", ImVec2(-1, 0)))
                loadDir(state);
            ImGui::Text("%zu images", state.images.size());
            ImGui::PopItemWidth();

            if (!state.images.empty())
            {
                ImGui::Spacing();
                int n = (int)state.images.size();
                float hw = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
                if (ImGui::Button("< Prev", ImVec2(hw, 0)))
                {
                    state.currentIdx = (state.currentIdx - 1 + n) % n;
                    state.texIdx = -1;
                }
                ImGui::SameLine();
                if (ImGui::Button("Next >", ImVec2(hw, 0)))
                {
                    state.currentIdx = (state.currentIdx + 1) % n;
                    state.texIdx = -1;
                }
                auto& ci = state.images[state.currentIdx];
                ImGui::Text("%d / %d  %s", state.currentIdx + 1, n, fs::path(ci.path).filename().string().c_str());
                if (ci.processed)
                {
                    if (ci.detected)
                        ImGui::TextColored(ImVec4(0, 1, 0, 1), "Corners found (%zu)", ci.corners.size());
                    else
                        ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "No corners detected");
                }
            }
        }

        // ── calibration ───────────────────────────────────────────────────────
        if (ImGui::CollapsingHeader("Calibration", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (!state.images.empty())
            {
                int prog = state.detectProgress.load();
                if (state.isDetecting())
                {
                    // show progress bar — button disabled
                    float frac = (float)prog / (float)state.detectTotal;
                    ImGui::ProgressBar(frac, ImVec2(-1, 0));
                    ImGui::TextDisabled("Detecting %d / %d ...", prog, state.detectTotal);
                    state.texIdx = -1; // keep refreshing current image as it gets processed
                }
                else
                {
                    // detection finished or not started — update status once
                    if (prog == state.detectTotal && state.detectTotal > 0)
                    {
                        int good2 = 0;
                        for (auto& ci : state.images)
                            if (ci.detected)
                                good2++;
                        state.statusMsg = "Detected: " + std::to_string(good2) + " / " + std::to_string(state.images.size());
                        state.detectProgress.store(-1); // back to idle
                        if (state.detectThread.joinable())
                            state.detectThread.join();
                    }
                    if (ImGui::Button("Detect corners in all", ImVec2(-1, 0)))
                        detectAll(state);
                }
            }

            int good = 0;
            for (auto& ci : state.images)
                if (ci.detected)
                    good++;
            if (!state.images.empty())
                ImGui::Text("Good images: %d / %zu", good, state.images.size());

            if (good >= 4)
            {
                if (ImGui::Button("Run calibration", ImVec2(-1, 0)))
                    runCalibration(state);
            }
        }

        // ── results ───────────────────────────────────────────────────────────
        if (state.calibrated)
        {
            if (ImGui::CollapsingHeader("Results", ImGuiTreeNodeFlags_DefaultOpen))
            {
                double fx = state.K.at<double>(0, 0);
                double fy = state.K.at<double>(1, 1);
                double cx = state.K.at<double>(0, 2);
                double cy = state.K.at<double>(1, 2);
                auto d = [&](int i)
                {
                    return i < state.D.rows ? state.D.at<double>(i) : 0.0;
                };
                ImGui::Text("Image:  %d x %d", state.imageSize.width, state.imageSize.height);
                ImGui::Text("fx:     %.2f", fx);
                ImGui::Text("fy:     %.2f", fy);
                ImGui::Text("cx:     %.2f", cx);
                ImGui::Text("cy:     %.2f", cy);
                ImGui::Separator();
                ImGui::Text("k1:     %.5f", d(0));
                ImGui::Text("k2:     %.5f", d(1));
                ImGui::Text("p1:     %.5f", d(2));
                ImGui::Text("p2:     %.5f", d(3));
                ImGui::Text("k3:     %.5f", d(4));
                ImGui::Text("k4:     %.5f", d(5));
                ImGui::Text("k5:     %.5f", d(6));
                ImGui::Text("k6:     %.5f", d(7));
                ImGui::Separator();
                ImGui::TextColored(
                    state.rmsError < 1.0 ? ImVec4(0, 1, 0, 1) : ImVec4(1, 0.6f, 0, 1), "RMS reprojection: %.4f px", state.rmsError);
                ImGui::Spacing();
                ImGui::PushItemWidth(-1);
                ImGui::InputText("##out", state.outPath, sizeof(state.outPath));
                if (ImGui::Button("Browse...##out", ImVec2(-1, 0)))
                {
                    std::string defaultName = fs::path(state.outPath).filename().string();
                    setBuf(
                        state.outPath,
                        sizeof(state.outPath),
                        calib::fd::SaveFileDialog("Save intrinsics JSON", calib::fd::CalibJsonFilter, ".json", defaultName));
                }
                if (ImGui::Button("Save JSON", ImVec2(-1, 0)))
                {
                    saveJson(state);
                    state.statusMsg = std::string("Saved: ") + state.outPath;
                }
                ImGui::PopItemWidth();
            }
        }

        // ── status ────────────────────────────────────────────────────────────
        if (!state.statusMsg.empty())
        {
            ImGui::Separator();
            ImGui::TextColored(ImVec4(1, 1, 0, 1), "%s", state.statusMsg.c_str());
        }

        ImGui::End();
        rlImGuiEnd();
        EndDrawing();
    }

    // stop background detection if still running
    state.detectStop.store(true);
    if (state.detectThread.joinable())
        state.detectThread.join();

    if (state.texOk)
        UnloadTexture(state.tex);
    rlImGuiShutdown();
    CloseWindow();
    return 0;
}
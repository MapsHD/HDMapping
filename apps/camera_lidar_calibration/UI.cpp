#include "UI.h"
#include "App.h"
#include "imgui.h"
#include "rlImGui.h"
#include <Core/pfd_wrapper.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <nlohmann/json.hpp>

// Copies `path` into `buf` (truncating to fit), for wiring a native-dialog
// result back into the same fixed-size char[] the text field edits.
static void setBuf(char* buf, size_t bufSize, const std::string& path)
{
    if (path.empty())
        return;
    std::strncpy(buf, path.c_str(), bufSize - 1);
    buf[bufSize - 1] = '\0';
}

// DragFloat with Shift=fine mode (10x smaller step)
static bool dragFloat(const char* label, float* v, float speed, float lo, float hi, const char* fmt = "%.3f")
{
    if (ImGui::GetIO().KeyShift)
        speed *= 0.01f;
    return ImGui::DragFloat(label, v, speed, lo, hi, fmt);
}

static void helpMarker(const char* desc)
{
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::TextUnformatted(desc);
        ImGui::EndTooltip();
    }
}

// ── Main draw ────────────────────────────────────────────────────────────────
void UI::draw(AppState& state)
{
    panelMenuBar(state);
    handleShortcuts(state);

    ImGuiIO& io = ImGui::GetIO();
    float menuBarH = ImGui::GetFrameHeight();
    float panelW = 340.f;
    float panelH = (float)GetScreenHeight() - menuBarH;

    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - panelW, menuBarH), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(panelW, panelH), ImGuiCond_Always);
    ImGui::Begin(
        "Controls",
        nullptr,
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse);

    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.f, 1.f), "LiDAR-Camera Calibration");
    ImGui::Separator();

    // Alt/Cmd = toggle Camera RGB ↔ Intensity (works anywhere in the window).
    // Cmd (Super) alongside Alt for macOS, where Option is awkward to use as
    // a modifier (it composes special characters).
    if (ImGui::IsKeyPressed(ImGuiKey_LeftAlt) || ImGui::IsKeyPressed(ImGuiKey_RightAlt) || ImGui::IsKeyPressed(ImGuiKey_LeftSuper) ||
        ImGui::IsKeyPressed(ImGuiKey_RightSuper))
    {
        auto& cm = state.vizParams.colorMode;
        if (cm == 3)
            cm = 1; // RGB → Intensity
        else
            cm = 3; // anything → RGB
    }

    panelStatus(state);
    ImGui::Spacing();

    if (ImGui::CollapsingHeader("Files", ImGuiTreeNodeFlags_DefaultOpen))
        panelFiles(state);
    if (ImGui::CollapsingHeader("Intrinsics", ImGuiTreeNodeFlags_DefaultOpen))
        panelIntrinsics(state);
    if (ImGui::CollapsingHeader("Extrinsics", ImGuiTreeNodeFlags_DefaultOpen))
        panelExtrinsics(state);
    if (ImGui::CollapsingHeader("Visualization"))
        panelVisualization(state);

    ImGui::End();

    // ── Image view window (pan + zoom) ────────────────────────────────────
    if (state.renderer.imageTexValid)
    {
        float viewW = io.DisplaySize.x - panelW;
        float viewH = (io.DisplaySize.y - menuBarH) * 0.5f;
        ImGui::SetNextWindowPos(ImVec2(0, menuBarH), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(viewW, viewH), ImGuiCond_Always);
        ImGui::Begin(
            "Image View",
            nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoTitleBar);
        drawImageView(state);
        ImGui::End();
    }
}

// ── Image view: pan + zoom ────────────────────────────────────────────────────
// zoom = 1 means "fit to window". offX/offY = image coords of the top-left
// visible pixel. Wheel zooms anchored at the cursor, LMB-drag pans,
// double-click resets.
void UI::drawImageView(AppState& state)
{
    const float imgW = (float)state.imageW;
    const float imgH = (float)state.imageH;
    if (imgW <= 0 || imgH <= 0)
        return;

    // Reset view when a different image is loaded
    if (state.imageW != viewImgW || state.imageH != viewImgH)
    {
        viewImgW = state.imageW;
        viewImgH = state.imageH;
        zoom2D = 1.f;
        offX = offY = 0.f;
    }

    ImVec2 origin = ImGui::GetCursorScreenPos(); // content region top-left
    ImVec2 avail = ImGui::GetContentRegionAvail();
    if (avail.x < 16 || avail.y < 16)
        return;

    const float fitScale = std::min(avail.x / imgW, avail.y / imgH);
    float scale = fitScale * zoom2D;

    // Displayed size and the visible sub-rect of the image
    float dispW = std::min(avail.x, imgW * scale);
    float dispH = std::min(avail.y, imgH * scale);
    float srcW = dispW / scale;
    float srcH = dispH / scale;

    ImVec2 imgScreenPos = ImVec2(origin.x + (avail.x - dispW) * 0.5f, origin.y + (avail.y - dispH) * 0.5f);

    ImGui::SetCursorScreenPos(imgScreenPos);
    // Render textures are y-flipped: select the sub-rect with negative height
    Rectangle src = { offX, imgH - offY, srcW, -srcH };
    rlImGuiImageRect(&state.renderer.imageTex.texture, (int)dispW, (int)dispH, src);

    // ── input ──────────────────────────────────────────────────────────────
    if (ImGui::IsWindowHovered())
    {
        ImGuiIO& io = ImGui::GetIO();

        if (io.MouseWheel != 0.f)
        {
            // image point under the cursor stays put while zooming
            // float mx = io.MousePos.x - imgScreenPos.x;
            // float my = io.MousePos.y - imgScreenPos.y;
            // float ix = offX + mx / scale;
            // float iy = offY + my / scale;

            zoom2D = std::max(1.f, std::min(zoom2D * std::exp(io.MouseWheel * 0.15f), 100.f));
            scale = fitScale * zoom2D;
            // offX = ix - mx / scale;
            // offY = iy - my / scale;
        }

        if (ImGui::IsMouseDragging(ImGuiMouseButton_Left))
        {
            offX -= io.MouseDelta.x / scale;
            offY -= io.MouseDelta.y / scale;
        }

        if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
        {
            zoom2D = 1.f;
            offX = offY = 0.f;
        }
    }

    // zoom indicator
    ImGui::SetCursorScreenPos(ImVec2(origin.x + 6, origin.y + 4));
    ImGui::TextColored(ImVec4(1, 1, 0, 0.8f), "%.0f%%  [wheel: zoom | drag: pan | dbl-click: reset]", zoom2D * fitScale * 100.f);
}

// ── Menu bar ─────────────────────────────────────────────────────────────────
// File actions moved here from the side panel's Browse buttons, matching the
// File-menu convention used by the rest of HDMapping's apps (e.g.
// mandeye_single_session_viewer). The side panel keeps its path textboxes and
// Load/Add/Save buttons for the manual-path-entry workflow.
// ── File actions ─────────────────────────────────────────────────────────────
// Factored out of panelMenuBar so the File menu items and their keyboard
// shortcuts (handleShortcuts) call the exact same code, matching the
// openSession()-style convention used by mandeye_single_session_viewer etc.
void UI::actionOpenImage(AppState& state)
{
    std::string path = mandeye::fd::OpenFileDialogOneFile("Select camera image", mandeye::fd::ImageFilter);
    if (!path.empty())
    {
        setBuf(imagePathBuf, sizeof(imagePathBuf), path);
        state.loadImage(imagePathBuf);
    }
}

void UI::actionOpenPointCloud(AppState& state)
{
    std::string path = mandeye::fd::OpenFileDialogOneFile("Select point cloud", mandeye::fd::LazFilter);
    if (!path.empty())
    {
        setBuf(cloudPathBuf, sizeof(cloudPathBuf), path);
        state.loadCloud(cloudPathBuf);
    }
}

void UI::actionAddPointCloud(AppState& state)
{
    std::vector<std::string> paths = mandeye::fd::OpenFileDialog("Select point cloud(s)", mandeye::fd::LazFilter, true);
    for (const std::string& path : paths)
    {
        setBuf(cloudPathBuf, sizeof(cloudPathBuf), path);
        state.addCloud(cloudPathBuf);
    }
}

void UI::actionOpenIntrinsics(AppState& state)
{
    std::string path = mandeye::fd::OpenFileDialogOneFile("Select intrinsics file", mandeye::fd::IntrinsicsFilter);
    if (!path.empty())
    {
        setBuf(intrPathBuf, sizeof(intrPathBuf), path);
        state.loadIntrinsics(intrPathBuf);
    }
}

void UI::actionOpenCalibration(AppState& state)
{
    std::string path = mandeye::fd::OpenFileDialogOneFile("Select calibration file", mandeye::fd::json_filter);
    if (!path.empty())
    {
        setBuf(savePath, sizeof(savePath), path);
        state.loadCalibration(savePath);
    }
}

void UI::actionSaveCalibration(AppState& state)
{
    std::string path = mandeye::fd::SaveFileDialog("Save calibration file", mandeye::fd::json_filter, ".json", "calibration.json");
    if (!path.empty())
    {
        setBuf(savePath, sizeof(savePath), path);
        state.saveCalibration(savePath);
    }
}

// ── Keyboard shortcuts ───────────────────────────────────────────────────────
// Ctrl-combos follow the io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_X, false)
// + io.AddKeyEvent(...) reset convention used by mandeye_single_session_viewer
// and multi_view_tls_registration_gui.cpp. Guarded by !WantCaptureKeyboard so
// they don't fire while a path textbox has focus.
void UI::handleShortcuts(AppState& state)
{
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureKeyboard)
        return;

    auto ctrlPressed = [&io](ImGuiKey key)
    {
        bool pressed = io.KeyCtrl && ImGui::IsKeyPressed(key, false);
        if (pressed)
        {
            io.AddKeyEvent(key, false);
            io.AddKeyEvent(ImGuiMod_Ctrl, false);
        }
        return pressed;
    };

    if (ctrlPressed(ImGuiKey_I))
        actionOpenImage(state);
    if (io.KeyShift && ctrlPressed(ImGuiKey_O))
        actionAddPointCloud(state);
    else if (ctrlPressed(ImGuiKey_O))
        actionOpenPointCloud(state);
    if (ctrlPressed(ImGuiKey_K))
        actionOpenIntrinsics(state);
    if (ctrlPressed(ImGuiKey_L))
        actionOpenCalibration(state);
    if (ctrlPressed(ImGuiKey_S))
        actionSaveCalibration(state);

    if (ImGui::IsKeyPressed(ImGuiKey_C, false))
        state.showCompassRuler = !state.showCompassRuler;
}

void UI::panelMenuBar(AppState& state)
{
    if (!ImGui::BeginMainMenuBar())
        return;

    if (ImGui::BeginMenu("File"))
    {
        if (ImGui::MenuItem("Open Image...", "Ctrl+I"))
            actionOpenImage(state);
        if (ImGui::MenuItem("Open Point Cloud...", "Ctrl+O"))
            actionOpenPointCloud(state);
        if (ImGui::MenuItem("Add Point Cloud...", "Ctrl+Shift+O"))
            actionAddPointCloud(state);
        ImGui::Separator();
        if (ImGui::MenuItem("Open Intrinsics...", "Ctrl+K"))
            actionOpenIntrinsics(state);
        ImGui::Separator();
        if (ImGui::MenuItem("Open Calibration...", "Ctrl+L"))
            actionOpenCalibration(state);
        if (ImGui::MenuItem("Save Calibration...", "Ctrl+S"))
            actionSaveCalibration(state);
        ImGui::EndMenu();
    }

    ImGui::EndMainMenuBar();
}

// ── Files ────────────────────────────────────────────────────────────────────
void UI::panelFiles(AppState& state)
{
    ImGui::PushItemWidth(-1);

    ImGui::Text("JPG image:");
    ImGui::InputText("##img", imagePathBuf, sizeof(imagePathBuf));
    if (ImGui::Button("Load Image##btn", ImVec2(-1, 0)))
        state.loadImage(imagePathBuf);

    ImGui::Spacing();
    ImGui::Text("LAZ/LAS point cloud:");
    ImGui::InputText("##laz", cloudPathBuf, sizeof(cloudPathBuf));
    {
        float hw = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
        if (ImGui::Button("Load##laz", ImVec2(hw, 0)))
            state.loadCloud(cloudPathBuf);
        ImGui::SameLine();
        if (ImGui::Button("Add##laz", ImVec2(hw, 0)))
            state.addCloud(cloudPathBuf);
    }

    ImGui::Spacing();
    ImGui::Text("Intrinsics JSON/YAML (optional):");
    ImGui::InputText("##intr", intrPathBuf, sizeof(intrPathBuf));
    if (ImGui::Button("Load Intrinsics##btn", ImVec2(-1, 0)))
        state.loadIntrinsics(intrPathBuf);

    ImGui::Separator();
    ImGui::Text("Calibration JSON:");
    ImGui::InputText("##save", savePath, sizeof(savePath));
    float hw = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
    if (ImGui::Button("Load##calib", ImVec2(hw, 0)))
        state.loadCalibration(savePath);
    ImGui::SameLine();
    if (ImGui::Button("Save##calib", ImVec2(hw, 0)))
        state.saveCalibration(savePath);

    ImGui::PopItemWidth();
}

// ── Intrinsics ────────────────────────────────────────────────────────────────
void UI::panelIntrinsics(AppState& state)
{
    Intrinsics& K = state.intrinsics;
    // Re-rectify only when an edit completes — remap on a full-res image
    // is too slow to run on every drag tick.
    bool edited = false;
    auto drag = [&](const char* label, float* v, float speed, float lo, float hi, const char* fmt)
    {
        dragFloat(label, v, speed, lo, hi, fmt);
        edited |= ImGui::IsItemDeactivatedAfterEdit();
    };

    ImGui::PushItemWidth(-80.f);
    drag("fx", &K.fx, 1.f, 1.f, 10000.f, "%.1f");
    drag("fy", &K.fy, 1.f, 1.f, 10000.f, "%.1f");
    drag("cx", &K.cx, 0.5f, 0.f, 10000.f, "%.1f");
    drag("cy", &K.cy, 0.5f, 0.f, 10000.f, "%.1f");
    ImGui::Separator();
    ImGui::Text("Radial (rational model):");
    drag("k1", &K.k1, 0.001f, -100.f, 100.f, "%.4f");
    drag("k2", &K.k2, 0.001f, -100.f, 100.f, "%.4f");
    drag("k3", &K.k3, 0.001f, -100.f, 100.f, "%.4f");
    drag("k4", &K.k4, 0.001f, -100.f, 100.f, "%.4f");
    drag("k5", &K.k5, 0.001f, -100.f, 100.f, "%.4f");
    drag("k6", &K.k6, 0.001f, -100.f, 100.f, "%.4f");
    ImGui::Text("Tangential:");
    drag("p1", &K.p1, 0.0001f, -1.f, 1.f, "%.5f");
    drag("p2", &K.p2, 0.0001f, -1.f, 1.f, "%.5f");
    helpMarker("Drag to adjust. Hold Ctrl+click to type a value.");
    ImGui::PopItemWidth();

    if (edited && state.intrinsicsLoaded)
        state.rebuildImageTexture();
}

// ── Extrinsics ────────────────────────────────────────────────────────────────
void UI::panelExtrinsics(AppState& state)
{
    Extrinsics& E = state.extrinsics;

    ImGui::PushItemWidth(-80.f);

    ImGui::Text("Camera position in world (m):");
    dragFloat("tx", &E.tx, 0.01f, -50.f, 50.f, "%.3f");
    dragFloat("ty", &E.ty, 0.01f, -50.f, 50.f, "%.3f");
    dragFloat("tz", &E.tz, 0.01f, -50.f, 50.f, "%.3f");

    ImGui::Spacing();
    ImGui::Text("Camera orientation in world ZYX (deg):");
    dragFloat("rx", &E.rx, 0.1f, -180.f, 180.f, "%.2f");
    dragFloat("ry", &E.ry, 0.1f, -180.f, 180.f, "%.2f");
    dragFloat("rz", &E.rz, 0.1f, -180.f, 180.f, "%.2f");
    helpMarker("R_wc = Rz*Ry*Rx: camera orientation in LiDAR world.\nT_lidar2cam = R_wc^T * (p - C).");

    ImGui::Spacing();
    if (ImGui::Button("Reset Extrinsics", ImVec2(-1, 0)))
        E = Extrinsics{};
    ImGui::PopItemWidth();

    // Show current rotation matrix
    if (ImGui::TreeNode("Rotation matrix"))
    {
        Eigen::Matrix3f R = eulerZYXtoMat3(E.rx, E.ry, E.rz);
        for (int r = 0; r < 3; r++)
        {
            ImGui::Text("[ %6.3f  %6.3f  %6.3f ]", R(r, 0), R(r, 1), R(r, 2));
        }
        ImGui::TreePop();
    }
}

// ── Visualization ──────────────────────────────────────────────────────────
void UI::panelVisualization(AppState& state)
{
    VisualizationParams& vp = state.vizParams;

    // -140 (not -1): every widget here has a trailing label; -1 gives the
    // slider/combo box the full row width and pushes the label off the
    // right edge of the panel instead of leaving it room to draw.
    ImGui::PushItemWidth(-140.f);
    ImGui::SliderFloat("Point size", &vp.pointSize, 1.f, 20.f);
    ImGui::SliderFloat("Depth min", &vp.depthMin, 0.f, vp.depthMax);
    ImGui::SliderFloat("Depth max", &vp.depthMax, vp.depthMin + 0.1f, 200.f);
    ImGui::SliderFloat("Opacity", &vp.opacity, 0.f, 1.f);

    const char* modes[] = { "Jet (depth)", "Jet (intensity)", "Jet (height)", "Camera RGB" };
    ImGui::Combo("Color mode", &vp.colorMode, modes, 4);
    ImGui::PopItemWidth();

    ImGui::Checkbox("Show compass/ruler (C)", &state.showCompassRuler);
}

// ── Status bar ────────────────────────────────────────────────────────────────
void UI::panelStatus(const AppState& state)
{
    if (!state.imagePath.empty())
        ImGui::TextColored(ImVec4(0, 1, 0, 1), "IMG: %s (%dx%d)", state.imagePath.c_str(), state.imageW, state.imageH);
    else
        ImGui::TextColored(ImVec4(1, 0.5f, 0, 1), "No image loaded");

    if (!state.cloudPaths.empty())
    {
        ImGui::TextColored(ImVec4(0, 1, 0, 1), "LAZ: %d file(s), %zu pts", (int)state.cloudPaths.size(), state.cloud.points.size());
        for (auto& p : state.cloudPaths)
            ImGui::TextDisabled("  %s", p.c_str());
    }
    else
    {
        ImGui::TextColored(ImVec4(1, 0.5f, 0, 1), "No point cloud loaded");
    }

    if (!state.statusMsg.empty())
        ImGui::TextColored(ImVec4(1, 1, 0, 1), "%s", state.statusMsg.c_str());
}

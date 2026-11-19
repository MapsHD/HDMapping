#include "UI.h"
#include "App.h"
#include "imgui.h"
#include "rlImGui.h"
#include <Core/pfd_wrapper.hpp>
#include <RaylibWidgets/PlatformKeys.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
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

// Which physical sensors the loaded data belongs to: the camera's serial and
// frame come from the rig's camera_info.yaml, the LiDAR's from the mandeye
// status sidecar beside the LAZ. Shown together, above everything else,
// because a calibration is only valid for the one pair it was measured on.
static void drawSensorIds(const AppState& state)
{
    if (state.cameraId.empty() && state.lidarId.empty())
        return;

    auto dimmed = [](const std::string& text)
    {
        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
        ImGui::TextWrapped("%s", text.c_str());
        ImGui::PopStyleColor();
    };

    if (!state.cameraId.serial.empty())
        ImGui::TextWrapped("Camera: %s (%s)", state.cameraId.serial.c_str(), state.cameraId.model.c_str());
    else if (!state.cameraId.frameId.empty())
        dimmed("Camera: (file named no serial)");
    if (!state.cameraId.frameId.empty())
        dimmed("    frame " + state.cameraId.frameId);

    if (!state.lidarId.empty())
        ImGui::TextWrapped("LiDAR:  %s", state.lidarId.c_str());

    ImGui::Separator();
}

// ── Main draw ────────────────────────────────────────────────────────────────
void UI::draw(AppState& state)
{
    // Set again below while a Remove button is hovered; the 3D view, drawn
    // before the UI, shows the previous frame's.
    state.hoveredRemoveIndex = -1;
    panelMenuBar(state);
    handleShortcuts(state);

    ImGuiIO& io = ImGui::GetIO();
    float menuBarH = ImGui::GetFrameHeight();
    float panelH = (float)GetScreenHeight() - menuBarH;

    // Pinned to the right edge at full height; dragging its left edge resizes
    // it, between kMinPanelW and leaving the 3D view kMinViewW.
    constexpr float kMinPanelW = 260.f, kMinViewW = 400.f;
    const float maxPanelW = std::max(kMinPanelW, io.DisplaySize.x - kMinViewW);
    state.panelW = std::clamp(state.panelW, kMinPanelW, maxPanelW);
    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - state.panelW, menuBarH), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(state.panelW, panelH), ImGuiCond_Always);
    ImGui::SetNextWindowSizeConstraints(ImVec2(kMinPanelW, panelH), ImVec2(maxPanelW, panelH));
    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse);
    state.panelW = ImGui::GetWindowWidth();

    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.f, 1.f), "LiDAR-Camera Calibration");
    ImGui::Separator();

    drawSensorIds(state);

    // Alt/Super = toggle Camera RGB ↔ Intensity (works anywhere in the window).
    // Super alongside Alt for macOS, where Option is awkward to use as a
    // modifier (it composes special characters); ImGui swaps Cmd and Ctrl
    // there, so Super is the physical Ctrl key and Cmd stays free for
    // shortcuts. Fires on release, unless Shift joined in: Shift+Alt is the
    // Image View's LiDAR-pick chord.
    const bool altDown = ImGui::IsKeyDown(ImGuiKey_LeftAlt) || ImGui::IsKeyDown(ImGuiKey_RightAlt) ||
        ImGui::IsKeyDown(ImGuiKey_LeftSuper) || ImGui::IsKeyDown(ImGuiKey_RightSuper);
    if (altDown && io.KeyShift)
        altChorded = true;
    if (altWasDown && !altDown)
    {
        if (!altChorded)
        {
            auto& cm = state.vizParams.colorMode;
            if (cm == 3)
                cm = 1; // RGB → Intensity
            else
                cm = 3; // anything → RGB
        }
        altChorded = false;
    }
    altWasDown = altDown;

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
    if (ImGui::CollapsingHeader("Correspondences", ImGuiTreeNodeFlags_DefaultOpen))
        panelCorrespondences(state);

    ImGui::End();

    // ── Image view window (pan + zoom) ────────────────────────────────────
    // A normal floating window (title bar, movable, resizable) that overlaps
    // the 3D view, which now fills the whole background instead of being
    // confined to half the screen -- simpler than the old fixed-region
    // split-screen layout. ImGuiCond_FirstUseEver only sets this starting
    // pos/size the very first time; the user's own placement afterward
    // persists.
    if (state.renderer.imageTexValid)
    {
        float viewW = io.DisplaySize.x - state.panelW;
        float viewH = io.DisplaySize.y - menuBarH;
        ImGui::SetNextWindowPos(ImVec2(0, menuBarH), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(viewW * 0.6f, viewH * 0.6f), ImGuiCond_FirstUseEver);
        ImGui::Begin("Image View");
        drawImageView(state);
        ImGui::End();
    }
}

// ── Image view: pan + zoom ────────────────────────────────────────────────────
// Pan and zoom are both handled by letting ImGui do the work instead of a
// hand-rolled offX/offY/scale coordinate system: the image is displayed at
// (imgW*imgZoom, imgH*imgZoom) inside a plain scrollable child region, so
// dragging just moves the child's own scroll offset (SetScrollX/Y, which
// ImGui clamps into range for us every frame) and zoom is a bare size
// multiplier (matching apps/manual_color's PageUp/PageDown zoom -- not
// anchored to the cursor, kept simple on purpose). This removes an entire
// class of bugs the previous custom-coordinate version had (invalid
// offX/offY silently desyncing the sampled crop from where markers were
// drawn) by construction: there is no separate "crop rectangle" to keep in
// sync any more, ImGui::GetItemRectMin() after drawing the (full,
// un-cropped) image *is* the single source of truth every other coordinate
// in this function is built from.
void UI::drawImageView(AppState& state)
{
    const float imgW = (float)state.imageW;
    const float imgH = (float)state.imageH;
    if (imgW <= 0 || imgH <= 0)
        return;

    // Reset zoom when a different image is loaded
    if (state.imageW != lastImgW || state.imageH != lastImgH)
    {
        lastImgW = state.imageW;
        lastImgH = state.imageH;
        imgZoom = 1.f;
    }

    // Shift is the modifier that dedicates the mouse to picking, mirroring
    // App.cpp's 3D-view picking and the CTRL-to-pick convention
    // core/src/control_points.cpp already uses elsewhere in this codebase.
    // Adding Alt picks the LiDAR point instead, in the intensity image
    // App::draw renders here meanwhile.
    const bool picking = ImGui::GetIO().KeyShift;
    const bool pickingLidar = picking && ImGui::GetIO().KeyAlt;
    ImGui::TextColored(
        ImVec4(1, 1, 0, 0.8f),
        pickingLidar  ? "LiDAR intensity image: Shift+" SHORTCUT_ALT "+click to pick a LiDAR point"
            : picking ? "Shift+click to pick a pixel | add " SHORTCUT_ALT " to pick a LiDAR point"
                      : "Scroll: zoom | Drag: pan with middle | Dbl-click: reset zoom");

    ImGui::BeginChild("image_scroll", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

    ImVec2 dispSize(imgW * imgZoom, imgH * imgZoom);
    ImGui::Image((ImTextureID)state.renderer.imageTex.texture.id, dispSize, ImVec2(0, 1), ImVec2(1, 0));
    ImVec2 imgMin = ImGui::GetItemRectMin(); // screen pos of the image's top-left, already scroll-adjusted
    bool hovered = ImGui::IsItemHovered();
    ImGuiIO& io = ImGui::GetIO();

    // Image pixel <-> screen coordinate conversion, shared by the click
    // handling below, the magnifier loupe, and the correspondence-pair
    // markers drawn further down.
    auto screenToImg = [&](ImVec2 s)
    {
        return ImVec2((s.x - imgMin.x) / imgZoom, (s.y - imgMin.y) / imgZoom);
    };
    auto imgToScreen = [&](ImVec2 p)
    {
        return ImVec2(imgMin.x + p.x * imgZoom, imgMin.y + p.y * imgZoom);
    };

    if (hovered)
    {
        if (io.MouseWheel != 0.f)
            imgZoom = std::clamp(imgZoom * std::exp(io.MouseWheel * 0.15f), 0.05f, 20.f);

        if (picking)
        {
            // LMB is dedicated to picking here -- dragging must not pan.
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
            {
                ImVec2 p = screenToImg(io.MousePos);
                if (pickingLidar)
                    state.pickLidarPointAtPixel(p.x, p.y);
                else
                    state.setPendingImagePoint(std::clamp(p.x, 0.f, imgW), std::clamp(p.y, 0.f, imgH));
            }
        }
        else if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle))
        {
            ImGui::SetScrollX(ImGui::GetScrollX() - io.MouseDelta.x);
            ImGui::SetScrollY(ImGui::GetScrollY() - io.MouseDelta.y);
        }

        if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
            imgZoom = 1.f;
    }

    // ── magnifier loupe (precision aid while picking an image pixel) ────────
    // Tooltip-based, mirroring apps/manual_color's draw_zoom_pick_point
    // (auto-follows the cursor, crosshair at its center).
    if (picking && hovered)
    {
        ImGui::BeginTooltip();

        const float regionSz = 24.f; // crop size, in image pixels
        const float loupeZoom = 6.f;

        ImVec2 p = screenToImg(io.MousePos);
        float srcX = std::clamp(p.x - regionSz * 0.5f, 0.f, std::max(0.f, imgW - regionSz));
        float srcY = std::clamp(p.y - regionSz * 0.5f, 0.f, std::max(0.f, imgH - regionSz));
        ImVec2 uv0(srcX / imgW, 1.f - srcY / imgH);
        ImVec2 uv1((srcX + regionSz) / imgW, 1.f - (srcY + regionSz) / imgH);

        ImGui::Image((ImTextureID)state.renderer.imageTex.texture.id, ImVec2(regionSz * loupeZoom, regionSz * loupeZoom), uv0, uv1);

        ImVec2 lMin = ImGui::GetItemRectMin();
        ImVec2 lMax = ImGui::GetItemRectMax();
        ImVec2 c((lMin.x + lMax.x) * 0.5f, (lMin.y + lMax.y) * 0.5f);
        ImGui::GetForegroundDrawList()->AddLine(ImVec2(c.x - 10, c.y), ImVec2(c.x + 10, c.y), IM_COL32(0, 255, 0, 220), 1.5f);
        ImGui::GetForegroundDrawList()->AddLine(ImVec2(c.x, c.y - 10), ImVec2(c.x, c.y + 10), IM_COL32(0, 255, 0, 220), 1.5f);

        ImGui::EndTooltip();
    }

    // ── correspondence-pair markers ─────────────────────────────────────────
    // No manual "is this on screen" check needed -- the child region clips
    // its own draw list, so markers scrolled out of view are simply cut off.
    {
        ImDrawList* dl = ImGui::GetWindowDrawList();
        for (size_t i = 0; i < state.pairs.size(); i++)
        {
            ImVec2 s = imgToScreen(ImVec2(state.pairs[i].u, state.pairs[i].v));
            dl->AddCircle(s, 6.f, IM_COL32(255, 255, 0, 255), 0, 2.f);
            dl->AddText(ImVec2(s.x + 8, s.y - 8), IM_COL32(255, 255, 0, 255), std::to_string(i).c_str());
        }
        if (state.pendingHasImage)
        {
            ImVec2 s = imgToScreen(ImVec2(state.pendingPair.u, state.pendingPair.v));
            dl->AddCircle(s, 6.f, IM_COL32(255, 140, 0, 255), 0, 2.f);
        }
        // Large cross for the pair selected in the Correspondences panel --
        // matches the magenta cross drawn for it in the 3D view -- and a red
        // one, ringed, for the pair whose Remove button is hovered.
        auto pairCross = [&](int index, ImU32 color)
        {
            if (index < 0 || index >= (int)state.pairs.size())
                return ImVec2(0, 0);
            ImVec2 s = imgToScreen(ImVec2(state.pairs[index].u, state.pairs[index].v));
            dl->AddLine(ImVec2(s.x - 18, s.y), ImVec2(s.x + 18, s.y), color, 2.f);
            dl->AddLine(ImVec2(s.x, s.y - 18), ImVec2(s.x, s.y + 18), color, 2.f);
            return s;
        };
        pairCross(state.selectedPairIndex, IM_COL32(255, 0, 255, 255));
        if (state.hoveredRemoveIndex >= 0 && state.hoveredRemoveIndex < (int)state.pairs.size())
            dl->AddCircle(pairCross(state.hoveredRemoveIndex, IM_COL32(255, 40, 40, 255)), 12.f, IM_COL32(255, 40, 40, 255), 0, 2.f);

        // In the intensity image, also the LiDAR side of each pair,
        // projected through the current calibration (cyan squares).
        if (pickingLidar)
        {
            const Intrinsics K = state.displayIntrinsics();
            const Eigen::Matrix3f R_wc = omFiKaToMat3(state.extrinsics.om, state.extrinsics.fi, state.extrinsics.ka);
            const Eigen::Vector3f C(state.extrinsics.tx, state.extrinsics.ty, state.extrinsics.tz);
            auto lidarMarker = [&](float x, float y, float z, ImU32 color, const char* label)
            {
                float u, v, depth;
                if (!projectPoint(x, y, z, K, R_wc, C, u, v, depth))
                    return;
                ImVec2 s = imgToScreen(ImVec2(u, v));
                dl->AddRect(ImVec2(s.x - 6, s.y - 6), ImVec2(s.x + 6, s.y + 6), color, 0.f, 0, 2.f);
                if (label)
                    dl->AddText(ImVec2(s.x + 8, s.y - 8), color, label);
            };
            for (size_t i = 0; i < state.pairs.size(); i++)
                lidarMarker(state.pairs[i].px, state.pairs[i].py, state.pairs[i].pz, IM_COL32(0, 255, 255, 255), std::to_string(i).c_str());
            if (state.pendingHasCloud)
                lidarMarker(state.pendingPair.px, state.pendingPair.py, state.pendingPair.pz, IM_COL32(255, 140, 0, 255), nullptr);
        }
    }

    ImGui::EndChild();
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
    std::string path = mandeye::fd::SaveFileDialog("Save calibration file", mandeye::fd::json_filter);
    if (!path.empty())
    {
        setBuf(savePath, sizeof(savePath), path);
        state.saveCalibration(savePath);
    }
}

static const std::vector<std::string> kPairsFilter = { "Picked pairs (*.json)", "*.json", "All files", "*" };

void UI::actionLoadPairs(AppState& state)
{
    std::string path = mandeye::fd::OpenFileDialogOneFile("Load picked pairs", kPairsFilter);
    if (!path.empty())
        state.loadPairs(path.c_str());
}

void UI::actionSavePairs(AppState& state)
{
    if (!state.pairsPath.empty())
        state.savePairs(state.pairsPath.c_str());
    else if (state.pairs.empty())
        state.statusMsg = "No pairs to save";
    else
        actionSavePairsAs(state);
}

void UI::actionSavePairsAs(AppState& state)
{
    std::string path = mandeye::fd::SaveFileDialog("Save picked pairs", kPairsFilter, ".json", "pairs.json");
    if (!path.empty())
        state.savePairs(path.c_str());
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
    if (io.KeyShift && ctrlPressed(ImGuiKey_S))
        actionSaveCalibration(state);
    else if (ctrlPressed(ImGuiKey_S))
        actionSavePairs(state);

    if (ImGui::IsKeyPressed(ImGuiKey_C, false))
        state.showCompassRuler = !state.showCompassRuler;
    if (ImGui::IsKeyPressed(ImGuiKey_F, false))
        state.focusOnSensors();
}

void UI::panelMenuBar(AppState& state)
{
    if (!ImGui::BeginMainMenuBar())
        return;

    if (ImGui::BeginMenu("File"))
    {
        if (ImGui::MenuItem("Open Image...", SHORTCUT_CTRL "+I"))
            actionOpenImage(state);
        if (ImGui::MenuItem("Open Point Cloud...", SHORTCUT_CTRL "+O"))
            actionOpenPointCloud(state);
        if (ImGui::MenuItem("Add Point Cloud...", SHORTCUT_CTRL "+Shift+O"))
            actionAddPointCloud(state);
        ImGui::Separator();
        if (ImGui::MenuItem("Open Intrinsics...", SHORTCUT_CTRL "+K"))
            actionOpenIntrinsics(state);
        ImGui::Separator();
        if (ImGui::MenuItem("Open Calibration...", SHORTCUT_CTRL "+L"))
            actionOpenCalibration(state);
        if (ImGui::MenuItem("Save Calibration...", SHORTCUT_CTRL "+Shift+S"))
            actionSaveCalibration(state);
        ImGui::Separator();
        if (ImGui::MenuItem("Load Pairs..."))
            actionLoadPairs(state);
        if (ImGui::MenuItem("Save Pairs", SHORTCUT_CTRL "+S", false, !state.pairs.empty() || !state.pairsPath.empty()))
            actionSavePairs(state);
        if (ImGui::MenuItem("Save Pairs As...", nullptr, false, !state.pairs.empty()))
            actionSavePairsAs(state);
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("View"))
    {
        // GPU-side point subsampling (draws only every Nth point) for
        // large clouds -- matches camera_lidar_trajectory_viewer's "Draw
        // decimation" slider.
        ImGui::SetNextItemWidth(140.f);
        ImGui::SliderInt("Draw decimation", &state.vizParams.drawDecim, 1, 64);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Renders only every Nth point -- raise this if the 3D view is slow with a large cloud.");
        if (ImGui::MenuItem("Focus on sensors", "F"))
            state.focusOnSensors();
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

    static const CameraModel kModels[] = { CameraModel::Pinhole, CameraModel::Mei, CameraModel::Fisheye };
    static const char* kModelNames[] = { "Pinhole", "Mei", "Fisheye (equidistant)" };
    static_assert(IM_ARRAYSIZE(kModels) == IM_ARRAYSIZE(kModelNames));
    int modelIdx = 0;
    for (int i = 0; i < IM_ARRAYSIZE(kModels); ++i)
        if (K.model == kModels[i])
            modelIdx = i;
    if (ImGui::Combo("Model", &modelIdx, kModelNames, IM_ARRAYSIZE(kModelNames)))
    {
        K.model = kModels[modelIdx];
        edited = true;
    }
    ImGui::Separator();

    drag("fx", &K.fx, 1.f, 1.f, 10000.f, "%.1f");
    drag("fy", &K.fy, 1.f, 1.f, 10000.f, "%.1f");
    drag("cx", &K.cx, 0.5f, 0.f, 10000.f, "%.1f");
    drag("cy", &K.cy, 0.5f, 0.f, 10000.f, "%.1f");
    ImGui::Separator();

    if (K.model == CameraModel::Mei)
    {
        // Unified-sphere fisheye (see calib::projectPoint): xi + a plain k1/k2/k3 +
        // p1/p2 polynomial, no rational denominator -- k4/k5/k6 don't apply
        // here, so they're hidden instead of shown as dead controls.
        drag("xi", &K.xi, 0.001f, 0.f, 3.f, "%.4f");
        ImGui::Text("Radial (Mei polynomial):");
        drag("k1", &K.k1, 0.001f, -100.f, 100.f, "%.4f");
        drag("k2", &K.k2, 0.001f, -100.f, 100.f, "%.4f");
        drag("k3", &K.k3, 0.001f, -100.f, 100.f, "%.4f");
        ImGui::Text("Tangential:");
        drag("p1", &K.p1, 0.0001f, -1.f, 1.f, "%.5f");
        drag("p2", &K.p2, 0.0001f, -1.f, 1.f, "%.5f");
        helpMarker(
            "Drag to adjust. Hold " SHORTCUT_CTRL "+click to type a value.\nUnlike Pinhole, the displayed image is never undistorted for "
            "Mei -- the projection overlay and Camera RGB coloring apply this distortion to the raw image directly.");

        ImGui::Text("Refine with \"Solve Extrinsics from Pairs\":");
        // Own ID scope: "xi" is also the drag field's label above.
        ImGui::PushID("refine");
        ImGui::Checkbox("fx, fy", &state.refineIntrinsics.focal);
        ImGui::SameLine();
        ImGui::Checkbox("cx, cy", &state.refineIntrinsics.principalPoint);
        ImGui::SameLine();
        ImGui::Checkbox("xi", &state.refineIntrinsics.xi);
        ImGui::Checkbox("k1-k3, p1, p2", &state.refineIntrinsics.distortion);
        ImGui::PopID();
        helpMarker(
            "Solved together with the extrinsics; the values above are replaced.\n"
            "Each pair gives 2 equations. xi trades off against fx/fy, and a few\n"
            "pairs barely pin the distortion -- refine only what the pairs support.\n"
            "Reload the calibration to undo.");
    }
    else if (K.model == CameraModel::Fisheye)
    {
        // OpenCV cv::fisheye (see calib::projectPoint): k1..k4 act on the
        // incidence angle, with no tangential terms and no k5/k6.
        ImGui::Text("Radial (theta polynomial):");
        drag("k1", &K.k1, 0.001f, -100.f, 100.f, "%.4f");
        drag("k2", &K.k2, 0.001f, -100.f, 100.f, "%.4f");
        drag("k3", &K.k3, 0.001f, -100.f, 100.f, "%.4f");
        drag("k4", &K.k4, 0.001f, -100.f, 100.f, "%.4f");
        helpMarker(
            "Drag to adjust. Hold " SHORTCUT_CTRL "+click to type a value.\nUnlike Pinhole, the displayed image is never undistorted for "
            "Fisheye -- the projection overlay and Camera RGB coloring apply this distortion to the raw image directly.");
    }
    else
    {
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
        helpMarker("Drag to adjust. Hold " SHORTCUT_CTRL "+click to type a value.");
    }
    ImGui::PopItemWidth();

    if (edited && state.intrinsicsLoaded)
        state.rebuildImageTexture();
}

// ── Extrinsics ────────────────────────────────────────────────────────────────
void UI::panelExtrinsics(AppState& state)
{
    Extrinsics& E = state.extrinsics;

    ImGui::PushItemWidth(-80.f);

    ImGui::Checkbox("Lock translation", &state.lockTranslation);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(
            "Blocks tx/ty/tz from being edited here or changed by\n\"Solve Extrinsics from Pairs\" -- use when the camera\nposition is "
            "already known and only orientation needs solving.");
    ImGui::SameLine();
    ImGui::Checkbox("Lock rotation", &state.lockRotation);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(
            "Blocks om/fi/ka from being edited here or changed by\n\"Solve Extrinsics from Pairs\" -- use when the camera\norientation is "
            "already known and only position needs solving.");

    ImGui::Text("Camera position in world (m):");
    ImGui::BeginDisabled(state.lockTranslation);
    dragFloat("tx", &E.tx, 0.01f, -50.f, 50.f, "%.3f");
    dragFloat("ty", &E.ty, 0.01f, -50.f, 50.f, "%.3f");
    dragFloat("tz", &E.tz, 0.01f, -50.f, 50.f, "%.3f");
    ImGui::EndDisabled();

    ImGui::Spacing();
    ImGui::Text("Camera orientation in world, om/fi/ka (deg):");
    ImGui::BeginDisabled(state.lockRotation);
    dragFloat("om", &E.om, 0.1f, -180.f, 180.f, "%.2f");
    dragFloat("fi", &E.fi, 0.1f, -180.f, 180.f, "%.2f");
    dragFloat("ka", &E.ka, 0.1f, -180.f, 180.f, "%.2f");
    ImGui::EndDisabled();
    helpMarker(
        "R_wc = Rx(om)*Ry(fi)*Rz(ka): camera orientation in LiDAR world.\nT_lidar2cam = R_wc^T * (p - C).\nAt fi=+/-90 deg (gimbal lock), "
        "om and ka are not individually\nunique -- only om+ka (or om-ka) is determined.");

    ImGui::Spacing();
    ImGui::BeginDisabled(state.lockTranslation && state.lockRotation);
    if (ImGui::Button("Reset Extrinsics", ImVec2(-1, 0)))
    {
        // Respect the locks: reset only the unlocked half.
        const Extrinsics defaults;
        if (!state.lockTranslation)
        {
            E.tx = defaults.tx;
            E.ty = defaults.ty;
            E.tz = defaults.tz;
        }
        if (!state.lockRotation)
        {
            E.om = defaults.om;
            E.fi = defaults.fi;
            E.ka = defaults.ka;
        }
    }
    ImGui::EndDisabled();
    ImGui::PopItemWidth();

    if (ImGui::TreeNode("Priors (soft constraints)"))
    {
        helpMarker(
            "Each enabled prior adds an observation equation\n(value - target) / sigma to \"Solve Extrinsics from Pairs\".\n"
            "A parameter sigma away from its target costs as much\nas 1 px of reprojection error, so it stays near the\n"
            "target unless the pairs clearly say otherwise.\nUse it for directions the pairs barely constrain,\n"
            "e.g. the camera height tz. Ignored while locked.");
        static const char* const kNames[6] = { "tx", "ty", "tz", "om", "fi", "ka" };
        float* const current[6] = { &E.tx, &E.ty, &E.tz, &E.om, &E.fi, &E.ka };
        if (ImGui::BeginTable("priors", 4, ImGuiTableFlags_SizingStretchProp))
        {
            ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("target");
            ImGui::TableSetupColumn("sigma");
            ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableHeadersRow();
            for (int i = 0; i < 6; ++i)
            {
                const bool isTranslation = i < 3;
                calib::ParameterPrior& prior = state.priors[i];
                ImGui::PushID(i);
                ImGui::BeginDisabled(isTranslation ? state.lockTranslation : state.lockRotation);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                // Enabling a prior holds the parameter where it is now.
                if (ImGui::Checkbox(kNames[i], &prior.enabled) && prior.enabled)
                    prior.target = *current[i];
                ImGui::BeginDisabled(!prior.enabled);
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(-FLT_MIN);
                float target = static_cast<float>(prior.target);
                if (dragFloat("##target", &target, isTranslation ? 0.01f : 0.1f, -180.f, 180.f, isTranslation ? "%.3f m" : "%.2f deg"))
                    prior.target = target;
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(-FLT_MIN);
                float sigma = static_cast<float>(prior.sigma);
                if (dragFloat(
                        "##sigma",
                        &sigma,
                        isTranslation ? 0.001f : 0.01f,
                        isTranslation ? 0.0001f : 0.001f,
                        isTranslation ? 10.f : 90.f,
                        isTranslation ? "%.4f m" : "%.3f deg"))
                    prior.sigma = std::max(sigma, isTranslation ? 0.0001f : 0.001f);
                ImGui::TableNextColumn();
                if (ImGui::SmallButton("= current"))
                    prior.target = *current[i];
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Set the target to the current %s", kNames[i]);
                ImGui::EndDisabled();
                ImGui::EndDisabled();
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        ImGui::TreePop();
    }

    // Show current rotation matrix
    if (ImGui::TreeNode("Rotation matrix"))
    {
        Eigen::Matrix3f R = omFiKaToMat3(E.om, E.fi, E.ka);
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
    ImGui::Checkbox("Lidar model", &state.showLidarModel);
    ImGui::SameLine();
    ImGui::Checkbox("Camera model", &state.showCameraModel);
    helpMarker(
        "Simplified, real-size Livox Mid-360 at the LiDAR origin (connector at the back, -x) and\n"
        "the camera body below at the camera pose. The yellow ring marks the lens these extrinsics describe.\n"
        "Focus on sensors (F) zooms in on them.");

    // Order matches CameraBody's enumerators.
    const char* bodies[] = { "Insta360 X5", "PCB camera" };
    int body = static_cast<int>(state.cameraBody);
    ImGui::BeginDisabled(!state.showCameraModel);
    ImGui::PushItemWidth(-140.f);
    if (ImGui::Combo("Camera body", &body, bodies, IM_ARRAYSIZE(bodies)))
        state.cameraBody = static_cast<CameraBody>(body);
    ImGui::PopItemWidth();
    ImGui::EndDisabled();
    helpMarker(
        "PCB camera: a bare board in the Raspberry Pi camera format (25 x 24 mm, e.g. an IMX519\n"
        "module), ribbon connector at the bottom of the image.");

    if (ImGui::Button("Focus on sensors (F)"))
        state.focusOnSensors();
}

// ── Correspondences ──────────────────────────────────────────────────────────
// Pick image-pixel <-> LiDAR-point pairs, then solve extrinsics from them.
// Mirrors the list-with-remove-buttons + ">=3 needed" pattern used by
// ControlPoints::imgui (core/src/control_points.cpp).
void UI::panelCorrespondences(AppState& state)
{
    ImGui::TextWrapped(
        "Shift+click the pixel in the Image View, and the LiDAR point in the 3D View -- or Shift+" SHORTCUT_ALT
        "+click it in the Image View, "
        "which shows the LiDAR intensity image while both are held. Picking both sides completes a pair.");

    ImGui::Text("Pending pair:");
    ImGui::SameLine();
    ImGui::TextColored(state.pendingHasImage ? ImVec4(0, 1, 0, 1) : ImVec4(0.6f, 0.6f, 0.6f, 1), "image");
    ImGui::SameLine();
    ImGui::TextColored(state.pendingHasCloud ? ImVec4(0, 1, 0, 1) : ImVec4(0.6f, 0.6f, 0.6f, 1), "3D point");
    if (state.pendingHasImage || state.pendingHasCloud)
    {
        ImGui::SameLine();
        if (ImGui::SmallButton("Clear"))
            state.clearPending();
    }

    ImGui::Spacing();
    ImGui::Separator();

    // R_wc/C for a live per-pair reprojection-error readout, using this
    // app's own camera model (calib::projectPoint) -- so the numbers reflect
    // whatever the current extrinsics are, whether from a solve or manual
    // slider edits.
    Eigen::Matrix3f R = omFiKaToMat3(state.extrinsics.om, state.extrinsics.fi, state.extrinsics.ka);
    Eigen::Vector3f C(state.extrinsics.tx, state.extrinsics.ty, state.extrinsics.tz);

    int removeIndex = -1;
    for (int i = 0; i < (int)state.pairs.size(); i++)
    {
        const auto& p = state.pairs[i];
        ImGui::PushID(i);
        char label[64];
        std::snprintf(label, sizeof(label), "#%d px(%.0f,%.0f)", i, p.u, p.v);
        // Click a row to highlight that pair as a large cross in the 3D
        // and image views (click again to clear the selection).
        if (ImGui::Selectable(label, state.selectedPairIndex == i, 0, ImVec2(140, 0)))
            state.selectedPairIndex = (state.selectedPairIndex == i) ? -1 : i;
        ImGui::SameLine();

        float u, v, depth;
        if (projectPoint(p.px, p.py, p.pz, state.intrinsics, R, C, u, v, depth))
        {
            float err = std::sqrt((u - p.u) * (u - p.u) + (v - p.v) * (v - p.v));
            ImGui::TextColored(err < 5.f ? ImVec4(0, 1, 0, 1) : ImVec4(1, 0.4f, 0, 1), "err %.1fpx", err);
        }
        else
        {
            ImGui::TextColored(ImVec4(1, 0, 0, 1), "behind camera");
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Remove"))
            removeIndex = i;
        if (ImGui::IsItemHovered())
            state.hoveredRemoveIndex = i;
        ImGui::PopID();
    }
    if (removeIndex >= 0)
        state.removePair(removeIndex);

    ImGui::BeginDisabled(state.pairs.empty() && state.pairsPath.empty());
    if (ImGui::Button("Save pairs"))
        actionSavePairs(state);
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
    {
        if (state.pairsPath.empty())
            ImGui::SetTooltip(SHORTCUT_CTRL "+S; asks for a file");
        else
            ImGui::SetTooltip(SHORTCUT_CTRL "+S, to %s", state.pairsPath.c_str());
    }
    ImGui::SameLine();
    if (ImGui::Button("Load pairs..."))
        actionLoadPairs(state);
    helpMarker(
        "Or drop a saved pairs .json on the window, or pass --pairs <file.json>.\n"
        "Loading replaces the current pairs. " SHORTCUT_CTRL "+S saves to the last pairs file\n"
        "loaded or saved, or the --pairs one (created if it does not exist yet).\n"
        "Pixels are of the shown image (rectified or raw) -- load the same image and calibration first.");
    if (!state.pairsPath.empty())
        ImGui::TextDisabled("Pairs file: %s", std::filesystem::path(state.pairsPath).filename().string().c_str());

    ImGui::Spacing();
    const std::string refined = state.refinedIntrinsicsLabel();
    const bool nothingFree = state.lockTranslation && state.lockRotation && refined.empty();
    ImGui::BeginDisabled(state.pairs.size() < 3 || nothingFree);
    if (ImGui::Button("Solve Extrinsics from Pairs", ImVec2(-1, 0)))
        state.solvePairs();
    ImGui::EndDisabled();
    if (state.pairs.size() < 3)
        ImGui::TextDisabled("At least 3 pairs needed");
    else if (nothingFree)
        ImGui::TextDisabled("Translation and rotation are both locked");
    if (!refined.empty())
        ImGui::TextDisabled("Also refines %s", refined.c_str());
    if (state.lastSolveRmsPixels >= 0.0)
        ImGui::Text("Last solve RMS reprojection error: %.2f px", state.lastSolveRmsPixels);
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

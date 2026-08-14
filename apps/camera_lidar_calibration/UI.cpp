#include "UI.h"
#include "App.h"
#include "imgui.h"
#include "rlImGui.h"
#include <Core/pfd_wrapper.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
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
        float viewW = io.DisplaySize.x - panelW;
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
    const bool picking = ImGui::GetIO().KeyShift;
    ImGui::TextColored(
        ImVec4(1, 1, 0, 0.8f), picking ? "Shift+click to pick a point" : "Scroll: zoom | Drag: pan | Dbl-click: reset zoom");

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
                state.setPendingImagePoint(std::clamp(p.x, 0.f, imgW), std::clamp(p.y, 0.f, imgH));
            }
        }
        else if (ImGui::IsMouseDragging(ImGuiMouseButton_Left))
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
        // matches the magenta cross drawn for it in the 3D view.
        if (state.selectedPairIndex >= 0 && state.selectedPairIndex < (int)state.pairs.size())
        {
            ImVec2 s = imgToScreen(ImVec2(state.pairs[state.selectedPairIndex].u, state.pairs[state.selectedPairIndex].v));
            dl->AddLine(ImVec2(s.x - 18, s.y), ImVec2(s.x + 18, s.y), IM_COL32(255, 0, 255, 255), 2.f);
            dl->AddLine(ImVec2(s.x, s.y - 18), ImVec2(s.x, s.y + 18), IM_COL32(255, 0, 255, 255), 2.f);
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

    if (ImGui::BeginMenu("View"))
    {
        // GPU-side point subsampling (draws only every Nth point) for
        // large clouds -- matches camera_lidar_trajectory_viewer's "Draw
        // decimation" slider.
        ImGui::SetNextItemWidth(140.f);
        ImGui::SliderInt("Draw decimation", &state.vizParams.drawDecim, 1, 64);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Renders only every Nth point -- raise this if the 3D view is slow with a large cloud.");
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

    ImGui::Checkbox("Lock translation", &state.lockTranslation);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(
            "Blocks tx/ty/tz from being edited here or changed by\n\"Solve Extrinsics from Pairs\" -- use when the camera\nposition is already known and only orientation needs solving.");

    ImGui::Text("Camera position in world (m):");
    ImGui::BeginDisabled(state.lockTranslation);
    dragFloat("tx", &E.tx, 0.01f, -50.f, 50.f, "%.3f");
    dragFloat("ty", &E.ty, 0.01f, -50.f, 50.f, "%.3f");
    dragFloat("tz", &E.tz, 0.01f, -50.f, 50.f, "%.3f");
    ImGui::EndDisabled();

    ImGui::Spacing();
    ImGui::Text("Camera orientation in world, om/fi/ka (deg):");
    dragFloat("om", &E.om, 0.1f, -180.f, 180.f, "%.2f");
    dragFloat("fi", &E.fi, 0.1f, -180.f, 180.f, "%.2f");
    avoidGimbalLock(E.fi);
    dragFloat("ka", &E.ka, 0.1f, -180.f, 180.f, "%.2f");
    helpMarker(
        "R_wc = Rx(om)*Ry(fi)*Rz(ka): camera orientation in LiDAR world.\nT_lidar2cam = R_wc^T * (p - C).\nAt fi=+/-90 deg (gimbal lock), om and ka are not individually\nunique -- only om+ka (or om-ka) is determined.");

    ImGui::Spacing();
    if (ImGui::Button("Reset Extrinsics", ImVec2(-1, 0)))
    {
        // Respect the translation lock: only reset orientation while locked.
        Extrinsics defaults;
        if (state.lockTranslation)
        {
            E.om = defaults.om;
            E.fi = defaults.fi;
            E.ka = defaults.ka;
        }
        else
        {
            E = defaults;
        }
    }
    ImGui::PopItemWidth();

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
}

// ── Correspondences ──────────────────────────────────────────────────────────
// Pick image-pixel <-> LiDAR-point pairs, then solve extrinsics from them.
// Mirrors the list-with-remove-buttons + ">=3 needed" pattern used by
// ControlPoints::imgui (core/src/control_points.cpp).
void UI::panelCorrespondences(AppState& state)
{
    ImGui::TextWrapped("Hold Shift and click a point in the Image View or the 3D View. Picking both sides completes a pair.");

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
        ImGui::PopID();
    }
    if (removeIndex >= 0)
        state.removePair(removeIndex);

    ImGui::Spacing();
    ImGui::BeginDisabled(state.pairs.size() < 3);
    if (ImGui::Button("Solve Extrinsics from Pairs", ImVec2(-1, 0)))
        state.solvePairs();
    ImGui::EndDisabled();
    if (state.pairs.size() < 3)
        ImGui::TextDisabled("At least 3 pairs needed");
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

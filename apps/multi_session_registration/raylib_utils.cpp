#include "raylib_utils.h"

#include "raymath.h"
#include "rlImGui.h"

#include <imgui_internal.h>

#include <RaylibWidgets/CenterOfRotationWindow.h>
#include <RaylibWidgets/CompassRuler.h>
#include <RaylibWidgets/PointPicking.h>
#include <RaylibWidgets/RayPlaneD.h>
#include <RaylibWidgets/WindowFit.h>

#include <HDMapping/Version.hpp>

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <memory>
#include <random>

raylib_widgets::OrbitCamera camera;

// GLUT step 3 used 1000 (every point was a glVertex call); GPU buffers draw full density, as in step 2.
int viewer_decimate_point_cloud = 2;
int mouse_old_x = 0, mouse_old_y = 0;
int mouse_buttons = 0;
bool& is_ortho = camera.isOrtho;
bool& lock_z = camera.lockZ;
bool show_axes = true;
ImVec4 bg_color = ImVec4(0.65f, 0.65f, 0.65f, 1.00f);
int point_size = 1;
bool info_gui = false;
bool compass_ruler = true;
bool cor_gui = false;
Eigen::Affine3f viewLocal = Eigen::Affine3f::Identity();

Eigen::Map<Eigen::Vector3f> rotation_center(&camera.euler.rotationCenter.x);
float& rotate_x = camera.euler.rotateX;
float& rotate_y = camera.euler.rotateY;
float& translate_x = camera.euler.translate.x;
float& translate_y = camera.euler.translate.y;
float& translate_z = camera.euler.translate.z;
Eigen::Map<Eigen::Vector3f> new_rotation_center(&camera.eulerGoal.rotationCenter.x);
float& new_rotate_x = camera.eulerGoal.rotateX;
float& new_rotate_y = camera.eulerGoal.rotateY;
float& new_translate_x = camera.eulerGoal.translate.x;
float& new_translate_y = camera.eulerGoal.translate.y;
float& new_translate_z = camera.eulerGoal.translate.z;
bool& camera_transition_active = camera.eulerTransitionActive;
float* const m_ortho_projection = camera.orthoProjection;
float* const m_ortho_gizmo_view = camera.orthoGizmoView;

namespace
{
    void (*display_cb)() = nullptr;
    void (*mouse_cb)(int, int, int, int) = nullptr;

    Matrix frame_mvp{};
    float current_color[3] = { 1.f, 1.f, 1.f };

    struct StripVertex
    {
        Vector3 p;
        float c[3];
    };
    std::vector<StripVertex> strip;

    struct Label
    {
        Vector3 p;
        std::string text;
        Color color;
    };
    Vector3 label_pos{};
    float label_color[3] = { 1.f, 1.f, 1.f };
    std::vector<Label> labels;

    std::vector<std::unique_ptr<ScanRenderer>> renderers;
    bool renderers_valid = false;
    const Session* renderers_base = nullptr;
    PointClouds::PointCloudDimensions scene_dims{ 0, 0, 0, 0, 0, 1, 1, 1, 1 };
    ScanColorMode color_mode = ScanColorMode::FlatIntensity;

    Vector3 toVec3(const Eigen::Vector3d& v)
    {
        return Vector3{ static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z()) };
    }

    Vector2 worldToScreen(const Vector3& w)
    {
        const ImGuiIO& io = ImGui::GetIO();
        const Matrix& m = frame_mvp;
        float cx = m.m0 * w.x + m.m4 * w.y + m.m8 * w.z + m.m12;
        float cy = m.m1 * w.x + m.m5 * w.y + m.m9 * w.z + m.m13;
        float cw = m.m3 * w.x + m.m7 * w.y + m.m11 * w.z + m.m15;
        if (cw < 1e-6f) // behind the camera
            return Vector2{ -1000.f, -1000.f };
        return Vector2{ (cx / cw * 0.5f + 0.5f) * io.DisplaySize.x, (0.5f - cy / cw * 0.5f) * io.DisplaySize.y };
    }

    void copyMatrix(const Matrix& m, float out[16])
    {
        const float v[16] = { m.m0, m.m1, m.m2, m.m3, m.m4, m.m5, m.m6, m.m7, m.m8, m.m9, m.m10, m.m11, m.m12, m.m13, m.m14, m.m15 };
        std::copy(v, v + 16, out);
    }

    ScanRenderer* rendererOf(int session_index)
    {
        return session_index >= 0 && session_index < static_cast<int>(renderers.size()) ? renderers[session_index].get() : nullptr;
    }
} // namespace

std::string truncPath(const std::string& fullPath)
{
    std::filesystem::path path(fullPath);
    return "..\\" + path.parent_path().parent_path().filename().string() + "\\" + path.parent_path().filename().string() + "\\" +
        path.filename().string();
}

void wheel(int, int, int, int)
{
    if (!ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow))
        camera.zoom(GetMouseWheelMove(), ImGui::GetIO().KeyShift);
}

void motion(int x, int y)
{
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse)
        return;

    float dx = static_cast<float>(x - mouse_old_x);
    float dy = static_cast<float>(y - mouse_old_y);

    // Ctrl/Shift+click picks a rotation center; don't let a stray drag break that transition.
    if (!io.KeyCtrl && !io.KeyShift)
    {
        if (mouse_buttons & 1)
            camera.dragOrbit(dx, dy);
        if (mouse_buttons & 4)
        {
            if (camera.isOrtho)
                camera.dragPanOrtho(dx, dy, io.DisplaySize.x, io.DisplaySize.y);
            else
                camera.dragPanPerspective(dx, dy);
        }
    }

    mouse_old_x = x;
    mouse_old_y = y;
}

void reshape(int w, int h)
{
    camera.applyPerspectiveProjection(w, h);
}

bool initGL(int*, char**, const std::string& winTitle, void (*display)(), void (*mouse)(int, int, int, int))
{
    display_cb = display;
    mouse_cb = mouse;

    // HiDPI breaks ImGui scaling on Windows, so only macOS/Linux enable it (as in step 2).
    unsigned int flags = FLAG_WINDOW_RESIZABLE;
#if defined(__APPLE__) || defined(__linux__)
    flags |= FLAG_WINDOW_HIGHDPI;
#endif
    SetConfigFlags(flags);
    InitWindow(1600, 900, winTitle.c_str());
    SetExitKey(KEY_NULL); // Esc must not close the window
    SetTargetFPS(60);
    raylib_widgets::fitWindowToScreen(100, 100, true);

    rlImGuiSetup(true);
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard | ImGuiConfigFlags_NavEnableGamepad | ImGuiConfigFlags_DockingEnable;
    io.ConfigDockingWithShift = true;
    return true;
}

void mainLoop()
{
    const int buttons[][2] = { { MOUSE_BUTTON_LEFT, GLUT_LEFT_BUTTON },
                               { MOUSE_BUTTON_MIDDLE, GLUT_MIDDLE_BUTTON },
                               { MOUSE_BUTTON_RIGHT, GLUT_RIGHT_BUTTON } };
    while (!WindowShouldClose())
    {
        int x = GetMouseX();
        int y = GetMouseY();
        for (const auto& b : buttons)
        {
            if (IsMouseButtonPressed(b[0]))
                mouse_cb(b[1], GLUT_DOWN, x, y);
            if (IsMouseButtonReleased(b[0]))
                mouse_cb(b[1], GLUT_UP, x, y);
        }
        motion(x, y);
        if (GetMouseWheelMove() != 0.0f)
            wheel(0, GetMouseWheelMove() > 0.0f ? 1 : -1, x, y);

        BeginDrawing();
        display_cb();
        EndDrawing();
    }
}

void shutdownGL()
{
    renderers.clear(); // GPU buffers must go while the GL context exists
    rlImGuiShutdown();
    CloseWindow();
}

void showAxes()
{
    if (!show_axes && !ImGui::GetIO().KeyCtrl)
        return;

    const Vector3 rc = camera.euler.rotationCenter;
    rlBegin(RL_LINES);
    rlColor3f(1.f, 1.f, 1.f);
    for (int axis = 0; axis < 3; axis++)
    {
        Vector3 d{ axis == 0 ? 1.f : 0.f, axis == 1 ? 1.f : 0.f, axis == 2 ? 1.f : 0.f };
        rlVertex3f(rc.x - d.x, rc.y - d.y, rc.z - d.z);
        rlVertex3f(rc.x + d.x, rc.y + d.y, rc.z + d.z);
    }
    rlColor3f(1.f, 0.f, 0.f);
    rlVertex3f(0, 0, 0);
    rlVertex3f(100, 0, 0);
    rlColor3f(0.f, 1.f, 0.f);
    rlVertex3f(0, 0, 0);
    rlVertex3f(0, 100, 0);
    rlColor3f(0.f, 0.f, 1.f);
    rlVertex3f(0, 0, 0);
    rlVertex3f(0, 0, 100);
    rlEnd();
}

void updateCameraTransition()
{
    camera.updateEulerTransition(GetFrameTime());
}

void updateOrthoView()
{
    const ImGuiIO& io = ImGui::GetIO();
    viewLocal.rotate(Eigen::AngleAxisf((camera.euler.rotateX + camera.euler.rotateY) * DEG_TO_RAD, Eigen::Vector3f::UnitZ()));
    camera.updateOrtho(io.DisplaySize.x / io.DisplaySize.y);
}

void camMenu()
{
    using P = raylib_widgets::OrbitCamera::EulerPreset;
    if (ImGui::BeginMenu("Camera"))
    {
        const std::pair<const char*, P> items[] = { { "Front (yz view)", P::Front }, { "Back", P::Back },
                                                    { "Left (xz view)", P::Left },   { "Right", P::Right },
                                                    { "Top (xy view)", P::Top },     { "Bottom", P::Bottom },
                                                    { "Isometric", P::Iso } };
        const char* keys[] = { "key F", "key B", "key L", "key R", "key T", "key U", "key I" };
        for (size_t i = 0; i < 7; i++)
            if (ImGui::MenuItem(items[i].first, keys[i]))
                camera.setEulerPreset(items[i].second);
        ImGui::Separator();
        if (ImGui::MenuItem("Reset", "key Z"))
        {
            camera.setEulerPreset(P::Reset);
            viewer_decimate_point_cloud = 2;
        }
        ImGui::EndMenu();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Change camera view to fixed positions");
}

void view_kbd_shortcuts()
{
    using P = raylib_widgets::OrbitCamera::EulerPreset;
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureKeyboard)
        return;

    const float step = 0.5f * camera.eulerMouseSensitivity;
    const std::pair<ImGuiKey, Vector2> arrows[] = { { ImGuiKey_RightArrow, { 1, 0 } },
                                                    { ImGuiKey_LeftArrow, { -1, 0 } },
                                                    { ImGuiKey_UpArrow, { 0, 1 } },
                                                    { ImGuiKey_DownArrow, { 0, -1 } } };
    for (const auto& [key, d] : arrows)
    {
        if (!ImGui::IsKeyPressed(key, true))
            continue;
        if (io.KeyShift)
        {
            camera.euler.translate.x += d.x * step;
            camera.euler.translate.y += d.y * step;
            camera.breakEulerTransition();
        }
        else if (io.KeyCtrl)
        {
            camera.euler.rotateY -= d.x * 0.6f;
            camera.euler.rotateX -= d.y * 0.6f;
            camera.breakEulerTransition();
        }
    }

    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_R, false))
        cor_gui = true;
    if (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_Z, false) && !camera.isOrtho)
        camera.lockZ = !camera.lockZ;

    if (io.KeyCtrl || io.KeyAlt || io.KeyShift)
        return;

    const std::pair<ImGuiKey, P> presets[] = { { ImGuiKey_B, P::Back },  { ImGuiKey_F, P::Front }, { ImGuiKey_I, P::Iso },
                                               { ImGuiKey_L, P::Left },  { ImGuiKey_R, P::Right }, { ImGuiKey_T, P::Top },
                                               { ImGuiKey_U, P::Bottom } };
    for (const auto& [key, preset] : presets)
        if (ImGui::IsKeyPressed(key))
            camera.setEulerPreset(preset);
    if (ImGui::IsKeyPressed(ImGuiKey_Z))
    {
        camera.setEulerPreset(P::Reset);
        viewer_decimate_point_cloud = 2;
    }

    if (ImGui::IsKeyPressed(ImGuiKey_C, false))
        compass_ruler = !compass_ruler;
    if (ImGui::IsKeyPressed(ImGuiKey_O, false))
        camera.isOrtho = !camera.isOrtho;
    if (ImGui::IsKeyPressed(ImGuiKey_X, false))
        show_axes = !show_axes;
    for (int k = 1; k <= 9; k++)
        if (ImGui::IsKeyPressed(static_cast<ImGuiKey>(ImGuiKey_0 + k)))
            point_size = k;
}

void cor_window()
{
    raylib_widgets::showEulerCenterOfRotationWindow(cor_gui, camera, xText, yText, zText);
}

void info_window(const std::vector<std::string>& infoLines, const std::vector<ShortcutEntry>& appShortcuts)
{
    raylib_widgets::ShowInfoWindow(info_gui, infoLines, appShortcuts, HDMAPPING_VERSION_STRING, __DATE__);
}

void drawMiniCompassWithRuler()
{
    // Rows 0/1 of the world-to-eye rotation are the world directions of screen right/up.
    const Eigen::Matrix3f& R = viewLocal.rotation();
    Color rulerColor = ColorFromNormalized(Vector4{ 1.0f - bg_color.x, 1.0f - bg_color.y, 1.0f - bg_color.z, 1.0f });
    raylib_widgets::drawCompassRuler(
        Vector3{ R(0, 0), R(0, 1), R(0, 2) },
        Vector3{ R(1, 0), R(1, 1), R(1, 2) },
        camera.euler.translate.z,
        rulerColor,
        raylib_widgets::CompassAxisLabels{ "X (long.)", "Y (lat.)", "Z (vert.)" });
}

Eigen::Vector3d rayIntersection(const LaserBeam& laser_beam, const RegistrationPlaneFeature::Plane& plane)
{
    Eigen::Vector3d hit = laser_beam.position;
    raylib_widgets::intersectPlane(laser_beam.position, laser_beam.direction, plane.a, plane.b, plane.c, plane.d, hit);
    return hit;
}

LaserBeam GetLaserBeam(int x, int y)
{
    Ray ray = camera.eulerScreenRay(x, y, GetScreenWidth(), GetScreenHeight());
    LaserBeam laser_beam;
    laser_beam.position = Eigen::Vector3d(ray.position.x, ray.position.y, ray.position.z);
    laser_beam.direction = Eigen::Vector3d(ray.direction.x, ray.direction.y, ray.direction.z);
    return laser_beam;
}

double distance_point_to_line(const Eigen::Vector3d& point, const LaserBeam& line)
{
    return raylib_widgets::distancePointToLine(point, line.position, line.direction);
}

// Same behavior as utils.cpp: Ctrl picks the loop closure source (and time_stamp_offset), Shift the target;
// with more than two visible sessions all visible ones are searched. Moves the rotation center to the pick.
void getClosestTrajectoriesPoint(
    std::vector<Session>& sessions,
    int x,
    int y,
    const int first_session_index,
    const int second_session_index,
    const int number_visible_sessions,
    int& index_loop_closure_source,
    int& index_loop_closure_target,
    bool KeyShift,
    double& time_stamp_offset)
{
    const auto laser_beam = GetLaserBeam(x, y);
    double min_distance = std::numeric_limits<double>::max();
    Vector3 center = camera.eulerGoal.rotationCenter;

    auto visit = [&](int s, bool source_target)
    {
        if (s < 0 || s >= static_cast<int>(sessions.size()))
            return;
        const auto& pcs = sessions[s].point_clouds_container.point_clouds;
        for (size_t i = 0; i < pcs.size(); i++)
            for (size_t j = 0; j < pcs[i].local_trajectory.size(); j++)
            {
                Eigen::Vector3d vp = pcs[i].m_pose * pcs[i].local_trajectory[j].m_pose.translation();
                double dist = distance_point_to_line(vp, laser_beam);
                if (dist >= min_distance)
                    continue;
                min_distance = dist;
                if (source_target && KeyShift)
                {
                    index_loop_closure_target = static_cast<int>(i);
                    continue;
                }
                if (source_target)
                    index_loop_closure_source = static_cast<int>(i);
                center = toVec3(vp);
                time_stamp_offset = pcs[i].local_trajectory[j].timestamps.first;
            }
    };

    if (number_visible_sessions == 1)
        visit(first_session_index, true);
    else if (number_visible_sessions == 2)
        visit(KeyShift ? second_session_index : first_session_index, true);
    else
        for (size_t s = 0; s < sessions.size(); s++)
            if (sessions[s].visible)
                visit(static_cast<int>(s), false);

    camera.moveEulerRotationCenterTo(center);
}

void setNewRotationCenter(int x, int y)
{
    RegistrationPlaneFeature::Plane pl;
    pl.a = 0;
    pl.b = 0;
    pl.c = 1;
    pl.d = 0;
    Eigen::Vector3d c = rayIntersection(GetLaserBeam(x, y), pl);
    std::cout << "Setting new rotation center to:\n" << c << std::endl;
    camera.moveEulerRotationCenterTo(toVec3(c));
}

bool checkClHelp(int argc, char** argv)
{
    for (int i = 1; i < argc; ++i)
    {
        std::string arg(argv[i]);
        if (arg == "-h" || arg == "/h" || arg == "--help" || arg == "/?")
            return true;
    }
    return false;
}

void getProjectionMatrix(float m[16])
{
    copyMatrix(rlGetMatrixProjection(), m);
}

void getModelviewMatrix(float m[16])
{
    copyMatrix(rlGetMatrixModelview(), m);
}

void captureFrameMatrices()
{
    camera.captureFrameMatrices();
    frame_mvp = MatrixMultiply(camera.frameView3D, camera.frameProj3D);
}

void color3f(float r, float g, float b)
{
    current_color[0] = r;
    current_color[1] = g;
    current_color[2] = b;
    rlColor3f(r, g, b);
}

void beginLineStrip()
{
    strip.clear();
}

void lineStripVertex3f(float x, float y, float z)
{
    strip.push_back({ Vector3{ x, y, z }, { current_color[0], current_color[1], current_color[2] } });
}

void endLineStrip()
{
    rlBegin(RL_LINES);
    for (size_t i = 1; i < strip.size(); i++)
        for (const auto* v : { &strip[i - 1], &strip[i] })
        {
            rlColor3f(v->c[0], v->c[1], v->c[2]);
            rlVertex3f(v->p.x, v->p.y, v->p.z);
        }
    rlEnd();
    strip.clear();
}

void labelPos3f(float x, float y, float z)
{
    label_pos = Vector3{ x, y, z };
    std::copy(current_color, current_color + 3, label_color);
}

void labelText(const std::string& text)
{
    labels.push_back({ label_pos, text, ColorFromNormalized(Vector4{ label_color[0], label_color[1], label_color[2], 1.f }) });
}

void end3DAndDrawLabels()
{
    const ImGuiIO& io = ImGui::GetIO();
    raylib_widgets::end3DMatrixStack(io.DisplaySize.x, io.DisplaySize.y);

    for (const auto& l : labels)
    {
        // Outlined, so labels stay readable over geometry of the same color.
        Vector2 s = worldToScreen(l.p);
        int x = static_cast<int>(s.x) + 6;
        int y = static_cast<int>(s.y) - 26;
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                if (dx != 0 || dy != 0)
                    DrawText(l.text.c_str(), x + dx, y + dy, 20, BLACK);
        DrawText(l.text.c_str(), x, y, 20, l.color);
    }
    labels.clear();
}

void syncSessionRenderers(const std::vector<Session>& sessions)
{
    if (renderers_valid && renderers.size() == sessions.size() && renderers_base == sessions.data())
    {
        for (size_t i = 0; i < sessions.size(); i++)
            renderers[i]->syncPoses(sessions[i].point_clouds_container.point_clouds);
        return;
    }

    renderers.clear();
    bool first = true;
    for (const auto& s : sessions)
    {
        auto renderer = std::make_unique<ScanRenderer>();
        renderer->init();
        renderer->rebuildAll(s.point_clouds_container.point_clouds);
        renderers.push_back(std::move(renderer));

        if (s.point_clouds_container.point_clouds.empty())
            continue;
        const auto d = s.point_clouds_container.compute_point_cloud_dimension();
        if (first)
            scene_dims = d;
        scene_dims.x_min = std::min(scene_dims.x_min, d.x_min);
        scene_dims.x_max = std::max(scene_dims.x_max, d.x_max);
        scene_dims.y_min = std::min(scene_dims.y_min, d.y_min);
        scene_dims.y_max = std::max(scene_dims.y_max, d.y_max);
        scene_dims.z_min = std::min(scene_dims.z_min, d.z_min);
        scene_dims.z_max = std::max(scene_dims.z_max, d.z_max);
        first = false;
    }
    scene_dims.length = scene_dims.x_max - scene_dims.x_min;
    scene_dims.width = scene_dims.y_max - scene_dims.y_min;
    scene_dims.height = scene_dims.z_max - scene_dims.z_min;
    renderers_base = sessions.data();
    renderers_valid = true;
}

void invalidateSessionRenderers()
{
    renderers_valid = false;
}

void renderSession(const Session& session, const ObservationPicking&, int decimate, int reduce_trajectory)
{
    ScanRenderer* r = rendererOf(static_cast<int>(&session - renderers_base));
    const auto& pcc = session.point_clouds_container;
    if (!r || pcc.point_clouds.empty())
        return;

    const Vector3 rc = camera.euler.rotationCenter;
    r->draw(
        pcc.point_clouds,
        static_cast<float>(pcc.point_clouds[0].point_size),
        color_mode,
        static_cast<float>(scene_dims.z_min),
        static_cast<float>(scene_dims.z_max),
        Eigen::Vector3d(rc.x, rc.y, rc.z),
        static_cast<float>(std::max({ scene_dims.length, scene_dims.width, scene_dims.height, 1.0 })),
        decimate,
        pcc.xz_intersection,
        pcc.yz_intersection,
        pcc.xy_intersection,
        static_cast<float>(pcc.intersection_width),
        pcc.show_with_initial_pose);
    r->drawTrajectories(
        pcc.point_clouds,
        reduce_trajectory,
        pcc.show_imu_to_lio_diff,
        pcc.xz_intersection,
        pcc.yz_intersection,
        pcc.xy_intersection,
        pcc.show_with_initial_pose,
        pcc.imu_to_lio_diff_scale);
}

void renderScan(
    int session_index,
    int index,
    bool show_with_initial_pose,
    const ObservationPicking&,
    int decimate,
    int reduce_trajectory,
    bool,
    bool,
    bool,
    double,
    bool)
{
    ScanRenderer* r = rendererOf(session_index);
    if (!r || index < 0 || index >= static_cast<int>(renderers_base[session_index].point_clouds_container.point_clouds.size()))
        return;
    const PointCloud& pc = renderers_base[session_index].point_clouds_container.point_clouds[index];
    renderScanAtPose(
        session_index, index, show_with_initial_pose ? pc.m_initial_pose : pc.m_pose, decimate, reduce_trajectory, pc.render_color);
}

void renderScanAtPose(int session_index, int index, const Eigen::Affine3d& pose, int, int reduce_trajectory, const float color[3])
{
    ScanRenderer* r = rendererOf(session_index);
    if (!r || index < 0 || index >= static_cast<int>(renderers_base[session_index].point_clouds_container.point_clouds.size()))
        return;
    const PointCloud& pc = renderers_base[session_index].point_clouds_container.point_clouds[index];
    if (!pc.visible)
        return;

    r->drawCachedWithTransform(
        static_cast<size_t>(index),
        pose * pc.m_pose.inverse(),
        ColorFromNormalized(Vector4{ color[0], color[1], color[2], 1.f }),
        static_cast<float>(pc.point_size),
        false);

    const size_t stride = std::max(1, reduce_trajectory);
    rlBegin(RL_LINES);
    rlColor3f(color[0], color[1], color[2]);
    for (size_t i = stride; i < pc.local_trajectory.size(); i += stride)
        for (size_t k : { i - stride, i })
        {
            Eigen::Vector3d p = (pc.m_pose * pc.local_trajectory[k].m_pose).translation();
            rlVertex3f(static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
        }
    rlEnd();
}

void renderGroundControlPoints(const GroundControlPoints& ground_control_points, const PointClouds& point_clouds_container)
{
    const Color mark{ 179, 77, 128, 255 };
    const Color connector{ 0, 77, 153, 255 };
    for (const auto& gcp : ground_control_points.gpcs)
    {
        if (gcp.index_to_node_inner < 0 || gcp.index_to_node_inner >= static_cast<int>(point_clouds_container.point_clouds.size()))
            continue;
        const auto& pc = point_clouds_container.point_clouds[gcp.index_to_node_inner];
        if (gcp.index_to_node_outer < 0 || gcp.index_to_node_outer >= static_cast<int>(pc.local_trajectory.size()))
            continue;

        Vector3 c = toVec3(pc.m_pose * pc.local_trajectory[gcp.index_to_node_outer].m_pose.translation());
        float h = static_cast<float>(gcp.lidar_height_above_ground);
        Vector3 g{ static_cast<float>(gcp.x), static_cast<float>(gcp.y), static_cast<float>(gcp.z) };
        DrawLine3D(Vector3{ g.x - 0.05f, g.y, g.z }, Vector3{ g.x + 0.05f, g.y, g.z }, mark);
        DrawLine3D(Vector3{ g.x, g.y - 0.05f, g.z }, Vector3{ g.x, g.y + 0.05f, g.z }, mark);
        DrawLine3D(g, Vector3{ g.x, g.y, g.z + h }, mark);
        DrawLine3D(c, Vector3{ g.x, g.y, g.z + h }, connector);
        labels.push_back({ Vector3{ g.x, g.y, g.z + h + 0.1f }, gcp.name, WHITE });
    }
}

void renderControlPoints(const ControlPoints& control_points, const PointClouds& point_clouds_container)
{
    const Color mark{ 179, 77, 128, 255 };
    const Color connector{ 0, 77, 153, 255 };
    const auto& pcs = point_clouds_container.point_clouds;
    for (const auto& cp : control_points.cps)
    {
        if (cp.index_to_pose < 0 || cp.index_to_pose >= static_cast<int>(pcs.size()))
            continue;
        Vector3 c = toVec3(pcs[cp.index_to_pose].m_pose * Eigen::Vector3d(cp.x_source_local, cp.y_source_local, cp.z_source_local));
        Vector3 g{ static_cast<float>(cp.x_target_global), static_cast<float>(cp.y_target_global), static_cast<float>(cp.z_target_global) };
        DrawLine3D(Vector3{ g.x - 0.05f, g.y, g.z }, Vector3{ g.x + 0.05f, g.y, g.z }, mark);
        DrawLine3D(Vector3{ g.x, g.y - 0.05f, g.z }, Vector3{ g.x, g.y + 0.05f, g.z }, mark);
        DrawLine3D(c, g, connector);
        labels.push_back({ Vector3{ g.x, g.y, g.z + 0.1f }, cp.name, WHITE });
    }
}

void pointsColorMenu()
{
    if (!ImGui::BeginMenu("Points color"))
        return;
    const std::tuple<const char*, ScanColorMode, const char*> items[] = {
        { "> Session color", ScanColorMode::FlatIntensity, "Each session in its own color (Settings window), shaded by intensity" },
        { "> By intensity (gradient)", ScanColorMode::Intensity, "Per-point jet colormap from LAS/LAZ intensity" },
        { "> By height (gradient)", ScanColorMode::Elevation, "Per-point jet colormap from world Z, over all sessions" },
        { "> By distance (gradient)", ScanColorMode::Distance, "Per-point jet colormap from distance to the rotation center" },
    };
    for (const auto& [label, mode, tooltip] : items)
    {
        if (ImGui::MenuItem(label, nullptr, color_mode == mode))
            color_mode = mode;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", tooltip);
    }
    ImGui::EndMenu();
}

void assignDistinctSessionColors(std::vector<Session>& sessions)
{
    static const float palette[][3] = {
        { 0.90f, 0.10f, 0.10f }, // red
        { 0.15f, 0.45f, 0.95f }, // blue
        { 1.00f, 0.55f, 0.05f }, // orange
        { 0.15f, 0.75f, 0.20f }, // green
        { 0.85f, 0.20f, 0.85f }, // magenta
        { 0.10f, 0.85f, 0.85f }, // cyan
        { 0.95f, 0.90f, 0.10f }, // yellow
        { 0.55f, 0.30f, 0.95f }, // violet
    };
    const size_t n = sizeof(palette) / sizeof(palette[0]);

    for (size_t i = 0; i < sessions.size(); i++)
    {
        float c[3];
        if (i < n)
            std::copy(palette[i], palette[i] + 3, c);
        else
        {
            std::mt19937 rng(static_cast<unsigned>(i));
            Color rgb = ColorFromHSV(std::uniform_real_distribution<float>(0.f, 360.f)(rng), 0.85f, 0.95f);
            c[0] = rgb.r / 255.f;
            c[1] = rgb.g / 255.f;
            c[2] = rgb.b / 255.f;
        }
        std::copy(c, c + 3, sessions[i].render_color);
        for (auto& pc : sessions[i].point_clouds_container.point_clouds)
        {
            std::copy(c, c + 3, pc.render_color);
            std::copy(c, c + 3, pc.traj_color);
        }
    }
}

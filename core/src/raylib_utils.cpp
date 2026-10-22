#include <Core/raylib_utils.hpp>

#include "external/glad.h"
#include "raymath.h"
#include "rlImGui.h"

#include <imgui_internal.h>

#include <RaylibWidgets/CenterOfRotationWindow.h>
#include <RaylibWidgets/CompassRuler.h>
#include <RaylibWidgets/RayPlaneD.h>
#include <RaylibWidgets/WindowFit.h>

#include <HDMapping/HDMAPPING_ConfigureInfo.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>

raylib_widgets::OrbitCamera camera;

// GLUT apps used 1000 (every point was a glVertex call); GPU buffers draw full density, as in step 2.
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

#ifdef _WIN32
//! For raylib_taskbar_win.cpp, which can't include raylib.h next to windows.h.
void* raylibWindowHandle()
{
    return GetWindowHandle();
}
#endif

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

    // beginPoints()/endPoints(): interleaved position + color, streamed through one dynamic VBO in
    // fixed-size chunks so the buffer stays small however many points a frame draws.
    struct PointVertex
    {
        float x, y, z;
        float r, g, b;
    };
    constexpr size_t kPointChunk = size_t(1) << 20;
    std::vector<PointVertex> points;
    float points_size = 1.f;

    struct PointBatchGPU
    {
        bool initialized = false;
        Shader shader{};
        int locMvp = -1;
        int locPointSize = -1;
        unsigned int vao = 0;
        unsigned int vbo = 0;
    } point_gpu;

    constexpr const char* kPointBatchVS = R"(
#version 330
in vec3 vertexPosition;
in vec3 vertexColor;
uniform mat4 mvp;
uniform float pointSize;
out vec3 fragColor;
void main()
{
    gl_Position = mvp * vec4(vertexPosition, 1.0);
    gl_PointSize = pointSize;
    fragColor = vertexColor;
}
)";

    constexpr const char* kPointBatchFS = R"(
#version 330
in vec3 fragColor;
out vec4 finalColor;
void main()
{
    finalColor = vec4(fragColor, 1.0);
}
)";

    // vertexColor is bound to rlgl's default color attribute location by LoadShaderFromMemory().
    constexpr int kPointColorLocation = RL_DEFAULT_SHADER_ATTRIB_LOCATION_COLOR;

    void initPointBatch()
    {
        point_gpu.initialized = true;
        point_gpu.shader = LoadShaderFromMemory(kPointBatchVS, kPointBatchFS);
        point_gpu.locMvp = GetShaderLocation(point_gpu.shader, "mvp");
        point_gpu.locPointSize = GetShaderLocation(point_gpu.shader, "pointSize");

        point_gpu.vao = rlLoadVertexArray();
        rlEnableVertexArray(point_gpu.vao);
        point_gpu.vbo = rlLoadVertexBuffer(nullptr, static_cast<int>(kPointChunk * sizeof(PointVertex)), true);
        rlSetVertexAttribute(0, 3, RL_FLOAT, false, sizeof(PointVertex), 0);
        rlEnableVertexAttribute(0);
        rlSetVertexAttribute(kPointColorLocation, 3, RL_FLOAT, false, sizeof(PointVertex), 3 * sizeof(float));
        rlEnableVertexAttribute(kPointColorLocation);
        rlDisableVertexArray();

        // Required for gl_PointSize (set from the vertex shader) to take effect, as in ScanRenderer::init().
        glEnable(GL_PROGRAM_POINT_SIZE);
    }

    void releasePointBatch()
    {
        if (!point_gpu.initialized)
            return;
        rlUnloadVertexArray(point_gpu.vao);
        rlUnloadVertexBuffer(point_gpu.vbo);
        UnloadShader(point_gpu.shader);
        point_gpu = PointBatchGPU{};
    }

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
    if (camera.isOrtho)
        camera.applyOrthoProjection(static_cast<float>(w) / static_cast<float>(h));
    else
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
    releasePointBatch(); // GPU buffers must go while the GL context exists
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
    raylib_widgets::ShowInfoWindow(info_gui, infoLines, appShortcuts, HDMAPPING_CONFIGURE_PROJECT_VERSION, __DATE__);
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

void draw_ellipse(const Eigen::Matrix3d& covar, const Eigen::Vector3d& mean, Eigen::Vector3f color, float nstd)
{
    Eigen::LLT<Eigen::Matrix<double, 3, 3>> cholSolver(covar);
    Eigen::Matrix3d transform = cholSolver.matrixL();

    const double pi = 3.141592;
    const double di = 0.02;
    const double dj = 0.04;
    const double du = di * 2 * pi;
    const double dv = dj * pi;

    rlBegin(RL_LINES);
    rlColor3f(color.x(), color.y(), color.z());
    for (double i = 0; i < 1.0; i += di) // horizontal
    {
        for (double j = 0; j < 1.0; j += dj) // vertical
        {
            double u = i * 2 * pi; // 0     to  2pi
            double v = (j - 0.5) * pi; //-pi/2 to pi/2

            const Eigen::Vector3d pp[4] = { { cos(v) * cos(u), cos(v) * sin(u), sin(v) },
                                            { cos(v) * cos(u + du), cos(v) * sin(u + du), sin(v) },
                                            { cos(v + dv) * cos(u + du), cos(v + dv) * sin(u + du), sin(v + dv) },
                                            { cos(v + dv) * cos(u), cos(v + dv) * sin(u), sin(v + dv) } };
            Vector3 tp[4];
            for (int k = 0; k < 4; k++)
                tp[k] = toVec3(transform * (nstd * pp[k]) + mean);

            // The cell's GL_LINE_LOOP, as 4 segments.
            for (int k = 0; k < 4; k++)
            {
                rlVertex3f(tp[k].x, tp[k].y, tp[k].z);
                rlVertex3f(tp[(k + 1) % 4].x, tp[(k + 1) % 4].y, tp[(k + 1) % 4].z);
            }
        }
    }
    rlEnd();
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

void setNewRotationCenter(const Eigen::Vector3d& center)
{
    std::cout << "Setting new rotation center to:\n" << center << std::endl;
    camera.moveEulerRotationCenterTo(toVec3(center));
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

void beginPoints(float size)
{
    points.clear();
    points_size = size;
}

void pointVertex3f(float x, float y, float z)
{
    points.push_back({ x, y, z, current_color[0], current_color[1], current_color[2] });
}

void endPoints()
{
    if (points.empty())
        return;
    if (!point_gpu.initialized)
        initPointBatch();

    // Lines queued so far must be drawn first, with the matrices they were queued under.
    rlDrawRenderBatchActive();

    Matrix mvp = MatrixMultiply(rlGetMatrixModelview(), rlGetMatrixProjection());
    rlEnableShader(point_gpu.shader.id);
    rlSetUniformMatrix(point_gpu.locMvp, mvp);
    rlSetUniform(point_gpu.locPointSize, &points_size, RL_SHADER_UNIFORM_FLOAT, 1);
    rlEnableVertexArray(point_gpu.vao);
    for (size_t first = 0; first < points.size(); first += kPointChunk)
    {
        const size_t count = std::min(kPointChunk, points.size() - first);
        rlUpdateVertexBuffer(point_gpu.vbo, points.data() + first, static_cast<int>(count * sizeof(PointVertex)), 0);
        glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(count));
    }
    rlDisableVertexArray();
    rlDisableShader();
    points.clear();
}

float IntensityRange::normalize(float intensity) const
{
    return max > min ? std::clamp((intensity - min) / (max - min), 0.f, 1.f) : 0.f;
}

IntensityRange intensityRange(const std::vector<Point3Di>& points, size_t stride)
{
    IntensityRange range;
    if (points.empty())
        return range;
    range.min = range.max = points[0].intensity;
    for (size_t i = 0; i < points.size(); i += std::max<size_t>(stride, 1))
    {
        range.min = std::min(range.min, points[i].intensity);
        range.max = std::max(range.max, points[i].intensity);
    }
    return range;
}

void color3fIntensityShaded(float r, float g, float b, float intensity, const IntensityRange& range)
{
    // Same as raylib_render_shaders.hpp's colorMode 4: sqrt lifts the typically low-skewed intensity
    // distribution, the 0.35 floor keeps low-intensity points' hue visible.
    const float shade = 0.35f + 0.65f * std::sqrt(range.normalize(intensity));
    color3f(r * shade, g * shade, b * shade);
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

void addLabel(const Vector3& position, const std::string& text, Color color)
{
    labels.push_back({ position, text, color });
}

std::vector<std::string> takeDroppedFiles()
{
    std::vector<std::string> paths;
    if (!IsFileDropped())
        return paths;
    FilePathList dropped = LoadDroppedFiles();
    for (unsigned int i = 0; i < dropped.count; i++)
        paths.emplace_back(dropped.paths[i]);
    UnloadDroppedFiles(dropped);
    return paths;
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

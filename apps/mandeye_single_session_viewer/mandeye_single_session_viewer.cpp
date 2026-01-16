// clang-format off
#include <GL/glew.h>
#include <GL/freeglut.h>
// clang-format on

#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <imgui_internal.h>

#include <ImGuizmo.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <utils.hpp>

#include <Eigen/Eigen>

#include <transformations.h>

#include "pfd_wrapper.hpp"

#include "../lidar_odometry_step_1/lidar_odometry_utils.h"
#include <filesystem>

#include <HDMapping/Version.hpp>

#include <mutex>

#ifdef _WIN32
#include "resource.h"
#include <shellapi.h> // <-- Required for ShellExecuteA
#include <windows.h>
#endif
#include <GL_assert.h>

///////////////////////////////////////////////////////////////////////////////////

std::string winTitle = std::string("Single session viewer ") + HDMAPPING_VERSION_STRING;

std::vector<std::string> infoLines = { "This program is optional step in MANDEYE process",
                                       "",
                                       "It analyzes session created in step_1 for problems that need to be addressed further",
                                       "Next step will be to load session file with 'multi_view_tls_registration_step_2' app" };

// App specific shortcuts (Type and Shortcut are just for easy reference)
static const std::vector<ShortcutEntry> appShortcuts = { { "Normal keys", "A", "" },
                                                         { "", "Ctrl+A", "" },
                                                         { "", "B", "" },
                                                         { "", "Ctrl+B", "" },
                                                         { "", "C", "" },
                                                         { "", "Ctrl+C", "" },
                                                         { "", "D", "" },
                                                         { "", "Ctrl+D", "" },
                                                         { "", "E", "" },
                                                         { "", "Ctrl+E", "" },
                                                         { "", "F", "" },
                                                         { "", "Ctrl+F", "" },
                                                         { "", "G", "" },
                                                         { "", "Ctrl+G", "" },
                                                         { "", "H", "" },
                                                         { "", "Ctrl+H", "" },
                                                         { "", "I", "" },
                                                         { "", "Ctrl+I", "" },
                                                         { "", "J", "" },
                                                         { "", "Ctrl+K", "" },
                                                         { "", "K", "" },
                                                         { "", "Ctrl+K", "" },
                                                         { "", "L", "" },
                                                         { "", "Ctrl+L", "" },
                                                         { "", "M", "" },
                                                         { "", "Ctrl+M", "" },
                                                         { "", "N", "" },
                                                         { "", "Ctrl+N", "show Neightbouring scans" },
                                                         { "", "O", "" },
                                                         { "", "Ctrl+O", "Open session" },
                                                         { "", "P", "" },
                                                         { "", "Ctrl+P", "Properties" },
                                                         { "", "Q", "" },
                                                         { "", "Ctrl+Q", "" },
                                                         { "", "R", "" },
                                                         { "", "Ctrl+R", "" },
                                                         { "", "Shift+R", "" },
                                                         { "", "S", "" },
                                                         { "", "Ctrl+S", "" },
                                                         { "", "Ctrl+Shift+S", "" },
                                                         { "", "T", "" },
                                                         { "", "Ctrl+T", "" },
                                                         { "", "U", "" },
                                                         { "", "Ctrl+U", "" },
                                                         { "", "V", "" },
                                                         { "", "Ctrl+V", "" },
                                                         { "", "W", "" },
                                                         { "", "Ctrl+W", "" },
                                                         { "", "X", "" },
                                                         { "", "Ctrl+X", "" },
                                                         { "", "Y", "" },
                                                         { "", "Ctrl+Y", "" },
                                                         { "", "Z", "" },
                                                         { "", "Ctrl+Z", "" },
                                                         { "", "Shift+Z", "" },
                                                         { "", "1-9", "" },
                                                         { "Special keys", "Up arrow", "Intensity offset +" },
                                                         { "", "Shift + up arrow", "" },
                                                         { "", "Ctrl + up arrow", "" },
                                                         { "", "Down arrow", "Intensity offset -" },
                                                         { "", "Shift + down arrow", "" },
                                                         { "", "Ctrl + down arrow", "" },
                                                         { "", "Left arrow", "Point cloud index -" },
                                                         { "", "Shift + left arrow", "" },
                                                         { "", "Ctrl + left arrow", "" },
                                                         { "", "Right arrow", "Point cloud index +" },
                                                         { "", "Shift + right arrow", "" },
                                                         { "", "Ctrl + right arrow", "" },
                                                         { "", "Pg down", "Point cloud index -" },
                                                         { "", "Pg up", "Point cloud index +" },
                                                         { "", "- key", "Point cloud index -" },
                                                         { "", "+ key", "Point cloud index +" },
                                                         { "Mouse related", "Left click + drag", "" },
                                                         { "", "Right click + drag", "n" },
                                                         { "", "Scroll", "" },
                                                         { "", "Shift + scroll", "" },
                                                         { "", "Shift + drag", "" },
                                                         { "", "Ctrl + left click", "" },
                                                         { "", "Ctrl + right click", "" },
                                                         { "", "Ctrl + middle click", "" } };

#define SAMPLE_PERIOD (1.0 / 200.0)
namespace fs = std::filesystem;

ImVec4 pc_neigbouring_color = ImVec4(0.5f, 0.5f, 0.5f, 1.0f);
ImVec4 pc_color2 = ImVec4(0.0f, 0.0f, 1.0f, 1.0f);

int index_rendered_points_local = -1;
float offset_intensity = 0.0;
bool show_neighbouring_scans = false;

ColorScheme colorScheme = CS_SOLID;
ColorScheme oldcolorScheme = CS_SOLID;
bool usePose = false;

bool is_properties_gui = false;
bool is_session_gui = true;
bool is_index_gui = false;

Session session;
bool session_loaded = false;

int session_total_number_of_points = 0;
PointClouds::PointCloudDimensions session_dims;

// built in console output redirection to imgui window
///////////////////////////////////////////////////////////////////////////////////

#ifdef _WIN32
bool consWin = true;
#endif
bool consImGui = false;
bool consHooked = false;

class DualStreamBuf : public std::streambuf
{
public:
    DualStreamBuf(std::streambuf* sb1, std::string* imguiBuffer, bool isCerr)
        : consoleBuf(sb1)
        , imguiBuffer(imguiBuffer)
        , isCerr(isCerr)
    {
    }

protected:
    std::string currentLine;
    bool isCerr;

    int overflow(int c) override
    {
        // handle EOF
        if (c == traits_type::eof())
            return traits_type::not_eof(c);

        // Forward character to real console; check for errors
        if (consoleBuf->sputc((char)c) == traits_type::eof())
            return traits_type::eof();

        if (c == '\n')
        {
            imguiBuffer->append(isCerr ? "cerr: " : "cout: ");
            imguiBuffer->append(currentLine);
            imguiBuffer->push_back('\n');
            currentLine.clear();
        }
        else
        {
            currentLine.push_back((char)c);
        }

        return c;
    }

    int sync() override
    {
        // Flush partial line
        if (!currentLine.empty())
        {
            imguiBuffer->append(isCerr ? "cerr: " : "cout: ");
            imguiBuffer->append(currentLine);
            imguiBuffer->push_back('\n');
            currentLine.clear();
        }

        return consoleBuf->pubsync();
    }

private:
    std::streambuf* consoleBuf;
    std::string* imguiBuffer;
};

static std::string g_ImGuiLog; // store text for ImGui
static DualStreamBuf* g_coutBuf = nullptr;
static DualStreamBuf* g_cerrBuf = nullptr;
static std::streambuf* g_origCoutBuf = nullptr;
static std::streambuf* g_origCerrBuf = nullptr;

void ConsoleHook()
{
    g_origCoutBuf = std::cout.rdbuf(); // save original buffer
    g_origCerrBuf = std::cerr.rdbuf(); // save original buffer
    g_coutBuf = new DualStreamBuf(g_origCoutBuf, &g_ImGuiLog, false);
    g_cerrBuf = new DualStreamBuf(g_origCerrBuf, &g_ImGuiLog, true);
    std::cout.rdbuf(g_coutBuf); // replace
    std::cerr.rdbuf(g_cerrBuf); // replace

    consHooked = true;
}

void ConsoleUnhook()
{
    if (g_origCoutBuf)
        std::cout.rdbuf(g_origCoutBuf); // restore original buffer
    if (g_origCerrBuf)
        std::cerr.rdbuf(g_origCerrBuf); // restore original buffer
    delete g_coutBuf; // delete custom buffers
    delete g_cerrBuf; // delete custom buffers
    g_coutBuf = g_cerrBuf = nullptr;

    g_ImGuiLog.clear();
    consHooked = false;
}

static char g_ImGuiLogBuf[65536]; // or larger if needed

void ImGuiConsole(bool* p_open)
{
    if (ImGui::Begin("Console", p_open))
    {
        if (ImGui::Button("Clear"))
            g_ImGuiLog.clear();
        // ImGui::SameLine();
        // if (ImGui::Button("Copy"))
        //	ImGui::LogToClipboard();

        ImGui::Separator();

        if (ImGui::BeginChild("scrolling_region", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar))
        {
            // ImGui::TextUnformatted(g_ImGuiLog.c_str());

            // Copy std::string to buffer
            strncpy(g_ImGuiLogBuf, g_ImGuiLog.c_str(), sizeof(g_ImGuiLogBuf));
            g_ImGuiLogBuf[sizeof(g_ImGuiLogBuf) - 1] = 0; // ensure null termination

            // Make the input background transparent
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0, 0, 0, 0));
            ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImVec4(0, 0, 0, 0));
            ImGui::PushStyleColor(ImGuiCol_FrameBgActive, ImVec4(0, 0, 0, 0));
            ImGui::InputTextMultiline(
                "##console",
                g_ImGuiLogBuf,
                sizeof(g_ImGuiLogBuf),
                ImVec2(-1.0f, -1.0f),
                ImGuiInputTextFlags_ReadOnly | ImGuiInputTextFlags_AllowTabInput);
            ImGui::PopStyleColor(3);

            // auto-scroll
            // if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
            //    ImGui::SetScrollHereY(1.0f);
        }
        ImGui::EndChild();
    }

    ImGui::End();
}

// VBO/VAO/SSBO proof of concept implementation through glew. openGL 4.6 required
///////////////////////////////////////////////////////////////////////////////////
//
// in order to test have to be enabled from View menu before loading session so buffers are created
// should be tested with smaller sessions since session is "doubled" with gl_Points vector
bool gl_useVBOs = false;

// vertex shader
const char* pointsVertSource = R"glsl(
    #version 460 core

    //SSBOs
    layout(std430, binding = 0) buffer VertexCloudIndex {
        int cloudIndex[];  // cloud index per vertex
    };
 
    struct Cloud {         //order matters for std430 layout
        int colorScheme;   // 0=solid,1=random,2=intensity gradient, etc.
        vec3 fixedColor;   // fixed color
        mat4 pose;         // cloud transformation
    };

    layout(std430, binding = 1) buffer CloudsBuffer {
        Cloud clouds[];
    };

    //vertex inputs
    layout(location = 0) in vec3 aPos;        // vertex position
    layout(location = 1) in float aIntensity; // per-point intensity

    //uniforms
    uniform mat4 uMVP;                        // model-view-projection matrix
    uniform float uPointSize;                 // point size from GUI
    uniform int uUsePose;                     // whether to use cloud pose

    out float vIntensity;                     // pass intensity to fragment shader
    flat out int vColorScheme;                // pass cloud color scheme to fragment shader
    out vec3 vFixedColor;                     // pass fixed color / computed color to fragment shader

    void main()
    {
        int idx = cloudIndex[gl_VertexID];       // get cloud index
        Cloud c = clouds[idx];                   // fetch attributes

        mat4 finalPose = (uUsePose == 0) ?  mat4(1.0) : c.pose;  // identity if not using pose
        gl_Position = uMVP * finalPose * vec4(aPos, 1.0);

        gl_PointSize = uPointSize;
        vIntensity = aIntensity;
        vColorScheme = c.colorScheme;
        vFixedColor = c.fixedColor;
        //vFixedColor = vec3(float(idx % 10) / 10.0, float((idx/10) % 10) / 10.0, 0.0);
    }
    )glsl";

// fragment shader
const char* pointsFragSource = R"glsl(
    #version 460 core

    in float vIntensity;
    flat in int vColorScheme;
    in vec3 vFixedColor;
    out vec4 FragColor;

    uniform float uIntensityScale; // optional multiplier or normalization

    void main()
    {
        float norm = clamp(vIntensity * uIntensityScale, 0.0, 1.0);
        vec3 endColor;

        if (vColorScheme == 0)
            endColor = vFixedColor;                 // solid color
        else if (vColorScheme == 2)
            endColor = vec3(norm, 0.0, 1.0 - norm); // blue–red gradient by intensity
        else
            endColor = vec3(0.5);                   // default gray

        FragColor = vec4(endColor, 1.0);
    }
    )glsl";

///////////////////////////////////////////////////////////////////////////////////

struct gl_point
{
    Eigen::Vector3f pos;
    float intensity;
};

std::vector<gl_point> gl_Points;
/*= {
    {{0.0f, 0.0f, 1.0f}, 1.0f},
    {{0.5f, 0.0f, 1.0f}, 0.7f},
    {{0.0f, 0.5f, 1.0f}, 0.5f},
    {{0.0f, 0.0f, 2.0f}, 0.2f},
};*/

struct gl_cloud
{
    GLint offset;
    GLsizei count;
    bool visible; // show/hide
};

// Add to clouds vector
std::vector<gl_cloud> gl_clouds;
/*= { {
    0,                                //offset
    static_cast<GLsizei>(gl_Points.size()), //count
    true,                             //visible
    Eigen::Matrix4f::Identity(),      //pose
    1,                                //colorScheme
    Eigen::Vector3f(1.0f, 0.0f, 0.0f) //color = red, for example
} };*/

// CPU-side vectors for SSBO data
struct gl_cloudSSBO
{
    int colorScheme; // 4 bytes
    float padding0[3]; // pad to vec4 alignment (std430 requires vec3 as 16 bytes)
    float fixedColor[3]; // 12 bytes for vec3
    float padding1; // pad to 16 bytes
    float pose[16]; // mat4 = 16 floats (16*4 = 64 bytes)
};

std::vector<int> gl_cloudIndexSSBO; // size = gl_Points.size()
std::vector<gl_cloudSSBO> gl_cloudsSSBO; // Cloud is your struct from GLSL

Eigen::Matrix4f gl_mvp; // Model-View-Projection matrix

// Uniform locations
GLuint VAO, VBO = 0;

GLuint gl_uPointSize = 0;
GLuint gl_uMVP = 0;
GLuint gl_shaderProgram = 0;
GLuint gl_uIntensityScale = 0;
GLuint gl_uUsePose = 0;

GLuint gl_ssboCloudIndex, gl_ssboClouds = 0;

// gl_*** functions related to glew/openGL VBO/VAO
GLuint gl_compileShader(GLenum type, const char* source)
{
    GLuint shader = GL_CALL_RET(glCreateShader(type));
    GL_CALL(glShaderSource(shader, 1, &source, nullptr));
    GL_CALL(glCompileShader(shader));

    // check compilation
    GLint success;
    GL_CALL(glGetShaderiv(shader, GL_COMPILE_STATUS, &success));
    if (!success)
    {
        char infoLog[512];
        GL_CALL(glGetShaderInfoLog(shader, 512, nullptr, infoLog));
        std::cerr << "openGL shader compilation failed: " << infoLog << std::endl;
    }
    return shader;
}

GLuint gl_createShaderProgram(const char* vertSource, const char* fragSource)
{
    GLuint vertexShader = gl_compileShader(GL_VERTEX_SHADER, vertSource);
    GLuint fragmentShader = gl_compileShader(GL_FRAGMENT_SHADER, fragSource);

    GLuint program = GL_CALL_RET(glCreateProgram());
    GL_CALL(glAttachShader(program, vertexShader));
    GL_CALL(glAttachShader(program, fragmentShader));
    GL_CALL(glLinkProgram(program));

    // check linking
    GLint success;
    GL_CALL(glGetProgramiv(program, GL_LINK_STATUS, &success));
    if (!success)
    {
        char infoLog[512];
        GL_CALL(glGetProgramInfoLog(program, 512, nullptr, infoLog));
        std::cerr << "openGL program linking failed: " << infoLog << std::endl;
    }

    GL_CALL(glDeleteShader(vertexShader));
    GL_CALL(glDeleteShader(fragmentShader));

    return program;
}

void gl_loadPointCloudBuffer(const std::vector<gl_point>& points, GLuint& VAO, GLuint& VBO)
{
    // Create VAO + VBO
    GL_CALL(glGenVertexArrays(1, &VAO));
    GL_CALL(glBindVertexArray(VAO));

    GL_CALL(glGenBuffers(1, &VBO));
    GL_CALL(glBindBuffer(GL_ARRAY_BUFFER, VBO));
    GL_CALL(glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(gl_point), points.data(), GL_STATIC_DRAW));

    // --- Attribute 0: position ---
    // layout(location = 0) in vec3 aPos;
    GL_CALL(glEnableVertexAttribArray(0));
    GL_CALL(glVertexAttribPointer(
        0, // attribute index
        3, // vec3
        GL_FLOAT,
        GL_FALSE,
        sizeof(gl_point), // stride = full struct
        (void*)offsetof(gl_point, pos) // offset of position field
        ));

    // --- Attribute 1: intensity ---
    // layout(location = 1) in float aIntensity;
    GL_CALL(glEnableVertexAttribArray(1));
    GL_CALL(glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(gl_point), (void*)offsetof(gl_point, intensity)));

    // Unbind
    GL_CALL(glBindBuffer(GL_ARRAY_BUFFER, 0));
    GL_CALL(glBindVertexArray(0));
}

void gl_updateSSBOs()
{
    // Vertex -> Cloud index
    if (gl_ssboCloudIndex == 0)
    {
        GL_CALL(glGenBuffers(1, &gl_ssboCloudIndex));
        GL_CALL(glBindBuffer(GL_SHADER_STORAGE_BUFFER, gl_ssboCloudIndex));
        GL_CALL(glBufferData(GL_SHADER_STORAGE_BUFFER, gl_cloudIndexSSBO.size() * sizeof(int), gl_cloudIndexSSBO.data(), GL_DYNAMIC_DRAW));
    }
    else
    {
        GL_CALL(glBindBuffer(GL_SHADER_STORAGE_BUFFER, gl_ssboCloudIndex));
        GL_CALL(glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, gl_cloudIndexSSBO.size() * sizeof(int), gl_cloudIndexSSBO.data()));
    }

    GL_CALL(glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, gl_ssboCloudIndex));

    // Cloud attributes
    if (gl_ssboClouds == 0)
    {
        GL_CALL(glGenBuffers(1, &gl_ssboClouds));
        GL_CALL(glBindBuffer(GL_SHADER_STORAGE_BUFFER, gl_ssboClouds));
        GL_CALL(glBufferData(GL_SHADER_STORAGE_BUFFER, gl_cloudsSSBO.size() * sizeof(gl_cloudSSBO), gl_cloudsSSBO.data(), GL_DYNAMIC_DRAW));
    }
    else
    {
        GL_CALL(glBindBuffer(GL_SHADER_STORAGE_BUFFER, gl_ssboClouds));
        GL_CALL(glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, gl_cloudsSSBO.size() * sizeof(gl_cloudSSBO), gl_cloudsSSBO.data()));
    }

    GL_CALL(glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, gl_ssboClouds));

    GL_CALL(glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0));
}

void gl_updateUserView()
{
    ImGuiIO& io = ImGui::GetIO();

    Eigen::Affine3f viewLocal1 = Eigen::Affine3f::Identity();
    viewLocal1.translate(rotation_center);

    viewLocal1.translate(Eigen::Vector3f(translate_x, translate_y, translate_z));
    viewLocal1.rotate(Eigen::AngleAxisf(rotate_x * DEG_TO_RAD, Eigen::Vector3f::UnitX()));
    viewLocal1.rotate(Eigen::AngleAxisf(rotate_y * DEG_TO_RAD, Eigen::Vector3f::UnitZ()));
    viewLocal1.translate(-rotation_center);

    // Get the projection matrix from your reshape() or manually rebuild it
    float aspect = float(io.DisplaySize.x) / float(io.DisplaySize.y);
    float fov = 60.0f * DEG_TO_RAD;
    float nearf = 0.001f; // closest distance to camera objects are rendered [m]
    float farf = 1000.0f; // furthest distance to camera objects are rendered [m]

    Eigen::Matrix4f projection = Eigen::Matrix4f::Zero();
    float f = 1.0f / tan(fov / 2.0f);
    projection(0, 0) = f / aspect;
    projection(1, 1) = f;
    projection(2, 2) = (farf + nearf) / (nearf - farf);
    projection(2, 3) = (2 * farf * nearf) / (nearf - farf);
    projection(3, 2) = -1.0f;

    // Combine into a single MVP (model-view-projection)
    gl_mvp = projection * viewLocal1.matrix();
}

void gl_renderPointCloud()
{
    GL_CALL(glUseProgram(gl_shaderProgram));

    // Set point size from GUI
    GL_CALL(glUniform1f(gl_uPointSize, static_cast<float>(point_size)));
    GL_CALL(glUniform1f(gl_uIntensityScale, offset_intensity));
    GL_CALL(glUniform1i(gl_uUsePose, usePose));
    GL_CALL(glUniformMatrix4fv(gl_uMVP, 1, GL_FALSE, gl_mvp.data()));

    if (oldcolorScheme != colorScheme)
    {
        if (colorScheme == CS_SOLID)
            for (size_t i = 0; i < gl_cloudsSSBO.size(); ++i)
            {
                gl_cloudsSSBO[i].colorScheme = CS_SOLID;
                gl_cloudsSSBO[i].fixedColor[0] = session.point_clouds_container.point_clouds[0].render_color[0] + 0.1;
                gl_cloudsSSBO[i].fixedColor[1] = session.point_clouds_container.point_clouds[0].render_color[0] + 0.1;
                gl_cloudsSSBO[i].fixedColor[2] = session.point_clouds_container.point_clouds[0].render_color[0] + 0.1;
            }

        else if (colorScheme == CS_GRAD_INTENS)
            for (size_t i = 0; i < gl_cloudsSSBO.size(); ++i)
                gl_cloudsSSBO[i].colorScheme = 1;

        else if (colorScheme == CS_RANDOM)
            for (size_t i = 0; i < gl_cloudsSSBO.size(); ++i)
            {
                gl_cloudsSSBO[i].colorScheme = CS_SOLID;
                gl_cloudsSSBO[i].fixedColor[0] = float(rand() % 255) / 255.0f;
                gl_cloudsSSBO[i].fixedColor[1] = float(rand() % 255) / 255.0f;
                gl_cloudsSSBO[i].fixedColor[2] = float(rand() % 255) / 255.0f;
            }

        oldcolorScheme = colorScheme;

        gl_updateSSBOs();
    }

    GL_CALL(glBindVertexArray(VAO));
    /*for (size_t i = 0; i < gl_clouds.size(); i++)
    {
        if (!gl_clouds[i].visible) continue;

        // Send per-cloud uniforms

        if (usePose)
        {
            Eigen::Matrix4f cloudMVP = gl_mvp * gl_cloudsSSBO[i].pose;
            GL_CALL(glUniformMatrix4fv(gl_uMVP, 1, GL_FALSE, cloudMVP.data()));
        }
        else
            GL_CALL(glUniformMatrix4fv(gl_uMVP, 1, GL_FALSE, gl_mvp.data()));

        // Draw only this cloud’s range
        //GL_CALL(glDrawArrays(GL_POINTS, gl_clouds[i].offset, gl_clouds[i].count));
    }*/

    GL_CALL(glDrawArrays(GL_POINTS, 0, gl_cloudIndexSSBO.size()));

    GL_CALL(glBindVertexArray(0));
    GL_CALL(glUseProgram(0)); // back to fixed-function for legacy code
}

void gl_init()
{
    GLenum err = glewInit();
    if (err != GLEW_OK)
    {
        std::cerr << "GLEW init failed: " << glewGetErrorString(err) << std::endl;
        return;
    }

    // Compile shaders and create shader program
    gl_shaderProgram = gl_createShaderProgram(pointsVertSource, pointsFragSource);

    // Get pointers
    gl_uMVP = static_cast<GLuint>(GL_CALL_RET(glGetUniformLocation(gl_shaderProgram, "uMVP")));
    gl_uPointSize = static_cast<GLuint>(GL_CALL_RET(glGetUniformLocation(gl_shaderProgram, "uPointSize")));
    gl_uIntensityScale = static_cast<GLuint>(GL_CALL_RET(glGetUniformLocation(gl_shaderProgram, "uIntensityScale")));
    gl_uUsePose = static_cast<GLuint>(GL_CALL_RET(glGetUniformLocation(gl_shaderProgram, "uUsePose")));

    GL_CALL(glEnable(GL_PROGRAM_POINT_SIZE));
}
///////////////////////////////////////////////////////////////////////////////////




void loadSession(const std::string& session_file_name)
{
    session_loaded = session.load(fs::path(session_file_name).string(), false, 0.0, 0.0, 0.0, false);
    index_rendered_points_local = 0;

    if (session_loaded)
    {
        std::string newTitle = winTitle + " - " + truncPath(session_file_name);
        glutSetWindowTitle(newTitle.c_str());

        for (const auto& pc : session.point_clouds_container.point_clouds)
            session_total_number_of_points += pc.points_local.size();

        session_dims = session.point_clouds_container.compute_point_cloud_dimension();
    }

    if (gl_useVBOs)
    {
        // clearing previous data
        GLint offset = 0;
        gl_clouds.clear();
        gl_clouds.shrink_to_fit();
        gl_cloudIndexSSBO.clear();
        gl_cloudIndexSSBO.shrink_to_fit();
        gl_cloudsSSBO.clear();
        gl_cloudsSSBO.shrink_to_fit();

        // Convert point cloud data to gl_Points
        for (size_t j = 0; j < session.point_clouds_container.point_clouds.size(); ++j)
        {
            const auto& pc = session.point_clouds_container.point_clouds[j];

            for (size_t i = 0; i < pc.points_local.size(); ++i)
            {
                const auto& pt = pc.points_local[i];

                gl_point gp;
                gp.pos[0] = pt.x();
                gp.pos[1] = pt.y();
                gp.pos[2] = pt.z();
                gp.intensity = pc.intensities[i]; // same index

                gl_Points.push_back(gp);
                gl_cloudIndexSSBO.push_back(static_cast<int>(j)); // cloud index
            }

            gl_cloud gc;
            gc.offset = offset;
            gc.count = static_cast<GLsizei>(pc.points_local.size());
            gc.visible = true;
            gl_clouds.push_back(gc);

            gl_cloudSSBO gcSSBO;
            Eigen::Matrix4f pose_f = pc.m_pose.matrix().cast<float>();
            std::memcpy(gcSSBO.pose, pose_f.data(), 16 * sizeof(float));
            // gcSSBO.colorScheme = colorScheme;
            // gcSSBO.fixedColor[0] = pc.render_color[0];
            // gcSSBO.fixedColor[1] = pc.render_color[1];
            // gcSSBO.fixedColor[2] = pc.render_color[2];
            oldcolorScheme = CS_FOLLOW; // force update with non-sense value
            gl_cloudsSSBO.push_back(gcSSBO);

            offset += static_cast<GLint>(pc.points_local.size());
        }

        // Load to OpenGL buffers
        gl_loadPointCloudBuffer(gl_Points, VAO, VBO);

        // Clear CPU-side data after load to save memory
        gl_Points.clear();
        gl_Points.shrink_to_fit();

        // Prepare SSBO data
        gl_updateSSBOs();
    }
}

void openSession()
{
    info_gui = false;

    std::string session_file_name = "";
    session_file_name = mandeye::fd::OpenFileDialogOneFile("Open session file", mandeye::fd::Session_filter);

    if (session_file_name.size() > 0)
    {
        loadSession(session_file_name);
    }
}

void session_gui()
{
    ImGui::Text("File name:");
    ImGui::Text(session.session_file_name.c_str());
    ImGui::Text("Working directory:");
    ImGui::Text(session.working_directory.c_str());

    ImGui::Separator();

    ImGui::Text("Is ground truth: %s", session.is_ground_truth ? "yes" : "no");
    ImGui::Text("Number of clouds: %zu", session.point_clouds_container.point_clouds.size());
    ImGui::Text("Number of points: %zu", session_total_number_of_points);

    ImGui::Separator();

    ImGui::Text("Offset [m]:");
    ImGui::Text(
        "X: %.10f; Y: %.10f; Z: %.10f",
        session.point_clouds_container.offset.x(),
        session.point_clouds_container.offset.y(),
        session.point_clouds_container.offset.z());
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Click to copy to clipboard");
    if (ImGui::IsItemClicked())
    {
        char tmp[64];
        snprintf(
            tmp,
            sizeof(tmp),
            "%.10f %.10f %.10f",
            session.point_clouds_container.offset.x(),
            session.point_clouds_container.offset.y(),
            session.point_clouds_container.offset.z());
        ImGui::SetClipboardText(tmp);
    }

    ImGui::Separator();

    ImGui::Text("Dimensions:");
    if (ImGui::BeginTable("Dimensions", 4))
    {
        ImGui::TableSetupColumn("Coord [m]");
        ImGui::TableSetupColumn("min");
        ImGui::TableSetupColumn("max");
        ImGui::TableSetupColumn("size");
        ImGui::TableHeadersRow();

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);

        std::string text = "X";
        float centered = ImGui::GetColumnWidth() - ImGui::CalcTextSize(text.c_str()).x;
        // Set cursor so text is centered
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + centered * 0.5f);

        ImGui::Text("X");

        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.3f", session_dims.x_min);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%.3f", session_dims.x_max);
        ImGui::TableSetColumnIndex(3);
        ImGui::Text("%.3f", session_dims.length);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + centered * 0.5f);
        ImGui::Text("Y");

        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.3f", session_dims.y_min);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%.3f", session_dims.y_max);
        ImGui::TableSetColumnIndex(3);
        ImGui::Text("%.3f", session_dims.width);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + centered * 0.5f);
        ImGui::Text("Z");

        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.3f", session_dims.z_min);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%.3f", session_dims.z_max);
        ImGui::TableSetColumnIndex(3);
        ImGui::Text("%.3f", session_dims.height);

        ImGui::EndTable();
    }
}

void index_gui()
{
    ImGui::Text("File name:");
    ImGui::Text(session.point_clouds_container.point_clouds[index_rendered_points_local].file_name.c_str());

    ImGui::Text("Current index: %zu / %zu", index_rendered_points_local, session.point_clouds_container.point_clouds.size());

    ImGui::Separator();

    ImGui::Text("Vector sizes:");

    ImGui::Text("index_pairs         : %zu", session.point_clouds_container.point_clouds[index_rendered_points_local].index_pairs.size());
    ImGui::Text("buckets             : %zu", session.point_clouds_container.point_clouds[index_rendered_points_local].buckets.size());
    ImGui::Text("points_local        : %zu", session.point_clouds_container.point_clouds[index_rendered_points_local].points_local.size());
    ImGui::Text(
        "normal_vectors_local: %zu", session.point_clouds_container.point_clouds[index_rendered_points_local].normal_vectors_local.size());
    ImGui::Text("colors              : %zu", session.point_clouds_container.point_clouds[index_rendered_points_local].colors.size());
    ImGui::Text("points_type         : %zu", session.point_clouds_container.point_clouds[index_rendered_points_local].points_type.size());
    ImGui::Text("intensities         : %zu", session.point_clouds_container.point_clouds[index_rendered_points_local].intensities.size());
    ImGui::Text("timestamps          : %zu", session.point_clouds_container.point_clouds[index_rendered_points_local].timestamps.size());

    ImGui::Separator();

    ImGui::Text("Timestamps:");
    double ts = session.point_clouds_container.point_clouds[index_rendered_points_local].timestamps.front();

    ImGui::Text("First point: %.0f [ns]", ts);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Click to copy to clipboard");
    if (ImGui::IsItemClicked())
    {
        char tmp[64];
        snprintf(tmp, sizeof(tmp), "%.0f", ts);
        ImGui::SetClipboardText(tmp);
    }

    ts = session.point_clouds_container.point_clouds[index_rendered_points_local].timestamps.back();
    ImGui::Text("Last point : %.0f [ns]", ts);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Click to copy to clipboard");
    if (ImGui::IsItemClicked())
    {
        char tmp[64];
        snprintf(tmp, sizeof(tmp), "%.0f", ts);
        ImGui::SetClipboardText(tmp);
    }
}

void properties_gui()
{
    ImGui::Begin("Properties", &is_properties_gui, ImGuiWindowFlags_MenuBar);
    {
        if (ImGui::BeginMenuBar())
        {
            bool justPushed = false;

            if (is_session_gui)
                ImGui::PushStyleColor(ImGuiCol_Button, orangeBorder);
            if (ImGui::Button("Session"))
            {
                if (!is_session_gui)
                {
                    is_session_gui = true;
                    is_index_gui = false;
                    justPushed = true;
                }
            }
            if (is_session_gui && !justPushed)
                ImGui::PopStyleColor();
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Properties related to whole session");

            ImGui::SameLine();

            if (is_index_gui)
                ImGui::PushStyleColor(ImGuiCol_Button, orangeBorder);
            if (ImGui::Button("Index"))
            {
                if (!is_index_gui)
                {
                    is_session_gui = false;
                    is_index_gui = true;
                    justPushed = true;
                }
            }
            if (is_index_gui && !justPushed)
                ImGui::PopStyleColor();
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Properties related to current cloud index");

            ImGui::EndMenuBar();
        }

        if (is_session_gui)
            session_gui();
        if (is_index_gui)
            index_gui();
    }

    ImGui::End();
}

void display()
{
    ImGuiIO& io = ImGui::GetIO();
    glViewport(0, 0, (GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);

    glClearColor(bg_color.x * bg_color.w, bg_color.y * bg_color.w, bg_color.z * bg_color.w, bg_color.w);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    if (gl_useVBOs)
    {
        gl_updateUserView(); // this can be optimized to be called only on change (camera movement, parameters, window resize)
        gl_renderPointCloud();
    }

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);

    updateCameraTransition();

    viewLocal = Eigen::Affine3f::Identity();

    if (!is_ortho)
    {
        reshape((GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);

        viewLocal.translate(rotation_center);

        viewLocal.translate(Eigen::Vector3f(translate_x, translate_y, translate_z));
        if (!lock_z)
            viewLocal.rotate(Eigen::AngleAxisf(rotate_x * DEG_TO_RAD, Eigen::Vector3f::UnitX()));
        else
            viewLocal.rotate(Eigen::AngleAxisf(-90.0 * DEG_TO_RAD, Eigen::Vector3f::UnitX()));
        viewLocal.rotate(Eigen::AngleAxisf(rotate_y * DEG_TO_RAD, Eigen::Vector3f::UnitZ()));

        viewLocal.translate(-rotation_center);

        glLoadMatrixf(viewLocal.matrix().data());
    }
    else
        updateOrthoView();

    showAxes();

    if (index_rendered_points_local >= 0 &&
        index_rendered_points_local < session.point_clouds_container.point_clouds[index_rendered_points_local].points_local.size())
    {
        const auto& cloud = session.point_clouds_container.point_clouds[index_rendered_points_local]; // avoiding multiple indexing

        const double inv_max_intensity = 1.0 / *std::max_element(cloud.intensities.begin(), cloud.intensities.end()); // precompute for
                                                                                                                      // speed

        Eigen::Affine3d pose = cloud.m_pose;

        if (usePose == false)
            pose.translation().setZero();

        glPointSize(point_size);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < cloud.points_local.size(); i++)
        {
            if (colorScheme == CS_SOLID) // solid color
            {
                glColor3f(cloud.render_color[0], cloud.render_color[1], cloud.render_color[2]);
            }
            else if (colorScheme == CS_GRAD_INTENS) // intensity gradient
            {
                const double norm = cloud.intensities[i] * inv_max_intensity + offset_intensity;
                glColor3f(norm, 0.0, 1.0 - norm);
            }

            Eigen::Vector3d p(cloud.points_local[i].x(), cloud.points_local[i].y(), cloud.points_local[i].z());
            p = pose * p;
            glVertex3f(p.x(), p.y(), p.z());
        }
        glEnd();

        if (show_neighbouring_scans)
        {
            glColor3f(pc_neigbouring_color.x, pc_neigbouring_color.y, pc_neigbouring_color.z);
            glPointSize(point_size);
            glBegin(GL_POINTS);
            for (int index = index_rendered_points_local - 20; index <= index_rendered_points_local + 20; index += 5)
            {
                if (index != index_rendered_points_local && index >= 0 && index < session.point_clouds_container.point_clouds.size())
                {
                    const auto& iCloud = session.point_clouds_container.point_clouds[index]; // avoiding multiple indexing
                    Eigen::Affine3d pose = iCloud.m_pose;

                    if (usePose == false)
                    {
                        pose(0, 3) -= cloud.m_pose(0, 3);
                        pose(1, 3) -= cloud.m_pose(1, 3);
                        pose(2, 3) -= cloud.m_pose(2, 3);
                    }

                    for (size_t i = 0; i < iCloud.points_local.size(); i++)
                    {
                        Eigen::Vector3d p(iCloud.points_local[i].x(), iCloud.points_local[i].y(), iCloud.points_local[i].z());
                        p = pose * p;
                        glVertex3f(p.x(), p.y(), p.z());
                    }
                }
            }
            glEnd();
        }
    }

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGLUT_NewFrame();
    ImGui::NewFrame();

    ShowMainDockSpace();

    view_kbd_shortcuts();

    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_O, false))
    {
        openSession();

        // workaround
        io.AddKeyEvent(ImGuiKey_O, false);
        io.AddKeyEvent(ImGuiMod_Ctrl, false);
    }

    if (session_loaded)
    {
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_P, false))
        {
            is_properties_gui = !is_properties_gui;

            // workaround
            io.AddKeyEvent(ImGuiKey_P, false);
            io.AddKeyEvent(ImGuiMod_Ctrl, false);
        }
    }

    if (session.point_clouds_container.point_clouds.size() > 0)
    {
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_N, false))
        {
            show_neighbouring_scans = !show_neighbouring_scans;

            // workaround
            io.AddKeyEvent(ImGuiKey_N, false);
            io.AddKeyEvent(ImGuiMod_Ctrl, false);
        }

        if (ImGui::IsKeyPressed(ImGuiKey_UpArrow, true))
            offset_intensity += 0.01;
        if (ImGui::IsKeyPressed(ImGuiKey_DownArrow, true))
            offset_intensity -= 0.01;

        if (offset_intensity < 0)
            offset_intensity = 0;
        else if (offset_intensity > 1)
            offset_intensity = 1;

        if ((!io.KeyCtrl && !io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_RightArrow, true)) || ImGui::IsKeyPressed(ImGuiKey_PageUp, true) ||
            ImGui::IsKeyPressed(ImGuiKey_KeypadAdd, true))
            index_rendered_points_local += 1;
        if ((!io.KeyCtrl && !io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_LeftArrow, true)) ||
            ImGui::IsKeyPressed(ImGuiKey_PageDown, true) || ImGui::IsKeyPressed(ImGuiKey_KeypadSubtract, true))
            index_rendered_points_local -= 1;

        if (index_rendered_points_local < 0)
            index_rendered_points_local = 0;
        if (index_rendered_points_local >= session.point_clouds_container.point_clouds.size())
            index_rendered_points_local = session.point_clouds_container.point_clouds.size() - 1;
    }

    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::Button("Open session"))
            openSession();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Select session to open for analyze (Ctrl+O)");

        ImGui::SameLine();
        ImGui::Dummy(ImVec2(20, 0));
        ImGui::SameLine();

        if (ImGui::BeginMenu("View"))
        {
            ImGui::BeginDisabled(!(session.point_clouds_container.point_clouds.size() > 0));
            {
                auto tmp = point_size;
                ImGui::SetNextItemWidth(ImGuiNumberWidth);
                ImGui::InputInt("Points size", &point_size);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("keyboard 1-9 keys");
                if (point_size < 1)
                    point_size = 1;
                else if (point_size > 10)
                    point_size = 10;

                if (tmp != point_size)
                    for (auto& point_cloud : session.point_clouds_container.point_clouds)
                        point_cloud.point_size = point_size;

                // ImGui::MenuItem("show_imu_to_lio_diff", nullptr, &session.point_clouds_container.show_imu_to_lio_diff);

                ImGui::Separator();
            }
            ImGui::EndDisabled();

            if (ImGui::MenuItem("Orthographic", "key O", &is_ortho))
            {
                if (is_ortho)
                {
                    new_rotation_center = rotation_center;
                    new_rotate_x = 0.0;
                    new_rotate_y = 0.0;
                    new_translate_x = translate_x;
                    new_translate_y = translate_y;
                    new_translate_z = translate_z;
                    camera_transition_active = true;
                }
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Switch between perspective view (3D) and orthographic view (2D/flat)");

            ImGui::MenuItem("Show axes", "key X", &show_axes);
            ImGui::MenuItem("Show compass/ruler", "key C", &compass_ruler);

            ImGui::MenuItem("Lock Z", "Shift + Z", &lock_z, !is_ortho);

            ImGui::MenuItem("Use segment pose", nullptr, &usePose);
            ImGui::MenuItem("Show neighbouring scans", "Ctrl+N", &show_neighbouring_scans);

            // ImGui::MenuItem("show_covs", nullptr, &show_covs);

            ImGui::Separator();

            ImGui::MenuItem("VBO/VAO proof of concept", nullptr, &gl_useVBOs);
            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                ImGui::Text("Has to be enabled before loading session so buffers are created");
                ImGui::Text("Should be tested with smaller sessions as session will be rendered in full, no decimation");
                ImGui::Text("Should be used with 'Use segment pose' active for relevant session view");
                ImGui::Text("Proof of concept! Not fully functional");

                ImGui::EndTooltip();
            }

            if (gl_useVBOs)
                usePose = true; // forces usePose when using VBOs for now

            ImGui::Separator();

            if (ImGui::BeginMenu("Colors"))
            {
                ImGui::ColorEdit3("Background", (float*)&bg_color, ImGuiColorEditFlags_NoInputs);

                ImGui::BeginDisabled(!(session.point_clouds_container.point_clouds.size() > 0));
                {
                    ImGui::BeginDisabled(!show_neighbouring_scans);
                    {
                        ImGui::ColorEdit3("Neighbours", (float*)&pc_neigbouring_color, ImGuiColorEditFlags_NoInputs);
                    }
                    ImGui::EndDisabled();

                    ImGui::Separator();

                    ImGui::Text("Point cloud:");

                    float color[3];
                    if (session.point_clouds_container.point_clouds.size() > 0)
                    {
                        color[0] = session.point_clouds_container.point_clouds[0].render_color[0];
                        color[1] = session.point_clouds_container.point_clouds[0].render_color[1];
                        color[2] = session.point_clouds_container.point_clouds[0].render_color[2];
                    }

                    if (ImGui::ColorEdit3("", (float*)&color, ImGuiColorEditFlags_NoInputs))
                    {
                        colorScheme = CS_SOLID;

                        for (auto& pc : session.point_clouds_container.point_clouds)
                        {
                            pc.render_color[0] = color[0];
                            pc.render_color[1] = color[1];
                            pc.render_color[2] = color[2];
                        }
                    }
                    ImGui::SameLine();
                    if (ImGui::MenuItem("> Solid color", nullptr, (colorScheme == CS_SOLID)))
                        colorScheme = CS_SOLID;

                    if (ImGui::MenuItem("> Intensity gradient", nullptr, (colorScheme == CS_GRAD_INTENS)))
                        colorScheme = CS_GRAD_INTENS;
                    ImGui::SetNextItemWidth(ImGuiNumberWidth);
                    ImGui::InputFloat("Offset intensity", &offset_intensity, 0.01, 0.1, "%.2f");
                    if (offset_intensity < 0)
                        offset_intensity = 0;
                    else if (offset_intensity > 1)
                        offset_intensity = 1;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("keyboard up/down arrows");

                    if (ImGui::MenuItem("> Random color per segment", nullptr, (colorScheme == CS_RANDOM)))
                        colorScheme = CS_RANDOM;
                }
                ImGui::EndDisabled();

                ImGui::EndMenu();
            }

            ImGui::Separator();

            ImGui::MenuItem("Properties", "Ctrl+P", &is_properties_gui, session_loaded);

            if (ImGui::BeginMenu("Console"))
            {
#ifdef _WIN32

                if (ImGui::MenuItem("Use Windows console", nullptr, &consWin))
                {
                    if (consWin)
                    {
                        AllocConsole();
                        freopen("CONOUT$", "w", stdout);
                        freopen("CONOUT$", "w", stderr);
                        freopen("CONIN$", "r", stdin);
                    }
                    else
                        FreeConsole();
                }
                if (ImGui::IsItemHovered())
                {
                    ImGui::BeginTooltip();
                    ImGui::Text("!!! If not used.. !!!");
                    ImGui::Text("- old console output is lost");
                    ImGui::Text("- new console output can only be seen in subwindow");
                    ImGui::Text("- app might run faster");
                    ImGui::EndTooltip();
                }
#endif
                ImGui::MenuItem("Subwindow", nullptr, &consImGui);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Show/hide console output as GUI window");

                ImGui::EndMenu();
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Control console output");

            ImGui::EndMenu();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Scene view relevant parameters");

        camMenu();

        ImGui::SameLine();
        ImGui::Dummy(ImVec2(20, 0));
        ImGui::SameLine();

        if (session.point_clouds_container.point_clouds.size() > 0)
        {
            int tempIndex = index_rendered_points_local;
            ImGui::Text("Index: ");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Index of rendered point cloud");
            ImGui::SameLine();
            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::SliderInt("##irpls", &tempIndex, 0, static_cast<int>(session.point_clouds_container.point_clouds.size() - 1));
            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                ImGui::Text(session.point_clouds_container.point_clouds[index_rendered_points_local].file_name.c_str());
                double ts = (session.point_clouds_container.point_clouds[index_rendered_points_local].timestamps[0] -
                             session.point_clouds_container.point_clouds[0].timestamps[0]) /
                    1e9;
                ImGui::Text("Delta 1st points timestamp [s]: %.6f", ts);
                ImGui::NewLine();
                ImGui::Text("Check Properties (Ctrl+P) for more info");
                ImGui::EndTooltip();
            }
            ImGui::SameLine();
            ImGui::InputInt("##irpli", &tempIndex, 1, 10);
            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                ImGui::Text(session.point_clouds_container.point_clouds[index_rendered_points_local].file_name.c_str());
                double ts = (session.point_clouds_container.point_clouds[index_rendered_points_local].timestamps[0] -
                             session.point_clouds_container.point_clouds[0].timestamps[0]) /
                    1e9;
                ImGui::Text("Delta 1st points timestamp [s]: %.6f", ts);
                ImGui::NewLine();
                ImGui::Text("Check Properties (Ctrl+P) for more info");
                ImGui::EndTooltip();
            }
            ImGui::PopItemWidth();

            if ((tempIndex >= 0) && (tempIndex < session.point_clouds_container.point_clouds.size()))
                index_rendered_points_local = tempIndex;
        }

        ImGui::SameLine();
        ImGui::Text("(%.1f FPS)", ImGui::GetIO().Framerate);

        ImGui::SameLine(
            ImGui::GetWindowWidth() - ImGui::CalcTextSize("Info").x - ImGui::GetStyle().ItemSpacing.x * 2 -
            ImGui::GetStyle().FramePadding.x * 2);

        ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4, 2));
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetStyleColorVec4(ImGuiCol_HeaderHovered));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImGui::GetStyleColorVec4(ImGuiCol_Header));
        if (ImGui::SmallButton("Info"))
            info_gui = !info_gui;

        ImGui::PopStyleVar(2);
        ImGui::PopStyleColor(3);

        ImGui::EndMainMenuBar();
    }

    if (is_properties_gui)
        properties_gui();

    if (consImGui)
    {
        if (!consHooked)
            ConsoleHook();
        ImGuiConsole(&consImGui);
    }
    else
    {
        if (consHooked)
            ConsoleUnhook();
    }

    cor_window();

    info_window(infoLines, appShortcuts);

    if (compass_ruler)
        drawMiniCompassWithRuler();

    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

    glutSwapBuffers();
    glutPostRedisplay();
}

void mouse(int glut_button, int state, int x, int y)
{
    ImGuiIO& io = ImGui::GetIO();
    io.MousePos = ImVec2((float)x, (float)y);

    int button = -1;
    if (glut_button == GLUT_LEFT_BUTTON)
        button = 0;
    if (glut_button == GLUT_RIGHT_BUTTON)
        button = 1;
    if (glut_button == GLUT_MIDDLE_BUTTON)
        button = 2;
    if (button != -1 && state == GLUT_DOWN)
        io.MouseDown[button] = true;
    if (button != -1 && state == GLUT_UP)
        io.MouseDown[button] = false;

    static int glutMajorVersion = glutGet(GLUT_VERSION) / 10000;
    if (state == GLUT_DOWN && (glut_button == 3 || glut_button == 4) && glutMajorVersion < 3)
        wheel(glut_button, glut_button == 3 ? 1 : -1, x, y);

    if (!io.WantCaptureMouse)
    {
        if ((glut_button == GLUT_MIDDLE_BUTTON || glut_button == GLUT_RIGHT_BUTTON) && state == GLUT_DOWN && io.KeyCtrl)
        {
            if (session_loaded)
            {
                const auto laser_beam = GetLaserBeam(x, y);
                double min_distance = std::numeric_limits<double>::max();

                for (size_t j = 0; j < session.point_clouds_container.point_clouds[index_rendered_points_local].points_local.size(); j++)
                {
                    auto vp = session.point_clouds_container.point_clouds[index_rendered_points_local].points_local[j];

                    if (usePose == true)
                        vp = session.point_clouds_container.point_clouds[index_rendered_points_local].m_pose * vp;

                    double dist = distance_point_to_line(vp, laser_beam);

                    if (dist < min_distance && dist < 0.1)
                    {
                        min_distance = dist;

                        new_rotation_center.x() = vp.x();
                        new_rotation_center.y() = vp.y();
                        new_rotation_center.z() = vp.z();
                    }
                }

                new_rotate_x = rotate_x;
                new_rotate_y = rotate_y;
                new_translate_x = -new_rotation_center.x();
                new_translate_y = -new_rotation_center.y();
                new_translate_z = translate_z;
                camera_transition_active = true;
            }
            else
                setNewRotationCenter(x, y);
        }

        if (state == GLUT_DOWN)
            mouse_buttons |= 1 << glut_button;
        else if (state == GLUT_UP)
            mouse_buttons = 0;

        mouse_old_x = x;
        mouse_old_y = y;
    }
}

int main(int argc, char* argv[])
{
    try
    {
        if (checkClHelp(argc, argv))
        {
            std::cout << winTitle << "\n\n"
                      << "USAGE:\n"
                      << std::filesystem::path(argv[0]).stem().string() << " <input_file> /?\n\n"
                      << "where\n"
                      << "   <input_file>         Path to Mandeye JSON Session file (*.mjs)\n"
                      << "   -h, /h, --help, /?   Show this help and exit\n\n";

            return 0;
        }

        initGL(&argc, argv, winTitle, display, mouse);
        gl_init();

        if (argc > 1)
        {
            for (int i = 1; i < argc; i++)
            {
                std::string ext = fs::path(argv[i]).extension().string();
                std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

                if (ext == ".mjs")
                {
                    loadSession(argv[i]);

                    break;
                }
            }
        }

        glutMainLoop();

        ImGui_ImplOpenGL2_Shutdown();
        ImGui_ImplGLUT_Shutdown();
        ImGui::DestroyContext();
    } catch (const std::bad_alloc& e)
    {
        std::cerr << "System is out of memory : " << e.what() << std::endl;
        mandeye::fd::OutOfMemMessage();
    } catch (const std::exception& e)
    {
        std::cout << e.what();
    } catch (...)
    {
        std::cerr << "Unknown fatal error occurred." << std::endl;
    }

    return 0;
}

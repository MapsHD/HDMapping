#include "overlay_layers.h"

#include <raylib.h>

#include <Core/e57_utils.h>
#include <Core/pfd_wrapper.hpp>
#include <Core/point_cloud.h>
#include <RaylibWidgets/PointBufferParts.h>
#include <RaylibWidgets/Shaders.h>

#include <imgui.h>
#include <laszip/laszip_api.h>
#include <raymath.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace step1_render
{
    namespace
    {
        namespace fs = std::filesystem;

        struct LoadSettings
        {
            int budget = 1000000;
            double spacing = 0;
        };

        struct LoadProgress
        {
            std::atomic<bool> cancel{ false };
            std::atomic<uint64_t> done{ 0 }, total{ 0 };
        };

        struct Layer
        {
            PointCloud cloud;
            std::vector<raylib_widgets::PointBufferPart> gpu;
            std::vector<unsigned int> indexBuffers;
            int indexedStride = 0;
            Eigen::Vector3d minimum = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
            Eigen::Vector3d maximum = -minimum;
            Eigen::Vector3d offset = Eigen::Vector3d::Zero(), editedOffset = Eigen::Vector3d::Zero();
            fs::path path;
            char name[256]{};
            uint64_t sourceCount = 0;
            bool trajectory = false, rgb = true, intensity = true, visible = true, dirty = true;
            bool points = true, lines = false, shade = true;
            int mode = 0, palette = 1, stride = 1;
            float size = 3, color[3] = { 0, 0.8f, 1 };
            double spacing = 0;
            float intensityMin = 0, intensityMax = 1;

            Eigen::Vector3d center() const
            {
                return minimum * 0.5 + maximum * 0.5;
            }
        };

        struct LoadResult
        {
            std::unique_ptr<Layer> layer;
            std::string error;
        };

        void clearIndices(Layer& layer)
        {
            for (auto buffer : layer.indexBuffers)
                rlUnloadVertexBuffer(buffer);
            layer.indexBuffers.clear();
            layer.indexedStride = 0;
        }

        void clearGpu(Layer& layer)
        {
            clearIndices(layer);
            raylib_widgets::unloadPointBufferParts(layer.gpu);
        }

        std::string lower(std::string text)
        {
            for (char& c : text)
                c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            return text;
        }

        bool supported(const fs::path& path)
        {
            const auto ext = lower(path.extension().string());
            return ext == ".las" || ext == ".laz" || ext == ".e57" || ext == ".xyz" || ext == ".txt" || ext == ".tum" || ext == ".csv";
        }

        void controlLabel(const char* text)
        {
            ImGui::TextUnformatted(text);
            ImGui::SetNextItemWidth(-1);
        }

        void offsetInput(Eigen::Vector3d& offset)
        {
            const char* labels[] = { "Offset X [m]", "Offset Y [m]", "Offset Z [m]" };
            for (int axis = 0; axis < 3; ++axis)
            {
                ImGui::PushID(axis);
                controlLabel(labels[axis]);
                ImGui::InputDouble("##offset", &offset[axis], 0, 0, "%.6f");
                ImGui::PopID();
            }
        }

        void cancelled(const LoadProgress& progress)
        {
            if (progress.cancel.load(std::memory_order_relaxed))
                throw std::runtime_error("Import cancelled");
        }

        struct CellHash
        {
            size_t operator()(const std::array<int64_t, 3>& key) const
            {
                size_t h = 0;
                for (int64_t n : key)
                    h ^= std::hash<int64_t>{}(n) + size_t(0x9e3779b9) + (h << 6) + (h >> 2);
                return h;
            }
        };

        class Sampler
        {
        public:
            Sampler(Layer& layer, const LoadSettings& settings, uint64_t expected)
                : layer_(layer)
                , settings_(settings)
                , stride_(std::max<uint64_t>(1, expected / settings.budget + (expected % settings.budget != 0)))
            {
            }

            void append(const Eigen::Vector3d& point, const Eigen::Vector3d* color, const unsigned short* intensity)
            {
                if (!point.allFinite())
                    throw std::runtime_error("Non-finite coordinates in input");
                layer_.minimum = layer_.minimum.cwiseMin(point);
                layer_.maximum = layer_.maximum.cwiseMax(point);
                const auto index = layer_.sourceCount++;
                if (index % stride_ || layer_.cloud.points_local.size() >= static_cast<size_t>(settings_.budget))
                    return;
                if (settings_.spacing > 0)
                {
                    if (!origin_)
                        origin_ = point;
                    const Eigen::Vector3d scaled = ((point - *origin_) / settings_.spacing).array().floor();
                    if (!scaled.allFinite() || scaled.cwiseAbs().maxCoeff() >= 9e18)
                        throw std::runtime_error("Sampling cell coordinates exceed supported range; increase spacing");
                    const std::array<int64_t, 3> key{ static_cast<int64_t>(scaled.x()),
                                                      static_cast<int64_t>(scaled.y()),
                                                      static_cast<int64_t>(scaled.z()) };
                    if (!occupied_.insert(key).second)
                        return;
                }
                layer_.cloud.points_local.push_back(point);
                layer_.cloud.colors.push_back(color ? *color : Eigen::Vector3d::Ones().eval());
                layer_.cloud.intensities.push_back(intensity ? *intensity : 0);
                layer_.rgb &= color != nullptr;
                layer_.intensity &= intensity != nullptr;
            }

        private:
            Layer& layer_;
            LoadSettings settings_;
            uint64_t stride_;
            std::optional<Eigen::Vector3d> origin_;
            std::unordered_set<std::array<int64_t, 3>, CellHash> occupied_;
        };

        struct LasReader
        {
            laszip_POINTER handle = nullptr;
            bool opened = false;
            ~LasReader()
            {
                if (opened)
                    laszip_close_reader(handle);
                if (handle)
                    laszip_destroy(handle);
            }
        };

        // Session loaders retain complete inputs. Stream display data to enforce the point budget and cancellation.
        void readLas(Layer& layer, const LoadSettings& settings, LoadProgress& progress)
        {
            LasReader reader;
            laszip_BOOL compressed = 0;
            if (laszip_create(&reader.handle) || laszip_open_reader(reader.handle, layer.path.string().c_str(), &compressed))
                throw std::runtime_error("Cannot open LAS/LAZ file");
            reader.opened = true;
            laszip_header* header = nullptr;
            laszip_point* p = nullptr;
            if (laszip_get_header_pointer(reader.handle, &header) || laszip_get_point_pointer(reader.handle, &p))
                throw std::runtime_error("Cannot read LAS/LAZ header");
            const uint64_t count =
                header->extended_number_of_point_records ? header->extended_number_of_point_records : header->number_of_point_records;
            progress.total = count;
            const int format = header->point_data_format & 63;
            const bool rgb = format == 2 || format == 3 || format == 5 || format == 7 || format == 8 || format == 10;
            Sampler sampler(layer, settings, count);
            for (uint64_t i = 0; i < count; ++i)
            {
                if ((i & 1023) == 0)
                {
                    cancelled(progress);
                    progress.done = i;
                }
                if (laszip_read_point(reader.handle))
                    throw std::runtime_error("LAS/LAZ read failed at point " + std::to_string(i));
                const Eigen::Vector3d position(
                    header->x_offset + header->x_scale_factor * p->X,
                    header->y_offset + header->y_scale_factor * p->Y,
                    header->z_offset + header->z_scale_factor * p->Z);
                const Eigen::Vector3d color(p->rgb[0] / 65535.0, p->rgb[1] / 65535.0, p->rgb[2] / 65535.0);
                sampler.append(position, rgb ? &color : nullptr, &p->intensity);
            }
            progress.done = count;
        }

        using PointVisitor = std::function<void(const Eigen::Vector3d&)>;

        // Preserve file order and bounded retained memory; the shared TUM loader materializes and sorts all poses.
        uint64_t readText(const fs::path& path, LoadProgress& progress, const PointVisitor& visit)
        {
            std::ifstream input(path);
            if (!input)
                throw std::runtime_error("Cannot open trajectory file");
            const bool tum = lower(path.extension().string()) == ".tum";
            std::array<size_t, 3> columns{ tum ? 1u : 0u, tum ? 2u : 1u, tum ? 3u : 2u };
            bool header = false;
            uint64_t count = 0, lineNumber = 0;
            std::string line;
            while (std::getline(input, line))
            {
                ++lineNumber;
                if ((lineNumber & 1023) == 1)
                    cancelled(progress);
                progress.done.fetch_add(line.size() + 1, std::memory_order_relaxed);
                const auto comment = line.find('#');
                if (comment != std::string::npos)
                    line.resize(comment);
                for (char& c : line)
                    if (c == ',' || c == ';')
                        c = ' ';
                std::istringstream stream(line);
                std::vector<std::string> fields;
                for (std::string token; stream >> token;)
                    fields.push_back(token);
                if (fields.empty())
                    continue;
                if (!count && !header && !tum)
                {
                    std::array<size_t, 3> found{ fields.size(), fields.size(), fields.size() };
                    for (size_t i = 0; i < fields.size(); ++i)
                    {
                        const auto token = lower(fields[i]);
                        if (token == "x")
                            found[0] = i;
                        if (token == "y")
                            found[1] = i;
                        if (token == "z")
                            found[2] = i;
                    }
                    if (*std::max_element(found.begin(), found.end()) < fields.size())
                    {
                        columns = found;
                        header = true;
                        continue;
                    }
                }
                try
                {
                    if ((tum && fields.size() != 8) || (!tum && !header && fields.size() != 3))
                        throw std::runtime_error("Expected XYZ, a header with x/y/z columns, or an eight-column .tum file");
                    Eigen::Vector3d point;
                    for (size_t axis = 0; axis < 3; ++axis)
                    {
                        const auto& token = fields.at(columns[axis]);
                        size_t consumed = 0;
                        point[axis] = std::stod(token, &consumed);
                        if (consumed != token.size() || !std::isfinite(point[axis]))
                            throw std::runtime_error("Invalid coordinate");
                    }
                    visit(point);
                } catch (const std::exception& error)
                {
                    throw std::runtime_error("Trajectory line " + std::to_string(lineNumber) + ": " + error.what());
                }
                ++count;
            }
            if (input.bad())
                throw std::runtime_error("Trajectory read failed");
            return count;
        }

        LoadResult loadLayer(const fs::path& path, LoadSettings settings, const std::shared_ptr<LoadProgress>& progress)
        {
            try
            {
                auto layer = std::make_unique<Layer>();
                layer->path = path;
                layer->spacing = settings.spacing;
                std::snprintf(layer->name, sizeof(layer->name), "%s", path.filename().string().c_str());
                const auto ext = lower(path.extension().string());
                if (ext == ".las" || ext == ".laz")
                    readLas(*layer, settings, *progress);
                else if (ext == ".e57")
                {
                    std::vector<mandeye::e57io::E57Scan> scans;
                    std::string error;
                    if (!mandeye::e57io::load_e57(path.string(), scans, error))
                        throw std::runtime_error(error);
                    uint64_t count = 0;
                    for (const auto& scan : scans)
                        count += scan.points.size();
                    progress->total = count;
                    Sampler sampler(*layer, settings, count);
                    for (const auto& scan : scans)
                        for (size_t i = 0; i < scan.points.size(); ++i)
                        {
                            if ((i & 1023) == 0)
                                cancelled(*progress);
                            sampler.append(
                                scan.pose * scan.points[i],
                                scan.colors.size() == scan.points.size() ? &scan.colors[i] : nullptr,
                                scan.intensities.size() == scan.points.size() ? &scan.intensities[i] : nullptr);
                            progress->done.fetch_add(1, std::memory_order_relaxed);
                        }
                }
                else
                {
                    layer->trajectory = layer->lines = true;
                    layer->shade = false;
                    progress->total = fs::file_size(path) * 2;
                    const auto count = readText(
                        path,
                        *progress,
                        [](const Eigen::Vector3d&)
                        {
                        });
                    Sampler sampler(*layer, settings, count);
                    readText(
                        path,
                        *progress,
                        [&](const Eigen::Vector3d& point)
                        {
                            sampler.append(point, nullptr, nullptr);
                        });
                }
                cancelled(*progress);
                if (layer->cloud.points_local.empty())
                    throw std::runtime_error("No points retained");
                if (layer->intensity)
                {
                    const auto [lo, hi] = std::minmax_element(layer->cloud.intensities.begin(), layer->cloud.intensities.end());
                    layer->intensityMin = *lo;
                    layer->intensityMax = *hi;
                }
                layer->shade &= layer->intensity;
                return { std::move(layer), {} };
            } catch (const std::exception& error)
            {
                return { nullptr, path.filename().string() + ": " + error.what() };
            }
        }

        // ScanRenderer has no native RGB or independent intensity shading for each color mode.
        // Reuse the shared buffer parts and GLSL palettes with this position/RGB/intensity layout.
        constexpr const char* vertexShader = R"(#version 330
layout(location=0) in vec3 vertexPosition;
layout(location=1) in vec3 vertexColor;
layout(location=2) in float vertexIntensity;
uniform mat4 mvp;
uniform float pointSize;
out vec3 nativeColor;
out float intensity;
out float elevation;
void main() {
    gl_Position = mvp * vec4(vertexPosition, 1.0);
    gl_PointSize = pointSize;
    nativeColor = vertexColor;
    intensity = vertexIntensity;
    elevation = vertexPosition.z;
})";

        const std::string fragmentShader = std::string(R"(#version 330
in vec3 nativeColor;
in float intensity;
in float elevation;
uniform vec3 layerColor;
uniform int colorMode;
uniform int palette;
uniform int shadeIntensity;
uniform vec2 elevationRange;
out vec4 finalColor;
)") + raylib_widgets::kColormapsGLSL +
            R"(
void main() {
    vec3 color = layerColor;
    if (colorMode == 1) color = nativeColor;
    if (colorMode == 2) color = applyColormap(palette, intensity);
    if (colorMode == 3) color = applyColormap(palette, clamp((elevation-elevationRange.x) / max(elevationRange.y-elevationRange.x, 1e-6), 0.0, 1.0));
    if (shadeIntensity != 0) color *= 0.35 + 0.65 * sqrt(clamp(intensity, 0.0, 1.0));
    finalColor = vec4(color, 1.0);
})";
    } // namespace

    struct OverlayLayers::Impl
    {
        enum class Phase
        {
            Idle,
            Settings,
            Loading,
            Confirm
        };
        std::vector<std::unique_ptr<Layer>> layers;
        std::deque<fs::path> pending;
        std::future<LoadResult> loading;
        std::shared_ptr<LoadProgress> progress;
        std::unique_ptr<Layer> incoming;
        std::optional<Eigen::Vector3d> commonOffset;
        LoadSettings settings;
        Phase phase = Phase::Idle;
        bool visible = true, collapsed = true, initialPresentation = true, expandRequested = false;
        bool openPopup = false, reuseOffset = true;
        std::string message, currentPath;
        Shader shader{};
        std::array<int, 7> uniforms{};
        bool shaderAttempted = false;

        void start()
        {
            const auto path = pending.front();
            pending.pop_front();
            currentPath = path.filename().string();
            progress = std::make_shared<LoadProgress>();
            loading = std::async(std::launch::async, loadLayer, path, settings, progress);
            phase = Phase::Loading;
        }

        void next()
        {
            incoming.reset();
            phase = pending.empty() ? Phase::Idle : Phase::Settings;
        }

        void poll()
        {
            if (loading.valid() && loading.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
            {
                auto result = loading.get();
                if (!result.layer)
                {
                    message = result.error;
                    next();
                    return;
                }
                incoming = std::move(result.layer);
                incoming->editedOffset = commonOffset.value_or(
                    incoming->center().cwiseAbs().maxCoeff() >= 10000 ? incoming->center() : Eigen::Vector3d::Zero().eval());
                phase = Phase::Confirm;
            }
        }

        void importPopup()
        {
            if (openPopup)
            {
                ImGui::OpenPopup("Import overlay");
                openPopup = false;
            }
            ImGui::SetNextWindowSize(ImVec2(570, 0), ImGuiCond_Appearing);
            if (!ImGui::BeginPopupModal("Import overlay", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
                return;
            if (phase == Phase::Idle)
                ImGui::CloseCurrentPopup();
            else if (phase == Phase::Settings)
            {
                ImGui::TextWrapped("%s", pending.front().string().c_str());
                ImGui::Text("Files queued: %zu", pending.size());
                controlLabel("Maximum retained points");
                ImGui::InputInt("##point-budget", &settings.budget, 100000, 1000000);
                settings.budget = std::clamp(settings.budget, 100, 5000000);
                controlLabel("Load spacing [m]");
                ImGui::InputDouble("##load-spacing", &settings.spacing, 0.01, 0.1, "%.3f");
                if (!std::isfinite(settings.spacing) || settings.spacing < 0)
                    settings.spacing = 0;
                ImGui::TextWrapped("Budget sampling is applied first, then optional spatial spacing. Sources are unchanged.");
                if (lower(pending.front().extension().string()) == ".e57")
                    ImGui::TextWrapped(
                        "E57 uses the existing full-file decoder before sampling. Peak RAM is not bounded by the retained-point budget; "
                        "cancellation waits for decoding to finish.");
                if (ImGui::Button("Load"))
                    start();
                ImGui::SameLine();
                if (ImGui::Button("Cancel batch"))
                {
                    pending.clear();
                    phase = Phase::Idle;
                    ImGui::CloseCurrentPopup();
                }
            }
            else if (phase == Phase::Loading)
            {
                ImGui::TextWrapped("Loading %s", currentPath.c_str());
                const auto total = progress->total.load();
                ImGui::ProgressBar(total ? std::min(1.f, static_cast<float>(progress->done.load()) / total) : 0.f);
                if (progress->cancel)
                    ImGui::TextUnformatted("Cancelling...");
                else if (ImGui::Button("Cancel batch"))
                {
                    pending.clear();
                    progress->cancel = true;
                }
            }
            else if (phase == Phase::Confirm)
            {
                auto& layer = *incoming;
                ImGui::TextWrapped("%s", layer.path.string().c_str());
                ImGui::Text(
                    "Source: %llu | Retained: %zu", static_cast<unsigned long long>(layer.sourceCount), layer.cloud.points_local.size());
                ImGui::TextUnformatted("Confirm coordinate offset before displaying this layer:");
                ImGui::TextUnformatted("display_position = source_position - offset");
                offsetInput(layer.editedOffset);
                if (ImGui::Button("Bounding-box center"))
                    layer.editedOffset = layer.center();
                if (ImGui::Button("Keep original coordinates"))
                    layer.editedOffset.setZero();
                if (commonOffset && ImGui::Button("Reuse previous offset"))
                    layer.editedOffset = *commonOffset;
                ImGui::Checkbox("Reuse this offset for following layers", &reuseOffset);
                ImGui::TextWrapped(
                    "Related clouds and trajectories need the same offset to preserve their relative placement. This does not register or "
                    "reproject data.");
                ImGui::BeginDisabled(!layer.editedOffset.allFinite());
                if (ImGui::Button("Add layer"))
                {
                    layer.offset = layer.editedOffset;
                    if (reuseOffset)
                        commonOffset = layer.offset;
                    layers.push_back(std::move(incoming));
                    next();
                }
                ImGui::EndDisabled();
                ImGui::SameLine();
                if (ImGui::Button("Skip file"))
                    next();
                ImGui::SameLine();
                if (ImGui::Button("Cancel batch"))
                {
                    pending.clear();
                    next();
                }
                if (phase == Phase::Idle)
                    ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        void upload(Layer& layer)
        {
            std::vector<float> vertices;
            vertices.resize(layer.cloud.points_local.size() * 7);
            const float range = std::max(1.f, layer.intensityMax - layer.intensityMin);
            for (size_t i = 0; i < layer.cloud.points_local.size(); ++i)
            {
                const Eigen::Vector3d p = layer.cloud.points_local[i] - layer.offset;
                const Eigen::Vector3d& color = layer.cloud.colors[i];
                if (!p.cast<float>().allFinite())
                    throw std::runtime_error("Overlay coordinates exceed GPU range; choose a finite nearby offset");
                Eigen::Map<Eigen::Vector3f>(vertices.data() + i * 7) = p.cast<float>();
                Eigen::Map<Eigen::Vector3f>(vertices.data() + i * 7 + 3) = color.cast<float>();
                vertices[i * 7 + 6] = (layer.cloud.intensities[i] - layer.intensityMin) / range;
            }
            auto parts = raylib_widgets::uploadPointBufferParts(vertices.data(), layer.cloud.points_local.size(), { 3, 3, 1 });
            clearGpu(layer);
            layer.gpu = std::move(parts);
            layer.dirty = false;
        }
    };

    OverlayLayers::OverlayLayers()
        : impl_(std::make_unique<Impl>())
    {
    }
    OverlayLayers::~OverlayLayers()
    {
        shutdown();
    }

    void OverlayLayers::open()
    {
        impl_->visible = true;
        impl_->expandRequested = true;
    }

    void OverlayLayers::queue(const std::vector<std::string>& files)
    {
        auto& state = *impl_;
        open();
        for (const auto& file : files)
        {
            const fs::path path(file);
            std::error_code error;
            if (!supported(path) || !fs::is_regular_file(path, error))
                state.message = "Unsupported or unavailable overlay: " + file;
            else
                state.pending.push_back(path);
        }
        if (state.phase == Impl::Phase::Idle && !state.pending.empty())
        {
            state.phase = Impl::Phase::Settings;
            state.openPopup = true;
        }
    }

    void OverlayLayers::gui()
    {
        auto& state = *impl_;
        state.poll();
        auto& io = ImGui::GetIO();
        if (io.KeyCtrl && !io.KeyAlt && !io.WantTextInput && ImGui::IsKeyPressed(ImGuiKey_L, false))
        {
            if (state.visible && !state.collapsed)
                state.visible = false;
            else
                open();
        }
        if (state.visible)
        {
            if (state.initialPresentation || state.expandRequested)
            {
                ImGui::SetNextWindowCollapsed(!state.expandRequested, ImGuiCond_Always);
                state.initialPresentation = state.expandRequested = false;
            }
            ImGui::SetNextWindowSize(
                ImVec2(std::min(720.f, io.DisplaySize.x * 0.85f), std::min(760.f, io.DisplaySize.y * 0.85f)), ImGuiCond_FirstUseEver);
            const bool expandedWindow = ImGui::Begin("Layers (Ctrl+L)", &state.visible);
            state.collapsed = ImGui::IsWindowCollapsed();
            if (expandedWindow)
            {
                if (ImGui::Button("Add files..."))
                    queue(
                        mandeye::fd::OpenFileDialog(
                            "Add display-only layers",
                            { "Clouds / trajectories", "*.las *.laz *.e57 *.xyz *.txt *.tum *.csv", "All files", "*" },
                            true));
                ImGui::TextWrapped("Drop cloud/trajectory files here or onto the scene. Drop a raw-data folder to start SLAM.");
                ImGui::TextWrapped("XYZ text; CSV with x/y/z columns; TUM in .tum files.");
                if (!state.message.empty())
                {
                    ImGui::TextWrapped("%s", state.message.c_str());
                    if (ImGui::SmallButton("Dismiss message"))
                        state.message.clear();
                }
                for (size_t i = 0; i < state.layers.size();)
                {
                    auto& layer = *state.layers[i];
                    ImGui::PushID(&layer);
                    ImGui::Checkbox("##visible", &layer.visible);
                    ImGui::SameLine();
                    const bool expanded = ImGui::TreeNodeEx("Layer", ImGuiTreeNodeFlags_DefaultOpen, "%s", layer.name);
                    bool remove = false;
                    if (expanded)
                    {
                        controlLabel("Name");
                        ImGui::InputText("##name", layer.name, sizeof(layer.name));
                        ImGui::TextWrapped("%s", layer.path.string().c_str());
                        const size_t count = layer.cloud.points_local.size();
                        ImGui::TextWrapped(
                            "Source: %llu | Retained: %zu | Drawn: %zu",
                            static_cast<unsigned long long>(layer.sourceCount),
                            count,
                            layer.visible ? (count + layer.stride - 1) / layer.stride : 0);
                        ImGui::Text("Load spacing: %.3f m", layer.spacing);
                        controlLabel("Render stride");
                        ImGui::SliderInt(
                            "##render-stride", &layer.stride, 1, 1000, "%d", ImGuiSliderFlags_Logarithmic | ImGuiSliderFlags_AlwaysClamp);
                        layer.stride = std::clamp(layer.stride, 1, 1000);
                        controlLabel("Point size");
                        ImGui::SliderFloat("##point-size", &layer.size, 1, 12, "%.1f", ImGuiSliderFlags_AlwaysClamp);
                        layer.size = std::isfinite(layer.size) ? std::clamp(layer.size, 1.f, 12.f) : 3.f;
                        if (layer.trajectory)
                        {
                            ImGui::Checkbox("Points", &layer.points);
                            ImGui::SameLine();
                            ImGui::Checkbox("Connected line", &layer.lines);
                        }
                        controlLabel("Color");
                        ImGui::Combo("##color-mode", &layer.mode, "Uniform\0Native RGB\0Intensity\0Elevation\0");
                        if (layer.mode == 1 && !layer.rgb)
                            ImGui::TextWrapped("Native RGB unavailable: using uniform color.");
                        if (layer.mode == 2 && !layer.intensity)
                            ImGui::TextWrapped("Intensity unavailable: using uniform color.");
                        controlLabel("Uniform color");
                        ImGui::ColorEdit3("##uniform-color", layer.color);
                        if (layer.mode >= 2)
                        {
                            controlLabel("Palette");
                            ImGui::Combo(
                                "##palette", &layer.palette, raylib_widgets::kColormapNames, IM_ARRAYSIZE(raylib_widgets::kColormapNames));
                        }
                        ImGui::BeginDisabled(!layer.intensity);
                        ImGui::Checkbox("Intensity shading", &layer.shade);
                        ImGui::EndDisabled();
                        offsetInput(layer.editedOffset);
                        ImGui::BeginDisabled(!layer.editedOffset.allFinite());
                        if (ImGui::Button("Apply offset"))
                        {
                            layer.offset = layer.editedOffset;
                            layer.dirty = true;
                        }
                        ImGui::EndDisabled();
                        ImGui::SameLine();
                        remove = ImGui::Button("Remove layer");
                        ImGui::TreePop();
                    }
                    ImGui::PopID();
                    if (remove)
                    {
                        clearGpu(layer);
                        state.layers.erase(state.layers.begin() + i);
                    }
                    else
                        ++i;
                }
            }
            ImGui::End();
        }
        state.importPopup();
    }

    void OverlayLayers::draw()
    {
        auto& state = *impl_;
        if (state.layers.empty())
            return;
        if (!state.shaderAttempted)
        {
            state.shaderAttempted = true;
            state.shader = LoadShaderFromMemory(vertexShader, fragmentShader.c_str());
            const char* names[] = { "mvp", "layerColor", "colorMode", "palette", "shadeIntensity", "elevationRange", "pointSize" };
            for (size_t i = 0; i < state.uniforms.size(); ++i)
                state.uniforms[i] = GetShaderLocation(state.shader, names[i]);
        }
        if (!state.shader.id || state.shader.id == rlGetShaderIdDefault())
        {
            state.message = "Overlay shader could not be loaded.";
            return;
        }
        rlDrawRenderBatchActive();
        glEnable(GL_PROGRAM_POINT_SIZE);
        rlEnableShader(state.shader.id);
        const Matrix mvp = MatrixMultiply(rlGetMatrixModelview(), rlGetMatrixProjection());
        rlSetUniformMatrix(state.uniforms[0], mvp);
        for (auto& item : state.layers)
        {
            auto& layer = *item;
            if (!layer.visible)
                continue;
            try
            {
                if (layer.dirty)
                    state.upload(layer);
            } catch (const std::exception& error)
            {
                state.message = error.what();
                layer.visible = false;
                continue;
            }
            const int mode = (layer.mode == 1 && !layer.rgb) || (layer.mode == 2 && !layer.intensity) ? 0 : layer.mode;
            const int shade = layer.shade && layer.intensity;
            const float elevation[2] = { static_cast<float>(layer.minimum.z() - layer.offset.z()),
                                         static_cast<float>(layer.maximum.z() - layer.offset.z()) };
            rlSetUniform(state.uniforms[1], layer.color, RL_SHADER_UNIFORM_VEC3, 1);
            rlSetUniform(state.uniforms[2], &mode, RL_SHADER_UNIFORM_INT, 1);
            rlSetUniform(state.uniforms[3], &layer.palette, RL_SHADER_UNIFORM_INT, 1);
            rlSetUniform(state.uniforms[4], &shade, RL_SHADER_UNIFORM_INT, 1);
            rlSetUniform(state.uniforms[5], elevation, RL_SHADER_UNIFORM_VEC2, 1);
            rlSetUniform(state.uniforms[6], &layer.size, RL_SHADER_UNIFORM_FLOAT, 1);
            if (layer.stride > 1 && layer.indexedStride != layer.stride)
            {
                clearIndices(layer);
                // Large attribute strides exceed driver limits. Keep geometry fixed and select indices instead.
                for (const auto& part : layer.gpu)
                {
                    std::vector<unsigned int> indices;
                    indices.reserve((part.count + layer.stride - 1) / layer.stride);
                    for (int i = 0; i < part.count; i += layer.stride)
                        indices.push_back(static_cast<unsigned int>(i));
                    rlEnableVertexArray(part.vao);
                    layer.indexBuffers.push_back(
                        rlLoadVertexBufferElement(indices.data(), static_cast<int>(indices.size() * sizeof(unsigned int)), false));
                }
                layer.indexedStride = layer.stride;
            }
            for (size_t i = 0; i < layer.gpu.size(); ++i)
            {
                const auto& part = layer.gpu[i];
                rlEnableVertexArray(part.vao);
                const int count = (part.count - 1) / layer.stride + 1;
                if (layer.stride > 1)
                {
                    rlEnableVertexBufferElement(layer.indexBuffers[i]);
                    if (layer.points)
                        glDrawElements(GL_POINTS, count, GL_UNSIGNED_INT, nullptr);
                    if (layer.lines && count > 1)
                        glDrawElements(GL_LINE_STRIP, count, GL_UNSIGNED_INT, nullptr);
                }
                else
                {
                    if (layer.points)
                        glDrawArrays(GL_POINTS, 0, count);
                    if (layer.lines && count > 1)
                        glDrawArrays(GL_LINE_STRIP, 0, count);
                }
            }
        }
        rlDisableVertexArray();
        rlDisableVertexBuffer();
        rlDisableShader();
    }

    void OverlayLayers::shutdown()
    {
        auto& state = *impl_;
        if (state.progress)
            state.progress->cancel = true;
        if (state.loading.valid())
            state.loading.wait();
        for (auto& layer : state.layers)
            clearGpu(*layer);
        state.layers.clear();
        if (state.shader.id && state.shader.id != rlGetShaderIdDefault())
            UnloadShader(state.shader);
        state.shader = {};
    }
} // namespace step1_render

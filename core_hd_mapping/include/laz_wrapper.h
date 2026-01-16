#pragma once

#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <imgui_internal.h>

#include <ImGuizmo.h>

#include <project_settings.h>
#include <structures.h>

class LazWrapper
{
public:
    LazWrapper() = default;
    ~LazWrapper() = default;

    void imgui(const CommonData& common_data, const ProjectSettings& project_setings);
    void render(const CommonData& common_data);

    LAZSector load_sector(const std::string& filename, const double shift_x, const double shift_y);

    void reload_all_LAZ_files(const double shift_x, const double shift_y);

    std::vector<LAZSector> sectors = {};
    int32_t decimation = 100;
    int32_t num_threads = 16;
};

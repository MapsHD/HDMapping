#ifndef _LAZ_WRAPPER_H_
#define _LAZ_WRAPPER_H_

#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <ImGuizmo.h>
#include <imgui_internal.h>

#include <structures.h>
#include <project_settings.h>

class LazWrapper {
public:
    LazWrapper() { ; };
	~LazWrapper() { ; };

	void imgui(CommonData& common_data, const ProjectSettings& project_setings);
	void render(const CommonData& common_data);
    LAZSector load_sector(const std::string& filename, double shift_x, double shift_y);
   // void reload_all_LAZ_files_job(int i, Job* job, /*LAZSector& s,*/ double shift_x, double shift_y);
    void reload_all_LAZ_files(double shift_x, double shift_y);

    std::vector<LAZSector> sectors;
    int decimation = 100;
    int num_threads = 16;
};

#endif
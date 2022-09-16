#include <laz_wrapper.h>

#include <iostream>
#include <fstream>
#include <thread>

#include <portable-file-dialogs.h>

#include <laszip/laszip_api.h>

#include <GL/freeglut.h>

void LazWrapper::imgui(CommonData& common_data, const ProjectSettings& project_setings) {
	ImGui::Begin("LazWrapper");
	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

	if (ImGui::Button("Add point cloud from LAZ file")) {
		static std::shared_ptr<pfd::open_file> open_file;
		std::vector<std::string> input_file_names;
		ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
		const auto t = [&]() {
            auto sel = pfd::open_file("Choose folder", "C:\\", {"LAZ files, *.laz"}, true).result();
			for (int i = 0; i < sel.size(); i++)
			{
				std::string input_file_name = sel[i];
				std::cout << "LAZ file: '" << input_file_name << "'" << std::endl;

                if (input_file_name.size() > 0) {
                    input_file_names.push_back(input_file_name);
                }
			}
		};
		std::thread t1(t);
		t1.join();

        for(int i = 0 ; i < input_file_names.size(); i++){
            std::cout << "loading progers: " << i + 1 << " of " << input_file_names.size() << std::endl;
            const auto& fn = input_file_names[i];
            LAZSector sector = load_sector(fn, project_setings.shift_x, project_setings.shift_y);
            if (sector.point_cloud.size() > 0) {
                sectors.push_back(sector);
                std::cout << "sector loaded from file '" << fn << "'" << std::endl;
            }
        }
	}

    if (ImGui::Button("LAZ hide all files")) {
        for (size_t i = 0; i < sectors.size(); i++) {
            sectors[i].visible = false;
        }
    }

    bool can_show = false;
    for (size_t i = 0; i < sectors.size(); i++) {
        if (sectors[i].point_cloud.size() > 0) {
            can_show = true;
        }
    }

    if (can_show) {
        ImGui::SameLine();
        if (ImGui::Button("LAZ show all files")) {
            decimation = 100;
            for (size_t i = 0; i < sectors.size(); i++) {
                if (sectors[i].point_cloud.size() == 0) {
                    LazWrapper lw;
                    LAZSector sector = lw.load_sector(sectors[i].file_name, project_setings.shift_x, project_setings.shift_y);
                    sectors[i].point_cloud = sector.point_cloud;

                }
                sectors[i].visible = true;
            }
        }
    }

    if (ImGui::Button("LAZ reload all files")) {
        decimation = 100;
        reload_all_LAZ_files(project_setings.shift_x, project_setings.shift_y);

        for (size_t i = 0; i < sectors.size(); i++) {
            sectors[i].visible = true;
        }
    }
    ImGui::SliderInt("LAZ loader threads", &num_threads, 1, 128);

    ImGui::SliderInt("LAZ decimation", &decimation, 1, 1000);
        
    for (size_t i = 0; i < sectors.size(); i++) {
        bool previous_visibility = sectors[i].visible;
        ImGui::Checkbox(("LAZ[" + std::to_string(i) +  "]: " + sectors[i].file_name).c_str(), &sectors[i].visible);
        ImGui::SameLine();
        ImGui::PushButtonRepeat(true);
        float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
        if (ImGui::ArrowButton(("[" + std::to_string(i) + "] ##sector_left").c_str(), ImGuiDir_Left)) { (sectors[i].point_size)--; }
        ImGui::SameLine(0.0f, spacing);
        if (ImGui::ArrowButton(("[" + std::to_string(i) + "] ##sector_right").c_str(), ImGuiDir_Right)) { (sectors[i].point_size)++; }
        ImGui::PopButtonRepeat();
        ImGui::SameLine();
        ImGui::Text("point size %d", sectors[i].point_size);
        if (sectors[i].point_size < 1) sectors[i].point_size = 1;

        if (sectors[i].visible != previous_visibility) {
            if (sectors[i].visible) {
                if (sectors[i].point_cloud.size() == 0) {
                    LAZSector sector = load_sector(sectors[i].file_name, project_setings.shift_x, project_setings.shift_y);
                    sectors[i].point_cloud = sector.point_cloud;
                }
            }
        }
    }

	ImGui::End();
}

void LazWrapper::render(const CommonData& common_data) {

    glColor3f(0.5, 0.5, 0.5);
    for (const auto& s : sectors) {
        if (s.visible) {
            glPointSize(s.point_size);
            glBegin(GL_POINTS);
            //for (const auto& p : s.point_cloud) {
            for(int i = 0 ; i < s.point_cloud.size(); i+= decimation){
                const auto &p = s.point_cloud[i];
                glColor3f(p.r, p.g, p.b);
                glVertex3f(p.x, p.y, p.z);
            }
            glEnd();
            glPointSize(1);
        }
    }
}

LAZSector LazWrapper::load_sector(const std::string& filename, double shift_x, double shift_y)
{
	LAZSector sector;
	sector.file_name = filename;

    laszip_POINTER laszip_reader;
    if (laszip_create(&laszip_reader))
    {
        fprintf(stderr, "DLL ERROR: creating laszip reader\n");
        std::abort();
    }

    laszip_BOOL is_compressed = 0;
    if (laszip_open_reader(laszip_reader, filename.c_str(), &is_compressed))
    {
        fprintf(stderr, "DLL ERROR: opening laszip reader for '%s'\n", filename.c_str());
        std::abort();
    }
    std::cout << "compressed : " << is_compressed << std::endl;
    laszip_header* header;

    if (laszip_get_header_pointer(laszip_reader, &header))
    {
        fprintf(stderr, "DLL ERROR: getting header pointer from laszip reader\n");
        std::abort();
    }
    fprintf(stderr, "file '%s' contains %u points\n", filename.c_str(), header->number_of_point_records);
    laszip_point* point;
    if (laszip_get_point_pointer(laszip_reader, &point))
    {
        fprintf(stderr, "DLL ERROR: getting point pointer from laszip reader\n");
        std::abort();
    }
    //    int64_t point_count;
   
    sector.point_cloud.reserve(header->number_of_point_records);

    sector.min_x = 1000000000000;
    sector.max_x = -1000000000000;
    sector.min_y = 1000000000000;
    sector.max_y = -1000000000000;

    for (int i = 0; i < header->number_of_point_records; i++)
    {
        if (laszip_read_point(laszip_reader))
        {
            fprintf(stderr, "DLL ERROR: reading point %u\n", i);
            std::abort();
        }
        LAZPoint p;
        p.x = header->x_offset + header->x_scale_factor * static_cast<double>(point->X) + shift_x;
        p.y = header->y_offset + header->y_scale_factor * static_cast<double>(point->Y) + shift_y;
        p.z = header->z_offset + header->z_scale_factor * static_cast<double>(point->Z);

        p.r = float(point->rgb[0]) / 65536.0f;
        p.g = float(point->rgb[1]) / 65536.0f;
        p.b = float(point->rgb[2]) / 65536.0f;
        p.classsification = point->classification;

        sector.point_cloud.push_back(p);

        if (p.x < sector.min_x) {
            sector.min_x = p.x;
        }
        if (p.x > sector.max_x) {
            sector.max_x = p.x;
        }
        if (p.y < sector.min_y) {
            sector.min_y = p.y;
        }
        if (p.y > sector.max_y) {
            sector.max_y = p.y;
        }
    }
   
	return sector;
}

void reload_all_LAZ_files_job(int i, Job* job, LAZSector* s, double shift_x, double shift_y) {
    LazWrapper lw;
    LAZSector sector = lw.load_sector((*s).file_name, shift_x, shift_y);
    (*s).point_cloud = sector.point_cloud;
}


void LazWrapper::reload_all_LAZ_files(double shift_x, double shift_y)
{
    std::vector<Job> jobs = get_jobs(sectors.size(), num_threads);
    std::vector<std::thread> threads;

    for (size_t i = 0; i < jobs.size(); i++) {
        threads.push_back(std::thread(reload_all_LAZ_files_job, i, &jobs[i], &(sectors[i]), shift_x, shift_y));
    }

    for (size_t j = 0; j < threads.size(); j++) {
        threads[j].join();
    }
    threads.clear();
}
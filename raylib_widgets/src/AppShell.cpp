#include "RaylibWidgets/AppShell.h"

#include <imgui_internal.h> // ImGui::DockBuilder*

#include <algorithm>

#ifdef _WIN32
// NOGDI/NOUSER: windows.h's wingdi.h/winuser.h #define (or, for CloseWindow/
// ShowCursor, directly declare) identifiers that collide with raylib.h's own
// DrawText/CloseWindow/ShowCursor in any translation unit that also includes
// raylib.h -- harmless here (this file doesn't), but matches the convention
// every other raylib_widgets .cpp that touches windows.h follows. NOUSER
// also strips SW_SHOWNORMAL, so ImGuiHyperlink's ShellExecuteA call below
// uses its literal value (1, a stable, decades-unchanged Win32 constant)
// instead.
#define NOGDI
#define NOUSER
// clang-format off
#include <windows.h>
#include <shellapi.h>
// clang-format on
#endif

namespace raylib_widgets
{

    void ShowMainDockSpace()
    {
        static bool first_time = true;

        ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
            ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoInputs;

        ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        ImGui::SetNextWindowViewport(viewport->ID);

        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);

        ImGui::Begin("MainDockSpace", nullptr, window_flags);

        ImGui::PopStyleVar(2);

        ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
        ImGui::DockSpace(dockspace_id, ImVec2(0, 0), ImGuiDockNodeFlags_PassthruCentralNode | ImGuiDockNodeFlags_NoDockingInCentralNode);

        if (first_time)
        {
            first_time = false;

            auto dock_id_left = ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Left, 0.2f, nullptr, &dockspace_id);
            auto dock_id_bottom = ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Down, 0.2f, nullptr, &dockspace_id);
            (void)dock_id_left;

            ImGui::DockBuilderDockWindow("Console", dock_id_bottom);
            ImGui::DockBuilderFinish(dockspace_id);
        }

        ImGui::End();
    }

    void ImGuiHyperlink(const char* url, ImVec4 color)
    {
        ImGui::PushStyleColor(ImGuiCol_Text, color);
        ImGui::TextUnformatted(url);
        ImGui::PopStyleColor();

        ImVec2 pos = ImGui::GetItemRectMin();
        ImVec2 size = ImGui::GetItemRectSize();

        if (ImGui::IsItemHovered())
            ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);

        if (ImGui::IsItemHovered())
        {
            ImDrawList* draw_list = ImGui::GetWindowDrawList();
            draw_list->AddLine(ImVec2(pos.x, pos.y + size.y), ImVec2(pos.x + size.x, pos.y + size.y), ImColor(color));
        }

        if (ImGui::IsItemClicked())
        {
#ifdef _WIN32
            ShellExecuteA(0, "open", url, 0, 0, 1 /* SW_SHOWNORMAL, unavailable under NOUSER -- see this file's top comment */);
#elif __APPLE__
            std::string cmd = std::string("open ") + url;
            system(cmd.c_str());
#else
            std::string cmd = std::string("xdg-open ") + url;
            system(cmd.c_str());
#endif
        }
    }

    void ShowInfoWindow(
        bool& open,
        const std::vector<std::string>& infoLines,
        const std::vector<ShortcutEntry>& appShortcuts,
        const char* versionString,
        const char* buildDate)
    {
        if (!open)
            return;

        static bool show_about = false;
        const ImVec2 workSize = ImGui::GetMainViewport()->WorkSize;
        const ImVec2 maximum(workSize.x * 0.95f, workSize.y * 0.95f);
        ImGui::SetNextWindowSize(ImVec2(std::min(1000.f, maximum.x), std::min(760.f, maximum.y)), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSizeConstraints(ImVec2(std::min(540.f, maximum.x), std::min(480.f, maximum.y)), maximum);

        if (ImGui::Begin("Info", &open, ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoCollapse))
        {
            if (ImGui::BeginTabBar("InfoTabs"))
            {
                if (ImGui::BeginTabItem("Controls", nullptr, ImGuiTabItemFlags_NoPushId))
                {
                    ShowShortcutsTable(appShortcuts, std::max(1.f, ImGui::GetContentRegionAvail().y));
                    ImGui::EndTabItem();
                }
                if (ImGui::BeginTabItem("About / camera guide", nullptr, ImGuiTabItemFlags_NoPushId))
                {
                    if (ImGui::BeginChild("InfoDetails"))
                    {
                        ImGui::PushTextWrapPos(0.f);
                        for (const auto& line : infoLines)
                        {
                            if (line.empty())
                                ImGui::NewLine();
                            else if (line.rfind("https://", 0) == 0)
                                ImGuiHyperlink(line.c_str());
                            else
                                ImGui::TextUnformatted(line.c_str());
                        }

                        ImGui::NewLine();
                        ImGui::TextUnformatted("Author: Janusz Bedkowski & contributors");
                        ImGui::NewLine();
                        ImGui::TextUnformatted("Part of HDMapping software suite");
                        ImGui::Text("Version: %s (%s)", versionString, buildDate);
                        ImGui::TextUnformatted("Project page:");
                        ImGuiHyperlink("https://github.com/MapsHD/HDMapping");
                        ImGui::PopTextWrapPos();
                        if (ImGui::Button("ImGui"))
                            show_about = true;
                    }
                    ImGui::EndChild();
                    ImGui::EndTabItem();
                }
                ImGui::EndTabBar();
            }
            if (show_about)
                ImGui::ShowAboutWindow(&show_about);
        }

        ImGui::End();
    }

} // namespace raylib_widgets

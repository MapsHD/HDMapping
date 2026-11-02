#include <RaylibWidgets/ShortcutsTable.h>
#include <imgui.h>

#include <algorithm>
#include <cfloat>

namespace raylib_widgets
{

    void ShowShortcutsTable(const std::vector<ShortcutEntry>& entries, float height)
    {
        if (!ImGui::BeginTable(
                "ShortcutsTable", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY, ImVec2(-FLT_MIN, height)))
            return;

        float shortcutWidth = 120.f;
        for (const auto& entry : entries)
            if (!entry.description.empty())
                shortcutWidth =
                    std::max(shortcutWidth, ImGui::CalcTextSize(entry.shortcut.c_str()).x + ImGui::GetStyle().CellPadding.x * 2);
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("Shortcut", ImGuiTableColumnFlags_WidthFixed, shortcutWidth);
        ImGui::TableSetupColumn("Description");
        ImGui::TableHeadersRow();

        std::string lastType;
        for (const auto& e : entries)
        {
            if (!e.type.empty() && e.type != lastType)
            {
                lastType = e.type;
                ImGui::TableNextRow();
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, IM_COL32(70, 70, 140, 255));
                ImGui::TableSetColumnIndex(0);
                ImGui::TextColored(ImVec4(0.8f, 0.8f, 1.0f, 1.0f), "%s", lastType.c_str());
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted("");
            }

            if (!e.description.empty())
            {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(e.shortcut.c_str());
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(e.description.c_str());
            }
        }

        ImGui::EndTable();
    }

} // namespace raylib_widgets

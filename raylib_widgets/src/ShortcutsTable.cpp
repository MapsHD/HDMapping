#include <RaylibWidgets/ShortcutsTable.h>
#include <imgui.h>

#include <cfloat>

namespace raylib_widgets {

void ShowShortcutsTable(const std::vector<ShortcutEntry>& entries)
{
    if (!ImGui::BeginTable(
            "ShortcutsTable", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY, ImVec2(-FLT_MIN, 200)))
        return;

    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("Shortcut", ImGuiTableColumnFlags_WidthFixed, 120);
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

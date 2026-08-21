#include <RaylibWidgets/CenterOfRotationWindow.h>

#include <imgui.h>

namespace raylib_widgets {

void showCenterOfRotationWindow(bool& open, OrbitCamera& camera)
{
    static Vector3 pending = { 0.f, 0.f, 0.f };

    if (open)
    {
        pending = camera.target;
        ImGui::OpenPopup("Center of rotation");
        open = false;
    }

    if (ImGui::BeginPopupModal("Center of rotation", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::Text("Select new center of rotation [m]:");
        ImGui::PushItemWidth(120.f);
        ImGui::InputFloat("X", &pending.x, 0.0f, 0.0f, "%.3f");
        ImGui::SameLine();
        ImGui::InputFloat("Y", &pending.y, 0.0f, 0.0f, "%.3f");
        ImGui::SameLine();
        ImGui::InputFloat("Z", &pending.z, 0.0f, 0.0f, "%.3f");
        ImGui::PopItemWidth();

        ImGui::Separator();

        if (ImGui::Button("Set"))
        {
            camera.moveTargetTo(pending);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel"))
        {
            ImGui::CloseCurrentPopup();
        }

        ImGui::EndPopup();
    }
}

void showEulerCenterOfRotationWindow(
    bool& open, OrbitCamera& camera, const char* xTooltip, const char* yTooltip, const char* zTooltip)
{
    if (open)
    {
        ImGui::OpenPopup("Center of rotation");
        open = false;
    }

    if (ImGui::BeginPopupModal("Center of rotation", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    {
        // Bound directly to camera.eulerGoal.rotationCenter (not a
        // separately-initialized local) -- matches the original
        // cor_window(), which edited its own persistent new_rotation_center
        // field in place rather than seeding a fresh copy each time the
        // popup opens.
        Vector3& pending = camera.eulerGoal.rotationCenter;

        ImGui::Text("Select new center of rotation [m]:");
        ImGui::PushItemWidth(120.f);
        ImGui::InputFloat("X", &pending.x, 0.0f, 0.0f, "%.3f");
        if (xTooltip && ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", xTooltip);
        ImGui::SameLine();
        ImGui::InputFloat("Y", &pending.y, 0.0f, 0.0f, "%.3f");
        if (yTooltip && ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", yTooltip);
        ImGui::SameLine();
        ImGui::InputFloat("Z", &pending.z, 0.0f, 0.0f, "%.3f");
        if (zTooltip && ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", zTooltip);
        ImGui::PopItemWidth();

        ImGui::Separator();

        if (ImGui::Button("Set"))
        {
            camera.moveEulerRotationCenterTo(pending);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel"))
        {
            ImGui::CloseCurrentPopup();
        }

        ImGui::EndPopup();
    }
}

} // namespace raylib_widgets

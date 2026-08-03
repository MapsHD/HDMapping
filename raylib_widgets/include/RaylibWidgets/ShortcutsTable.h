#pragma once
#include <string>
#include <vector>

// A generic ImGui table for documenting an app's keyboard/mouse shortcuts,
// shared between apps/multi_view_tls_registration (rl_utils.cpp's
// ShowShortcutsTable/ShortcutEntry) and the camera_lidar_* apps. Depends on
// raylib_widgets' existing imgui_raylib link, no Eigen/core coupling.
namespace raylib_widgets {

struct ShortcutEntry {
    std::string type; // group header (e.g. "Normal keys"), shown once when it changes from the previous entry ("" = no header)
    std::string shortcut; // key combo label, e.g. "Ctrl+O"
    std::string description; // shown next to shortcut; entries with an empty description are skipped (but still contribute a header if `type` is set)
};

// Renders `entries` as a bordered, scrollable two-column table (Shortcut |
// Description) inside the current ImGui window.
void ShowShortcutsTable(const std::vector<ShortcutEntry>& entries);

} // namespace raylib_widgets

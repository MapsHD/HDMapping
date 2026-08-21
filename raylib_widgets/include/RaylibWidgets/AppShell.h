#pragma once
#include <imgui.h>

#include <RaylibWidgets/ShortcutsTable.h>

#include <string>
#include <vector>

// Generic ImGui app-shell helpers -- a full-window docking host, a
// clickable hyperlink, and an "about/shortcuts" info window -- shared
// between apps/multi_view_tls_registration (rl_utils.cpp's
// ShowMainDockSpace/ImGuiHyperlink/info_window) and any other raylib_widgets
// consumer that wants the same scaffolding. Depends on raylib_widgets'
// existing imgui_raylib link only -- no Eigen/core coupling.
namespace raylib_widgets {

// Full-window transparent host for ImGui docking. Call once per frame,
// before building any dockable panel windows. On its first call ever, splits
// off a left column and a bottom "Console" dock -- later calls just
// re-assert the dockspace.
void ShowMainDockSpace();

// Clickable-looking text that opens `url` in the system browser when
// clicked (Windows: ShellExecuteA; macOS/Linux: `open`/`xdg-open`).
void ImGuiHyperlink(const char* url, ImVec4 color = ImVec4(0.2f, 0.4f, 0.8f, 1.0f));

// "About/shortcuts" window. `open` is both the visibility toggle (the call
// is a no-op while false) and gets cleared by the window's own close
// button/X. infoLines is rendered as plain text, except a blank line (blank
// line in the window) or one starting with "https://" (rendered as a
// clickable ImGuiHyperlink()); appShortcuts is rendered via
// ShowShortcutsTable(). versionString/buildDate go into a "Version: %s (%s)"
// line (pass e.g. HDMAPPING_VERSION_STRING and __DATE__).
void ShowInfoWindow(
    bool& open, const std::vector<std::string>& infoLines, const std::vector<ShortcutEntry>& appShortcuts,
    const char* versionString, const char* buildDate);

} // namespace raylib_widgets

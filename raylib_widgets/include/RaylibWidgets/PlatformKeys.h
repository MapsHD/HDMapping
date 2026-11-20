#pragma once

// Platform names of the shortcut modifier keys, as string literals so they join adjacent literals at compile
// time: SHORTCUT_CTRL "+O" is "Ctrl+O" on Windows and Linux, "Cmd+O" on macOS. Dear ImGui swaps Cmd and
// Ctrl on macOS (io.ConfigMacOSXBehaviors), so io.KeyCtrl / ImGuiMod_Ctrl already mean Cmd there.
#ifdef __APPLE__
#define SHORTCUT_CTRL "Cmd"
#define SHORTCUT_ALT "Option"
#else
#define SHORTCUT_CTRL "Ctrl"
#define SHORTCUT_ALT "Alt"
#endif

namespace raylib_widgets
{

    //! True while the platform's shortcut modifier is held, read from raylib: Cmd on macOS, Ctrl elsewhere.
    bool isShortcutModifierDown();

    //! True on the frame the platform's shortcut modifier is pressed, read from raylib: Cmd on macOS, Ctrl elsewhere.
    bool isShortcutModifierPressed();

} // namespace raylib_widgets

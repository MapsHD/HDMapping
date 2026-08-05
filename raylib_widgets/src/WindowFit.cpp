#include <RaylibWidgets/WindowFit.h>
#include "raylib.h"

#include <algorithm>

namespace raylib_widgets {

void fitWindowToScreen(int marginW, int marginH, bool centerVertically)
{
    int monitor = GetCurrentMonitor();
    // GetMonitorWidth/Height return the monitor's native PIXEL resolution
    // (GLFW's glfwGetVideoMode), while GetScreenWidth/Height, SetWindowSize
    // and SetWindowPosition all operate in logical points -- on a 2x Retina
    // display that's a 2x unit mismatch. Divide by the DPI scale to bring
    // the monitor size into the same points space everything else uses;
    // without this, SetWindowPosition computes an X centered on a monitor
    // twice too wide, pushing most of the window off the right edge of the
    // actual (points-sized) screen.
    Vector2 dpi = GetWindowScaleDPI();
    if (dpi.x <= 0.f)
        dpi.x = 1.f;
    if (dpi.y <= 0.f)
        dpi.y = 1.f;
    int monW = (int)(GetMonitorWidth(monitor) / dpi.x);
    int monH = (int)(GetMonitorHeight(monitor) / dpi.y);
    if (monW <= 0 || monH <= 0)
        return; // monitor info unavailable, leave as-is

    int w = std::min(GetScreenWidth(), monW - marginW);
    int h = std::min(GetScreenHeight(), monH - marginH);
    if (w != GetScreenWidth() || h != GetScreenHeight())
        SetWindowSize(w, h);

    int posY = centerVertically ? std::max(0, (monH - h) / 2) : 30;
    SetWindowPosition(std::max(0, (monW - w) / 2), posY);

    // SetWindowSize/SetWindowPosition only update GLFW's window state; raylib's
    // cached mouse/window geometry (what rlImGui reads into io.MousePos every
    // frame) isn't refreshed until the next PollInputEvents(), which otherwise
    // wouldn't happen until the first EndDrawing() -- after rlImGuiSetup() has
    // already run. Without this, every click lands offset from the cursor by
    // however far this function just moved/resized the window.
    PollInputEvents();
}

} // namespace raylib_widgets

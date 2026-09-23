#pragma once

// Windows taskbar progress indicator (ITaskbarList3). Kept in its own translation unit so that <windows.h> never meets
// raylib.h, whose CloseWindow/ShowCursor/DrawText names it collides with.
#ifdef _WIN32
//! Initializes the taskbar interface for the given native window.
//! @param hwnd native window handle (raylib's GetWindowHandle()).
void InitTaskbarProgress(void* hwnd);
//! @param progress01 progress in [0, 1].
void SetTaskbarProgress(double progress01);
void ClearTaskbarProgress();
#endif

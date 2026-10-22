// Windows taskbar progress for the raylib apps -- same as core/src/utils.cpp's GLUT version, but the
// window handle comes from raylib (GetWindowHandle()) instead of FindWindow() on the title. Its own
// translation unit because windows.h and raylib.h declare conflicting CloseWindow/ShowCursor/DrawText.
#ifdef _WIN32

// clang-format off
#include <windows.h>
#include <shobjidl.h>
// clang-format on

//! Defined in raylib_utils.cpp (GetWindowHandle()).
void* raylibWindowHandle();

namespace
{
    ITaskbarList3* g_taskbar = nullptr;
}

void InitTaskbarProgress()
{
    CoInitialize(nullptr);

    HRESULT hr = CoCreateInstance(CLSID_TaskbarList, nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&g_taskbar));

    if (SUCCEEDED(hr))
        g_taskbar->HrInit();
}

void SetTaskbarProgress(double progress01)
{
    if (!g_taskbar)
        return;

    const ULONGLONG total = 1000;
    ULONGLONG value = (ULONGLONG)(progress01 * total);

    HWND hwnd = static_cast<HWND>(raylibWindowHandle());
    g_taskbar->SetProgressState(hwnd, TBPF_NORMAL);
    g_taskbar->SetProgressValue(hwnd, value, total);
}

void ClearTaskbarProgress()
{
    if (!g_taskbar)
        return;
    g_taskbar->SetProgressState(static_cast<HWND>(raylibWindowHandle()), TBPF_NOPROGRESS);
}

#endif

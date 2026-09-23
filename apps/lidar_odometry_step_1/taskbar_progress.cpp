#include "taskbar_progress.h"

#ifdef _WIN32
#include <shobjidl.h> // ITaskbarList3
#include <windows.h>

namespace
{
    HWND g_hwnd = nullptr;
    ITaskbarList3* g_taskbar = nullptr;
} // namespace

void InitTaskbarProgress(void* hwnd)
{
    g_hwnd = static_cast<HWND>(hwnd);

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
    const ULONGLONG value = static_cast<ULONGLONG>(progress01 * total);

    g_taskbar->SetProgressState(g_hwnd, TBPF_NORMAL);
    g_taskbar->SetProgressValue(g_hwnd, value, total);
}

void ClearTaskbarProgress()
{
    if (!g_taskbar)
        return;
    g_taskbar->SetProgressState(g_hwnd, TBPF_NOPROGRESS);
}
#endif

#include "sleep_blocker.h"

#if defined(__APPLE__)
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/pwr_mgt/IOPMLib.h>
#elif defined(_WIN32)
#include <windows.h>
#endif

namespace easy_gui
{
#if defined(__APPLE__)
    SleepBlocker::SleepBlocker(const std::string& reason)
    {
        CFStringRef name = CFStringCreateWithCString(kCFAllocatorDefault, reason.c_str(), kCFStringEncodingUTF8);
        IOPMAssertionID id = 0;
        active_ = name &&
            IOPMAssertionCreateWithName(kIOPMAssertionTypePreventUserIdleSystemSleep, kIOPMAssertionLevelOn, name, &id) == kIOReturnSuccess;
        if (name)
            CFRelease(name);
        assertion_id_ = id;
    }

    SleepBlocker::~SleepBlocker()
    {
        if (active_)
            IOPMAssertionRelease(assertion_id_);
    }
#elif defined(_WIN32)
    SleepBlocker::SleepBlocker(const std::string&)
    {
        active_ = SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED) != 0;
    }

    SleepBlocker::~SleepBlocker()
    {
        if (active_)
            SetThreadExecutionState(ES_CONTINUOUS);
    }
#else
    SleepBlocker::SleepBlocker(const std::string&)
    {
    }

    SleepBlocker::~SleepBlocker()
    {
    }
#endif
} // namespace easy_gui

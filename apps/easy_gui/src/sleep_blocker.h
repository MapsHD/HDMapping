#pragma once

#include <string>

namespace easy_gui
{
    //! Keeps the computer from going to sleep on its own while it exists (the display may still turn off). A batch can
    //! run for hours unattended, and idle sleep would suspend lidar odometry halfway.
    //!  - macOS: an IOKit power assertion (kIOPMAssertionTypePreventUserIdleSystemSleep), visible in `pmset -g assertions`.
    //!  - Windows: SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED). It applies to the calling thread, so a
    //!    SleepBlocker must be destroyed on the thread that created it.
    //!  - elsewhere: does nothing.
    class SleepBlocker
    {
    public:
        explicit SleepBlocker(const std::string& reason);
        ~SleepBlocker();
        SleepBlocker(const SleepBlocker&) = delete;
        SleepBlocker& operator=(const SleepBlocker&) = delete;

        //! Whether the system accepted the request.
        bool active() const
        {
            return active_;
        }

    private:
        bool active_ = false;
        unsigned int assertion_id_ = 0; // macOS IOPMAssertionID
    };
} // namespace easy_gui

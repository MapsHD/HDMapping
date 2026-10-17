#pragma once

#include <algorithm>

namespace easy_gui
{
    //! User-adjustable settings. Everything else comes from the forestry preset.
    struct Settings
    {
        //! Near points (horizontal range below the 1.0 m load filter) are exported only from this 3D range outwards.
        //! 0.4 m: on measured raw data the operator's body/handle shows up only as small blobs inside 0.4 m.
        double near_min_range_m = 0.4;
        static constexpr double kNearMinRangeLowest = 0.1;
        static constexpr double kNearMinRangeHighest = 0.95; // must stay below the 1.0 m load filter

        //! Keep the step 1 staging/preview folders after a successful run (for debugging).
        bool keep_work_files = false;

        void clamp()
        {
            near_min_range_m = std::clamp(near_min_range_m, kNearMinRangeLowest, kNearMinRangeHighest);
        }
    };
} // namespace easy_gui

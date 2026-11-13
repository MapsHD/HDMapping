#pragma once

// Scrubbable time ruler with ticks and a playhead, shared between
// camera_lidar_trajectory_viewer's flyover bar and mandeye_raw_data_viewer's
// playback bar.
namespace raylib_widgets
{

    //! Timeline across the available width, with time ticks and a playhead.
    //! Click or drag on it to scrub.
    //! @param id ImGui id of the scrub area
    //! @param progress 0-1 playhead position; written while the user drags
    //! @param durationSec time the full width spans, for the tick labels
    //! @param height total height, including the tick label row
    //! @return true on the frames the user moved the playhead
    bool timeline(const char* id, float& progress, double durationSec, float height);

} // namespace raylib_widgets

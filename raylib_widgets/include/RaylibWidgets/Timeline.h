#pragma once

#include <string>
#include <vector>

// Scrubbable time ruler with ticks and a playhead, shared between
// camera_lidar_trajectory_viewer's flyover bar and mandeye_raw_data_viewer's
// playback bar.
namespace raylib_widgets
{

    //! A band marked across the timeline's track, e.g. a segment of interest.
    struct TimelineSpan
    {
        float from = 0.f, to = 0.f; //!< 0-1 positions along the timeline
        unsigned int color = 0; //!< ImU32, drawn over the track; give it some transparency
        std::string label; //!< added to the hover tooltip over the band; may be empty
    };

    //! A row of bars under the timeline's track, e.g. one bar per input file, Gantt-style.
    struct TimelineLane
    {
        std::string name; //!< drawn at the lane's left end; may be empty
        std::vector<TimelineSpan> bars; //!< in time order; their labels name them in the tooltip
    };

    //! Height of one TimelineLane [px].
    inline constexpr float kTimelineLaneHeight = 12.f;

    //! Timeline across the available width, with time ticks and a playhead.
    //! Click or drag on it to scrub.
    //! @param id ImGui id of the scrub area
    //! @param progress 0-1 playhead position; written while the user drags
    //! @param durationSec time the full width spans, for the tick labels
    //! @param height height of the track and the tick label row
    //! @param spans bands to mark on the track
    //! @param lanes rows of bars between the track and the tick labels, each adding
    //!        kTimelineLaneHeight to the height; hovering between two bars shows the gap
    //! @return true on the frames the user moved the playhead
    bool timeline(
        const char* id,
        float& progress,
        double durationSec,
        float height,
        const std::vector<TimelineSpan>& spans = {},
        const std::vector<TimelineLane>& lanes = {});

} // namespace raylib_widgets

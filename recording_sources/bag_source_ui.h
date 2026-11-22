#pragma once

#include "bag_source.h"

#include <filesystem>
#include <string>
#include <vector>

//! ImGui form that builds the BagSourceConfig of one ROS1 bag or MCAP file: lidar and IMU topic pickers filled from the
//! file's topics (the only one of each type preselected), PointCloud2 layout, IMU acceleration unit and rotation into the
//! lidar frame, and chunk length.
class BagSourceConfigUi
{
public:
    //! Reads the file's topic list.
    explicit BagSourceConfigUi(const std::filesystem::path& file);

    //! Draws the form into the current ImGui window. Returns true on the frame the user presses Open with a usable
    //! configuration.
    bool draw();

    const BagSourceConfig& config() const
    {
        return config_;
    }

private:
    BagSourceConfig config_;
    std::vector<BagChannel> channels_;
    std::string list_error_;
};

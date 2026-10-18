#pragma once

#include <filesystem>
#include <string>

struct LidarOdometryParams;

namespace easy_gui
{
    //! "Presets -> 2 Velocity < 8km/h, larger spaces, precise forestry (generic)" of lidar_odometry_step_1.
    //!
    //! NOTE: the preset only exists as inline GUI code (lidar_odometry_gui.cpp, and a copy in
    //! drag_folder_with_mandeye_data_and_drop_here-precision_forestry.cpp), so the values are repeated here. They
    //! overwrite every field set_lidar_odometry_default_params() touches, so the result equals "start the step 1 GUI,
    //! pick preset 2" regardless of which app's default function ran first. Keep in sync with lidar_odometry_gui.cpp.
    void applyPrecisionForestryPreset(LidarOdometryParams& params);

    //! Easy GUI's only deviation from the preset: threshould_output_filter (output-only) is lowered to
    //! filter_threshold_xy_inner, so step 1 exports every point the load filter kept, and the near-point export picks
    //! up exactly the points the load filter removed. The trajectory is unaffected (see docs/DESIGN.md).
    void applyEasyGuiOutputSettings(LidarOdometryParams& params);

    //! Writes the parameter file the step 1 child process runs with (value-initialised params + preset + Easy GUI
    //! output settings), through step 1's own TomlIO so every serialisable field is present.
    bool writeStep1ParameterFile(const std::filesystem::path& file, std::string& error);
} // namespace easy_gui

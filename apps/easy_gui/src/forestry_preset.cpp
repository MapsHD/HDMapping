#include "forestry_preset.h"

#include <toml_io.h>

#include <memory>

namespace easy_gui
{
    void applyPrecisionForestryPreset(LidarOdometryParams& params)
    {
        params.decimation = 0.01;
        params.in_out_params_indoor.resolution_X = 0.1;
        params.in_out_params_indoor.resolution_Y = 0.1;
        params.in_out_params_indoor.resolution_Z = 0.1;

        params.in_out_params_outdoor.resolution_X = 0.3;
        params.in_out_params_outdoor.resolution_Y = 0.3;
        params.in_out_params_outdoor.resolution_Z = 0.3;

        params.filter_threshold_xy_inner = 1.0;
        params.filter_threshold_xy_outer = 70.0;
        params.threshould_output_filter = 1.5;

        params.use_robust_and_accurate_lidar_odometry = false;
        params.distance_bucket = 0.2;
        params.polar_angle_deg = 10.0;
        params.azimutal_angle_deg = 10.0;
        params.robust_and_accurate_lidar_odometry_iterations = 20;

        params.max_distance_lidar = 70.0;
        params.nr_iter = 500;
        params.sliding_window_trajectory_length_threshold = 10000;
        params.real_time_threshold_seconds = 10;
    }

    void applyEasyGuiOutputSettings(LidarOdometryParams& params)
    {
        params.threshould_output_filter = params.filter_threshold_xy_inner;
    }

    bool writeStep1ParameterFile(const std::filesystem::path& file, std::string& error)
    {
        // Heap + value-initialisation, as lidar_odometry_gui.cpp's --dump-default-params does: the NDT grid members
        // carry no initialisers (and would be written to the TOML), and the struct is too large for comfort on the
        // stack of the pipeline thread.
        auto params = std::make_unique<LidarOdometryParams>();
        applyPrecisionForestryPreset(*params);
        applyEasyGuiOutputSettings(*params);

        TomlIO toml_io;
        if (!toml_io.SaveParametersToTomlFile(file.string(), *params))
        {
            error = "Could not write the processing settings file '" + file.string() + "'.";
            return false;
        }
        return true;
    }
} // namespace easy_gui

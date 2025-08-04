# HDMapping TOML Configuration Guide

## 🚀 Overview

HDMapping uses TOML (Tom's Obvious, Minimal Language) configuration files for various applications and tools. TOML provides a clean, readable format for configuration that is both human-friendly and machine-parseable.

## 📋 TOML Structure in HDMapping

### Configuration File Locations

TOML configuration files are used in several HDMapping applications:

- **Lidar Odometry:** `apps/lidar_odometry_step_1/` - Main SLAM parameters
- **Multi-session Registration:** `apps/multi_session_registration/` - Registration settings
- **Trajectory Comparison:** `apps/compare_trajectories/` - Comparison parameters
- **Manual Coloring:** `apps/manual_color/` - Visualization settings

### Common TOML Sections

#### Motion Model Parameters
```toml
[motion_model_uncertainty]
lidar_odometry_motion_model_fi_1_sigma_deg = 0.01
lidar_odometry_motion_model_ka_1_sigma_deg = 0.01
lidar_odometry_motion_model_om_1_sigma_deg = 0.01

[motion_model_correction]
motion_model_correction_fi = 0.0
motion_model_correction_ka = 0.0
motion_model_correction_om = 0.0
```

#### Algorithm Settings
```toml
[icp_settings]
search_radius = 1.0
number_of_iterations = 100
convergence_threshold = 0.001

[point_cloud_processing]
voxel_size = 0.1
normal_estimation_radius = 0.5
outlier_removal_threshold = 2.0
```

#### File Input/Output
```toml
[file_paths]
input_directory = "/path/to/input/data"
output_directory = "/path/to/output/results"
trajectory_file = "trajectory.txt"
point_cloud_format = "las"
```

## 🔧 Working with TOML Files

### Creating Configuration Files

1. **Start with a template:**
```toml
# HDMapping Configuration File
# Generated on: 2025-01-01

[general]
application = "lidar_odometry"
version = "0.85.0"
```

2. **Add specific sections as needed:**
```toml
[processing_parameters]
enable_loop_closure = true
max_distance_threshold = 10.0
min_points_per_voxel = 5
```

### Loading TOML in Applications

HDMapping applications automatically load TOML configuration files using the integrated TOML parser. The typical workflow:

1. **Check for configuration file**
2. **Load default values if file not found**
3. **Parse TOML and override defaults**
4. **Validate parameter ranges**
5. **Report configuration status**

### Configuration Validation

HDMapping includes built-in validation for TOML configurations:

- **Type checking:** Ensures parameters are correct types (float, int, string, bool)
- **Range validation:** Checks values are within acceptable ranges
- **Required parameters:** Verifies essential parameters are present
- **Version compatibility:** Warns about configuration format changes

## 📝 TOML Best Practices

### Organization
```toml
# Group related parameters together
[motion_model_uncertainty]
# All uncertainty parameters here

[motion_model_correction]
# All correction parameters here

[algorithm_settings]
# Algorithm-specific settings here
```

### Naming Conventions
```toml
# Use clear, descriptive names
search_radius = 1.0                    # Good
sr = 1.0                              # Avoid

# Use consistent naming patterns
lidar_odometry_motion_model_fi_1_sigma_deg = 0.01    # Consistent
motion_model_uncertainty_fi = 0.01                   # Also good
```

### Comments and Documentation
```toml
# Motion model uncertainty parameters (in degrees)
[motion_model_uncertainty]
lidar_odometry_motion_model_fi_1_sigma_deg = 0.01    # Roll uncertainty
lidar_odometry_motion_model_ka_1_sigma_deg = 0.01    # Pitch uncertainty  
lidar_odometry_motion_model_om_1_sigma_deg = 0.01    # Yaw uncertainty
```

### Data Types
```toml
# Strings - use quotes
output_format = "las"
input_directory = "/path/to/data"

# Numbers - no quotes
search_radius = 1.0          # Float
max_iterations = 100         # Integer

# Booleans
enable_visualization = true
save_intermediate = false

# Arrays
file_extensions = ["las", "laz", "pcd"]
threshold_values = [1.0, 2.0, 3.0]
```

## 🔍 Common Configuration Examples

### Lidar Odometry Configuration
```toml
# Lidar Odometry Configuration
[general]
application = "lidar_odometry_step_1"
description = "SLAM processing configuration"

[motion_model_uncertainty]
lidar_odometry_motion_model_fi_1_sigma_deg = 0.01
lidar_odometry_motion_model_ka_1_sigma_deg = 0.01
lidar_odometry_motion_model_om_1_sigma_deg = 0.01

[motion_model_correction]
motion_model_correction_fi = 0.0
motion_model_correction_ka = 0.0
motion_model_correction_om = 0.0

[icp_parameters]
search_radius = 1.0
number_of_iterations = 100
convergence_threshold = 0.001

[point_cloud_processing]
voxel_size = 0.1
normal_estimation_radius = 0.5
outlier_removal = true
outlier_threshold = 2.0
```

### Multi-Session Registration
```toml
# Multi-Session Registration Configuration
[general]
application = "multi_session_registration"
version = "0.85.0"

[registration_settings]
max_correspondence_distance = 2.0
transformation_epsilon = 1e-6
euclidean_fitness_epsilon = 1e-6
max_iterations = 100

[point_cloud_filters]
statistical_outlier_removal = true
radius_outlier_removal = true
voxel_grid_filter = true
voxel_size = 0.1

[output_settings]
save_transformed_clouds = true
save_transformation_matrices = true
output_format = "las"
```

### Trajectory Comparison
```toml
# Trajectory Comparison Configuration
[general]
application = "compare_trajectories"
description = "Trajectory analysis and comparison"

[comparison_parameters]
temporal_alignment = true
spatial_alignment = false
max_time_difference = 1.0
max_spatial_distance = 5.0

[analysis_settings]
compute_rms_error = true
compute_absolute_trajectory_error = true
compute_relative_pose_error = true
sampling_interval = 1.0

[visualization]
plot_trajectories = true
save_plots = true
plot_format = "png"
show_statistics = true
```

## 🚨 Troubleshooting TOML Issues

### Common Parsing Errors

#### 1. Syntax Errors
```toml
# Wrong - missing quotes for strings
output_directory = /path/to/output

# Correct
output_directory = "/path/to/output"
```

#### 2. Type Mismatches
```toml
# Wrong - string where number expected
search_radius = "1.0"

# Correct
search_radius = 1.0
```

#### 3. Invalid Section Names
```toml
# Wrong - spaces in section names
[motion model uncertainty]

# Correct
[motion_model_uncertainty]
```

### Validation Errors

If you encounter validation errors:

1. **Check parameter types** match expected values
2. **Verify parameter names** are spelled correctly
3. **Ensure required sections** are present
4. **Check value ranges** are within acceptable limits

### Configuration File Not Found

If configuration file is missing:

1. **Application will use defaults** and continue
2. **Check file path** and name spelling
3. **Verify file permissions** are readable
4. **Create template** from examples above

## 📋 TOML File Management

### Version Control
- **Include configuration files** in version control
- **Use meaningful commit messages** when changing configurations
- **Document parameter changes** in commit descriptions
- **Keep backup copies** of working configurations

### Environment-Specific Configurations
```bash
# Development configuration
config_dev.toml

# Production configuration  
config_prod.toml

# Testing configuration
config_test.toml
```

### Configuration Templates
Create template files for common use cases:
```bash
# Copy template and customize
cp templates/lidar_odometry_template.toml my_config.toml
# Edit my_config.toml as needed
```

## 🔗 Integration with HDMapping Applications

### Command Line Usage
```bash
# Specify configuration file
./lidar_odometry_step_1 --config my_config.toml

# Use default configuration
./lidar_odometry_step_1

# Override specific parameters
./lidar_odometry_step_1 --config my_config.toml --search-radius 2.0
```

### Programmatic Access
HDMapping applications access TOML configurations through the integrated parser:

```cpp
// Load configuration
auto config = load_toml_config("config.toml");

// Access parameters
double search_radius = config["icp_parameters"]["search_radius"].as<double>();
bool enable_viz = config["visualization"]["enable"].as<bool>();
```

## 📞 Support

For TOML configuration issues:

1. **Validate TOML syntax** using online TOML validators
2. **Check application logs** for parsing error messages
3. **Compare with working examples** in documentation
4. **Report configuration-related bugs** with sample TOML files

---

*This guide covers the TOML configuration system used throughout HDMapping applications for flexible, maintainable parameter management.*

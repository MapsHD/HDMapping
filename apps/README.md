# HDMapping Applications Reference

## 📋 Application Overview

HDMapping includes 15 specialized applications for different aspects of point cloud processing and analysis. Each application has its own README.md with specific documentation.

## 🔧 Core Processing Pipeline

### Primary Workflow Applications
| Application | Purpose | Input | Output | Documentation Status |
|-------------|---------|-------|--------|---------------------|
| **[hd_mapper](hd_mapper/README.md)** | Main GUI application | Point clouds, config | Maps, trajectories | ⚠️ Basic |
| **[lidar_odometry_step_1](lidar_odometry_step_1/README.md)** | Initial trajectory estimation | Raw lidar data | Session file, initial trajectory | ✅ Updated v0.85 |
| **[multi_view_tls_registration](multi_view_tls_registration/README.md)** | Final registration & mapping | Step 1 session | Final map, trajectory | ✅ Updated v0.85 |
| **[quick_start_demo](quick_start_demo/README.md)** | Tutorial and demonstration | Sample data | Demo results | ⚠️ Basic |

## 👁️ Visualization & Analysis

### Data Viewers
| Application | Purpose | Input | Output | Documentation Status |
|-------------|---------|-------|--------|---------------------|
| **[mandeye_raw_data_viewer](mandeye_raw_data_viewer/README.md)** | Raw data visualization | Raw MandEye data | Quality reports | ⚠️ Basic |
| **[mandeye_single_session_viewer](mandeye_single_session_viewer/README.md)** | Session analysis | Session files | Analysis reports | ⚠️ Basic |

### Analysis Tools
| Application | Purpose | Input | Output | Documentation Status |
|-------------|---------|-------|--------|---------------------|
| **[compare_trajectories](compare_trajectories/README.md)** | Trajectory comparison | Trajectory files | Comparison reports | ⚠️ Basic |

## 🎯 Calibration & Configuration

### Sensor Calibration
| Application | Purpose | Input | Output | Documentation Status |
|-------------|---------|-------|--------|---------------------|
| **[livox_mid_360_intrinsic_calibration](livox_mid_360_intrinsic_calibration/README.md)** | Livox sensor calibration | Calibration scans | Calibration parameters | ⚠️ Basic |
| **[mandeye_mission_recorder_calibration](mandeye_mission_recorder_calibration/README.md)** | MandEye calibration | Mission data | Calibration config | ⚠️ Basic |

## 🛠️ Utility Applications

### Processing Tools
| Application | Purpose | Input | Output | Documentation Status |
|-------------|---------|-------|--------|---------------------|
| **[multi_session_registration](multi_session_registration/README.md)** | Multi-session fusion | Multiple sessions | Fused map | ⚠️ Basic |
| **[split_multi_livox](split_multi_livox/README.md)** | Livox data splitting | Multi-sensor data | Individual streams | ✅ Updated v0.85 |
| **[precision_forestry_tools](precision_forestry_tools/README.md)** | Forestry analysis | Forest point clouds | Tree metrics | ⚠️ Basic |

### Visualization Tools
| Application | Purpose | Input | Output | Documentation Status |
|-------------|---------|-------|--------|---------------------|
| **[manual_color](manual_color/README.md)** | Manual colorization | Point clouds | Colored clouds | ⚠️ Basic |
| **[single_session_manual_coloring](single_session_manual_coloring/README.md)** | Session colorization | Session data | Colored session | ⚠️ Basic |

### Development Tools
| Application | Purpose | Input | Output | Documentation Status |
|-------------|---------|-------|--------|---------------------|
| **[matrix_mul](matrix_mul/README.md)** | Matrix operations benchmark | Test matrices | Performance metrics | ⚠️ Basic |

## 🚀 Quick Start Recommendations

### For New Users:
1. **Start with [quick_start_demo](quick_start_demo/README.md)** - Learn the workflow
2. **Use [hd_mapper](hd_mapper/README.md)** - Main GUI application
3. **Try command-line pipeline** - Step 1 → Step 2 workflow

### For Advanced Users:
1. **[lidar_odometry_step_1](lidar_odometry_step_1/README.md)** + **[multi_view_tls_registration](multi_view_tls_registration/README.md)** - Command-line pipeline
2. **[compare_trajectories](compare_trajectories/README.md)** - Quality analysis
3. **Calibration tools** - Sensor optimization

### For Developers:
1. **Python bindings** - See [PYBIND documentation](../docs/PYBIND_QUICK_START.md)
2. **[matrix_mul](matrix_mul/README.md)** - Performance benchmarking
3. **Custom applications** - Use HDMapping libraries

## 📖 Documentation Strategy

### Documentation Status Legend:
- ✅ **Complete** - Full documentation with examples
- ⚠️ **Basic** - Template with essential information
- 📝 **TODO** - Needs documentation

### Contributing Documentation:
1. **Use the [template](../docs/APP_DOCUMENTATION_TEMPLATE.md)** for new applications
2. **Start with basic info** - Purpose, input/output, key parameters
3. **Expand gradually** - Add details as you use the applications
4. **Include examples** - Real usage scenarios

## 🔗 Related Documentation

- **[CPU Optimization Guide](../docs/CPU_OPTIMIZATION_GUIDE.md)** - Performance tuning
- **[CMake Configuration](../docs/CMAKE_CONFIGURATION.md)** - Build configuration
- **[Python Bindings](../docs/PYBIND_QUICK_START.md)** - Programmatic access
- **[TOML Configuration](../docs/TOML_CONFIGURATION_GUIDE.md)** - Configuration files

---

*This reference is continuously updated as applications are documented. Contributions welcome!*

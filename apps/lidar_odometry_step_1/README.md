# Lidar Odometry Step 1

## Purpose
First step of HDMapping pipeline: processes raw MANDEYE lidar data to generate initial trajectory estimates using lidar odometry algorithms with IMU fusion.

## Quick Start
```bash
# GUI mode (recommended for configuration)
./lidar_odometry_step_1

# Command line processing
./lidar_odometry_step_1 --config config.toml
```

## Input/Output
- **Input**: 
  - Raw MANDEYE lidar files (.laz, .las) 
  - IMU data (.csv files)
  - Calibration files (calibration.json, .sn files)
- **Output**: 
  - Initial trajectory estimates (.csv)
  - Processed point clouds (.laz)
  - Session configuration file (session.json)
  - Preview point clouds (preview/ folder)

## Key Features
- **MANDEYE Integration**: Native support for MANDEYE multi-sensor data
- **IMU Fusion**: AHRS-based sensor fusion with configurable gain
- **Calibration Support**: Automatic loading of sensor calibration files
- **Export Functionality**: Unified export system via core export functions
- **Preview Generation**: Automatic preview point clouds for verification
- **Session Management**: Complete session save/load functionality

## Workflow
1. **Load Data**: Select all .laz and .csv files from MANDEYE 'continousScanning_*' folder
2. **Configure**: Adjust AHRS gain and processing parameters
3. **Process**: Run lidar odometry with motion estimation
4. **Export**: Save results including session.json for Step 2

## Key Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `ahrs_gain` | 0.5 | AHRS filter gain (0.1-0.3 for dynamic motion) |
| `nr_iter` | 100 | Maximum NDT optimization iterations |
| `filter_threshold_xy` | 2.0 | Point filtering threshold in XY plane |
| `point_count_threshold` | 50000 | Batch size for point processing |
| `fusionConvention` | NWU | Coordinate system (NWU/ENU/NED) |

## File Structure Requirements
```
working_directory/
├── scan_*.laz              # MANDEYE point cloud files
├── imu_*.csv              # IMU data files  
├── *.sn                   # Sensor serial number mapping
└── calibration.json       # Sensor extrinsic calibrations
```

## Export Outputs
After processing, the following files are generated:
- `scan_lio_*.laz` - Processed point clouds with trajectory
- `trajectory_lio_*.csv` - Detailed trajectory data
- `session.json` - Complete session for Step 2
- `lio_initial_poses.reg` - Initial pose estimates  
- `poses.reg` - Optimized pose estimates
- `preview/` - Preview point clouds for validation

## Usage Example
```bash
# GUI workflow (recommended)
./lidar_odometry_step_1
# 1. Use "load data (step 1)" button to select .laz and .csv files
# 2. Adjust ahrs_gain if needed (0.3 for dynamic motion)
# 3. Press "calculate motion (step 2)" 
# 4. Press "save result (step 3)" to export session

# Export additional data
# Use "save all point clouds to single file" for combined output
# Use "save trajectory to ascii" for simple trajectory export
```

## Integration with Multi-View Registration
```bash
# After Step 1 completion, session.json can be used in Step 2:
./multi_view_tls_registration --session working_directory/session.json
```

## Notes
- **MANDEYE Focus**: Optimized for MANDEYE multi-sensor lidar systems
- **Export Centralization**: Uses unified export functions from core library (PR #160)
- **Session Continuity**: Output session.json required for multi_view_tls_registration
- **Memory Management**: Automatic batching for large datasets
- **Preview System**: Built-in validation through preview point clouds

## Troubleshooting
- **File Selection**: Ensure equal number of .laz and .csv files
- **Calibration**: calibration.json must be in the same directory as data files
- **Memory Issues**: Reduce point_count_threshold for large datasets
- **Dynamic Motion**: Use lower ahrs_gain (0.1-0.3) for vehicles/UAVs

---
*Documentation status: ✅ Updated for HDMapping v0.85.0 with PR #160 export centralization*
- [ ] Add troubleshooting for common issues
- [ ] Add performance optimization guidelines
-->

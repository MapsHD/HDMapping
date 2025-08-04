# Lidar Odometry Step 1

## Purpose
First step of HDMapping pipeline: processes raw lidar data to generate initial trajectory estimates using lidar odometry algorithms.

## Quick Start
```bash
# Basic processing
./lidar_odometry_step_1 --input data/ --output step1_results/

# With GUI for parameter tuning
./lidar_odometry_step_1 --gui
```

## Input/Output
- **Input**: Raw lidar point cloud files (.laz, .las), IMU data (optional)
- **Output**: Initial trajectory (.csv), processed point clouds, session file (.json)

## Key Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `--input` | - | Directory with input lidar files |
| `--output` | "./output" | Output directory |
| `--decimation` | 0.01 | Point cloud decimation factor (0.001-0.1) |
| `--max-distance` | 30.0 | Maximum lidar range (meters) |
| `--iterations` | 100 | Maximum optimization iterations |

## Usage Example
```bash
# Process dataset with custom parameters
./lidar_odometry_step_1 \
  --input /data/lidar_scan/ \
  --output /results/step1/ \
  --decimation 0.02 \
  --max-distance 50.0
```

## Notes
- First step in the HDMapping pipeline (followed by multi_view_tls_registration)
- Output session file needed for Step 2 processing
- Memory usage scales with point cloud density

---
*Documentation status: ⚠️ Basic template - needs detailed documentation*

<!-- 
TODO for detailed documentation:
- [ ] Add complete parameter reference
- [ ] Add algorithm details and parameter tuning
- [ ] Add integration with Step 2
- [ ] Add troubleshooting for common issues
- [ ] Add performance optimization guidelines
-->

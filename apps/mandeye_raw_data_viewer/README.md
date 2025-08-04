# MandEye Raw Data Viewer

## Purpose
Visualization tool for viewing and analyzing raw MandEye lidar data before processing, with real-time preview capabilities.

## Quick Start
```bash
# View raw data directory
./mandeye_raw_data_viewer --input raw_data/

# Interactive viewer
./mandeye_raw_data_viewer --gui
```

## Input/Output
- **Input**: Raw MandEye lidar files, sensor configuration files
- **Output**: Data quality reports, preview images, filtered datasets

## Key Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `--input` | - | Raw data directory |
| `--config` | - | Sensor configuration file |
| `--filter` | false | Apply basic filtering |
| `--export` | - | Export filtered data path |

## Usage Example
```bash
# View and analyze raw data
./mandeye_raw_data_viewer \
  --input /data/raw_mandeye/ \
  --config sensor_config.toml \
  --filter
```

## Notes
- Designed specifically for MandEye sensor data format
- Useful for data quality assessment before processing
- Helps identify sensor calibration issues
- Supports real-time data preview

---
*Documentation status: ⚠️ Basic template - needs detailed documentation*

<!-- 
TODO for detailed documentation:
- [ ] Add MandEye data format details
- [ ] Add quality assessment criteria
- [ ] Add filtering options
- [ ] Add export format options
-->

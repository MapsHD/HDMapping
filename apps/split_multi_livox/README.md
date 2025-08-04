# Split Multi-Livox

## Purpose
Utility for splitting multi-sensor Livox data streams into individual sensor files for separate processing or analysis.

## Quick Start
```bash
# Split multi-sensor data
./split_multi_livox --input multi_sensor_data/ --output split_data/

# Specify sensor count
./split_multi_livox --input data/ --sensors 4 --output results/
```

## Input/Output
- **Input**: Multi-sensor Livox data files, sensor configuration
- **Output**: Individual sensor data files, sensor mapping information

## Key Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `--input` | - | Multi-sensor data directory |
| `--output` | "./split_output" | Output directory |
| `--sensors` | auto | Number of sensors (auto-detect) |
| `--format` | "laz" | Output format (laz/las) |

## Usage Example
```bash
# Split 4-sensor setup
./split_multi_livox \
  --input /data/multi_livox/ \
  --output /data/individual_sensors/ \
  --sensors 4
```

## Notes
- Preserves temporal synchronization between sensors
- Useful for individual sensor calibration
- Enables parallel processing of sensor streams
- Maintains sensor ID and timing information

---
*Documentation status: 📝 TODO - needs detailed documentation*

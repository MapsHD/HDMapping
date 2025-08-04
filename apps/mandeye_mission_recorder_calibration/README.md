# Mandeye Mission Recorder Calibration

## Purpose
Calibration utility for Mandeye mission recording systems to ensure accurate data synchronization and sensor alignment.

## Quick Start
```bash
# Start calibration wizard
./mandeye_mission_recorder_calibration --input mission_data/ --output calibration.toml

# Quick calibration check
./mandeye_mission_recorder_calibration --check --config existing_calibration.toml
```

## Input/Output
- **Input**: Mission recording data, calibration targets
- **Output**: Calibration parameters, validation reports

## Key Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `--input` | - | Mission data directory |
| `--output` | "calibration.toml" | Calibration file |
| `--mode` | "auto" | Calibration mode |
| `--check` | false | Validation mode |
| `--targets` | - | Calibration targets |

## Usage Example
```bash
# Full calibration procedure
./mandeye_mission_recorder_calibration \
  --input /data/mission_raw/ \
  --output /config/mission_cal.toml \
  --mode full_auto
```

## Notes
- Essential for mission data accuracy
- Supports multiple calibration methods
- Provides calibration quality metrics
- Enables mission data validation

---
*Documentation status: 📝 TODO - needs detailed documentation*

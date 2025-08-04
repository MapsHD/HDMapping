# Livox MID-360 Intrinsic Calibration

## Purpose
Calibration tool for Livox MID-360 lidar sensors to determine intrinsic parameters and correct systematic measurement errors.

## Quick Start
```bash
# Calibrate with calibration target data
./livox_mid_360_intrinsic_calibration --target calibration_data/ --output calibration.yaml

# Interactive calibration mode
./livox_mid_360_intrinsic_calibration --gui --target data/
```

## Input/Output
- **Input**: Calibration target scan data, sensor configuration
- **Output**: Calibration parameters (.yaml), calibration report, corrected data

## Key Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `--target` | - | Calibration target scan directory |
| `--output` | "calibration.yaml" | Output calibration file |
| `--target-size` | 1.0 | Calibration target size (meters) |
| `--iterations` | 100 | Calibration optimization iterations |

## Usage Example
```bash
# Full calibration procedure
./livox_mid_360_intrinsic_calibration \
  --target /data/calibration_scans/ \
  --output /config/livox_calibration.yaml \
  --target-size 0.5
```

## Notes
- Requires calibration target scans from multiple positions
- Improves measurement accuracy significantly
- Essential for high-precision mapping applications
- Calibration parameters sensor-specific

---
*Documentation status: ⚠️ Basic template - needs detailed documentation*

<!-- 
TODO for detailed documentation:
- [ ] Add calibration procedure details
- [ ] Add target requirements and setup
- [ ] Add calibration quality assessment
- [ ] Add parameter application workflow
-->

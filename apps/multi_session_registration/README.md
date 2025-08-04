# Multi-Session Registration

## Purpose
Advanced registration and alignment of multiple scanning sessions for creating comprehensive 3D maps from overlapping datasets.

## Quick Start
```bash
# Register multiple sessions
./multi_session_registration --sessions session1.laz session2.laz session3.laz --output merged.laz

# With configuration file
./multi_session_registration --config multi_session_config.toml
```

## Input/Output
- **Input**: Multiple session point clouds, optional ground control points
- **Output**: Globally registered point cloud, transformation matrices

## Key Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `--sessions` | - | Input session files |
| `--output` | "registered.laz" | Output file |
| `--config` | - | Configuration file |
| `--registration_type` | "icp" | Registration method |
| `--overlap_threshold` | 0.1 | Minimum overlap ratio |

## Usage Example
```bash
# Register with ICP
./multi_session_registration \
  --sessions /data/session*.laz \
  --output /results/merged_map.laz \
  --registration_type icp
```

## Notes
- Supports multiple registration algorithms
- Handles large-scale datasets efficiently
- Provides quality metrics for registration
- Essential for multi-day mapping campaigns

---
*Documentation status: 📝 TODO - needs detailed documentation*

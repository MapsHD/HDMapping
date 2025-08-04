# MandEye Single Session Viewer

## Purpose
Specialized viewer for analyzing individual MandEye scanning sessions with trajectory visualization and point cloud inspection tools.

## Quick Start
```bash
# View single session
./mandeye_single_session_viewer --session session.json

# Load with trajectory
./mandeye_single_session_viewer --session session.json --trajectory path.csv
```

## Input/Output
- **Input**: MandEye session files (.json), trajectory data (.csv), point clouds
- **Output**: Session analysis reports, trajectory plots, quality metrics

## Key Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `--session` | - | Session file to analyze |
| `--trajectory` | - | Trajectory file (optional) |
| `--output` | "./analysis" | Output directory for reports |
| `--interactive` | true | Enable interactive viewing |

## Usage Example
```bash
# Analyze session with trajectory
./mandeye_single_session_viewer \
  --session /data/session_001.json \
  --trajectory /data/trajectory_001.csv \
  --output /analysis/session_001/
```

## Notes
- Optimized for single session analysis workflows
- Provides detailed trajectory and data quality metrics
- Interactive 3D visualization capabilities
- Useful for session validation and troubleshooting

---
*Documentation status: ⚠️ Basic template - needs detailed documentation*

<!-- 
TODO for detailed documentation:
- [ ] Add session file format details
- [ ] Add visualization controls
- [ ] Add quality metrics explanation
- [ ] Add export options
-->

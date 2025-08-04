# Manual Color

## Purpose
Interactive tool for manual color assignment and correction of point cloud data with visual feedback and editing capabilities.

## Quick Start
```bash
# Start manual coloring
./manual_color --input pointcloud.laz --output colored.laz

# With specific color palette
./manual_color --input data.laz --palette custom_colors.txt --output result.laz
```

## Input/Output
- **Input**: Point cloud data, optional color palettes
- **Output**: Manually colored point cloud, color assignments

## Key Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `--input` | - | Input point cloud |
| `--output` | "colored.laz" | Output file |
| `--palette` | - | Custom color palette |
| `--brush_size` | 1.0 | Selection brush size |
| `--gui` | true | Enable GUI interface |

## Usage Example
```bash
# Interactive coloring session
./manual_color \
  --input /data/scan.laz \
  --output /data/colored_scan.laz \
  --gui true
```

## Notes
- Provides interactive 3D visualization
- Supports various selection tools
- Useful for manual classification correction
- Enables artistic point cloud coloring

---
*Documentation status: 📝 TODO - needs detailed documentation*

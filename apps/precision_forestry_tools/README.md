# Precision Forestry Tools

## Purpose
Specialized tools for forest analysis including tree detection, forest inventory, and biomass estimation from LiDAR data.

## Quick Start
```bash
# Tree detection
./precision_forestry_tools --mode tree_detection --input forest.laz --output trees.json

# Forest inventory
./precision_forestry_tools --mode inventory --input forest.laz --output inventory.csv
```

## Input/Output
- **Input**: Forest LiDAR point clouds, optional DTM
- **Output**: Tree positions, forest metrics, inventory reports

## Key Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `--mode` | "tree_detection" | Analysis mode |
| `--input` | - | Input point cloud |
| `--output` | - | Output file |
| `--min_tree_height` | 2.0 | Minimum tree height (m) |
| `--crown_detection` | true | Enable crown detection |

## Usage Example
```bash
# Complete forest analysis
./precision_forestry_tools \
  --mode full_analysis \
  --input /data/forest_scan.laz \
  --output /results/forest_analysis/ \
  --min_tree_height 3.0
```

## Notes
- Supports various tree species parameters
- Generates detailed forest inventory reports
- Useful for sustainable forest management
- Provides biomass and carbon estimates

---
*Documentation status: 📝 TODO - needs detailed documentation*

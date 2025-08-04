# HDMapping Core Application

## Purpose
Main HDMapping application for high-definition point cloud mapping and processing. Provides GUI interface for interactive mapping workflows.

## Quick Start
```bash
# Launch GUI
./hd_mapper

# Command line processing
./hd_mapper --input data/ --output results/ --config config.toml
```

## Input/Output
- **Input**: Point cloud data (.laz, .las), configuration files (.toml)
- **Output**: Processed maps (.las), trajectory files (.csv), session data (.json)

## Key Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `--input` | - | Input directory with point cloud files |
| `--output` | "./output" | Output directory for results |
| `--config` | - | Configuration file (TOML format) |
| `--gui` | true | Launch with GUI interface |

## Usage Example
```bash
# Interactive mode with GUI
./hd_mapper

# Batch processing
./hd_mapper --input /path/to/data --output /path/to/results --config mapping.toml --no-gui
```

## Notes
- Supports both interactive GUI and batch processing modes
- Configuration via TOML files for reproducible workflows
- Requires substantial RAM for large datasets (16GB+ recommended)

---
*Documentation status: ⚠️ Basic template - needs detailed documentation*

<!-- 
TODO for detailed documentation:
- [ ] Add complete parameter reference
- [ ] Add GUI workflow examples  
- [ ] Add configuration file documentation
- [ ] Add troubleshooting section
- [ ] Add performance guidelines
-->

# Single Session Manual Coloring

## Purpose
Manual coloring tool specifically designed for single scanning session data with session-aware color management and consistency.

## Quick Start
```bash
# Color single session
./single_session_manual_coloring --session session_data/ --output colored_session/

# With predefined color scheme
./single_session_manual_coloring --session data/ --scheme terrain --output result/
```

## Input/Output
- **Input**: Single session data directory, color schemes
- **Output**: Colored session data, color metadata

## Key Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `--session` | - | Session data directory |
| `--output` | "colored_session" | Output directory |
| `--scheme` | "default" | Color scheme |
| `--interactive` | true | Interactive mode |
| `--auto_save` | 300 | Auto-save interval (sec) |

## Usage Example
```bash
# Interactive session coloring
./single_session_manual_coloring \
  --session /data/session_001/ \
  --output /data/colored_session_001/ \
  --scheme vegetation
```

## Notes
- Optimized for single session workflows
- Maintains temporal consistency in coloring
- Supports session-specific color schemes
- Provides automatic backup functionality

---
*Documentation status: 📝 TODO - needs detailed documentation*
